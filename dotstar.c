/*------------------------------------------------------------------------
  Python module to control Adafruit Dot Star addressable RGB LEDs.

  This has some Known Issues(tm):

  It's modeled after the Adafruit_DotStar Arduino library (C++), for
  better or for worse.  The idea is that the majority of existing Arduino
  code for DotStar & NeoPixel LEDs can then port over with less fuss.
  As such, it's less "Python-like" than it could (and perhaps should)
  be...for example, RGB colors might be more elegantly expressed as
  tuples or something other than the packed 32-bit integers used here.
  Also, it does not have 100% feature parity with that library...e.g.
  getPixels() is missing here, and this code allows changing the SPI
  bitrate (Arduino lib does not).

  There's no doc strings yet.

  The library can use either hardware SPI or "bitbang" output....but one
  must be careful in the latter case not to overlap the SPI GPIO pins...
  once they're set as bitbang outputs by this code, they're no longer
  usable for SPI even after the code exits (and not just by this library;
  subsequent runs, other code, etc. all are locked out of SPI, only fix
  seems to be a reboot).  The library checks for an exact overlap between
  the requested bitbang data & clock pins and the hardware SPI pins, and
  will switch over to hardware SPI in that case...but partial overlaps
  (just the data -or- clock pin, or if their positions are swapped) are
  not protected.

  As of 9/15 this is using the empirical APA102 data format (rather than
  the datasheet specification).  If it suddenly starts misbehaving with
  new LEDs in the future, may be a hardware production change in the LEDs.

  Written by Phil Burgess for Adafruit Industries, with contributions from
  the open source community.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  ------------------------------------------------------------------------
  This file is part of the Adafruit Dot Star library.

  Adafruit Dot Star is free software: you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public License
  as published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  Adafruit Dot Star is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoPixel.  If not, see <http://www.gnu.org/licenses/>.
  ------------------------------------------------------------------------*/

#include <python2.7/Python.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>

// From GPIO example code by Dom and Gert van Loo on elinux.org:
#define PI1_BCM2708_PERI_BASE 0x20000000
#define PI1_GPIO_BASE         (PI1_BCM2708_PERI_BASE + 0x200000)
#define PI2_BCM2708_PERI_BASE 0x3F000000
#define PI2_GPIO_BASE         (PI2_BCM2708_PERI_BASE + 0x200000)
#define BLOCK_SIZE            (4*1024)
#define INP_GPIO(g)          *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g)          *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))

#define SPI_MOSI_PIN 10
#define SPI_CLK_PIN  11

static volatile unsigned
  *gpio = NULL, // Memory-mapped GPIO peripheral
  *gpioSet,     // Write bitmask of GPIO pins to set here
  *gpioClr;     // Write bitmask of GPIO pins to clear here

static uint8_t isPi2 = 0; // For clock pulse timing & stuff

// SPI transfer operation setup.  These are only used w/hardware SPI
// and LEDs at full brightness (or raw write); other conditions require
// per-byte processing.  Explained further in the show() method.
static struct spi_ioc_transfer xfer[3] = {
 { .tx_buf        = 0, // Header (zeros)
   .rx_buf        = 0,
   .len           = 4,
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 },
 { .rx_buf        = 0, // Color payload
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 },
 { .tx_buf        = 0, // Footer (zeros)
   .rx_buf        = 0,
   .delay_usecs   = 0,
   .bits_per_word = 8,
   .cs_change     = 0 }
};

typedef struct {             // Python object for DotStar strip
	PyObject_HEAD
	uint32_t numLEDs,    // Number of pixels in strip
	         dataMask,   // Data pin bitmask if using bitbang SPI
	         clockMask,  // Clock pin bitmask if bitbang SPI
	         bitrate;    // SPI clock speed if using hardware SPI
	int      fd;         // File descriptor if using hardware SPI
	uint8_t *pixels,     // -> pixel data
	        *pBuf,       // -> temp buf for brightness-scaling w/SPI
	         dataPin,    // Data pin # if bitbang SPI
	         clockPin,   // Clock pin # if bitbang SPI
	         brightness, // Global brightness setting
	         rOffset,    // Index of red in 4-byte pixel
	         gOffset,    // Index of green byte
	         bOffset;    // Index of blue byte
} DotStarObject;

// Allocate new DotStar object.  There's a few ways this can be called:
// x = Adafruit_DotStar(nleds, datapin, clockpin)       Bitbang output
// x = Adafruit_DotStar(nleds, bitrate)   Use hardware SPI @ bitrate
// x = Adafruit_DotStar(nleds)            Hardware SPI @ default rate
// x = Adafruit_DotStar()                 0 LEDs, HW SPI, default rate
// 0 LEDs is valid, but one must then pass a properly-sized and -rendered
// bytearray to the show() method.
static PyObject *DotStar_new(
  PyTypeObject *type, PyObject *arg, PyObject *kw) {
        DotStarObject *self     = NULL;
	uint8_t       *pixels   = NULL, dPin = 0xFF, cPin = 0xFF;
	uint32_t       n_pixels = 0, bitrate = 8000000, i;
	PyObject      *string;
	char          *order    = NULL, *c;
	uint8_t        rOffset = 2, gOffset = 3, bOffset = 1; // BRG default

	switch(PyTuple_Size(arg)) {
	   case 3: // Pixel count, data pin, clock pin
		if(!PyArg_ParseTuple(arg, "Ibb", &n_pixels, &dPin, &cPin))
			return NULL;
		// If pins happen to correspond to hardware SPI data and
		// clock, hardware SPI is used instead.  Because reasons.
		if((dPin == SPI_MOSI_PIN) && (cPin  == SPI_CLK_PIN))
			dPin = cPin = 0xFF;
		break;
	   case 2: // Pixel count, hardware SPI bitrate
		if(!PyArg_ParseTuple(arg, "II", &n_pixels, &bitrate))
			return NULL;
		break;
	   case 1: // Pixel count (hardware SPI w/default bitrate)
		if(!PyArg_ParseTuple(arg, "I", &n_pixels)) return NULL;
		break;
	   case 0: // No LED buffer (raw writes only), default SPI bitrate
		break;
	   default: // Unexpected number of arguments
		return NULL;
	}

	// Can optionally append keyword to specify R/G/B pixel order
	// "order='rgb'" or similar (switch r/g/b around to match strip).
	// Order string isn't much validated; nonsense may occur.
	if(kw && (string = PyDict_GetItemString(kw, "order")) &&
	  (order = PyString_AsString(string))) {
		for(i=0; order[i]; i++) order[i] = tolower(order[i]);
		if((c = strchr(order, 'r'))) rOffset = c - order + 1;
		if((c = strchr(order, 'g'))) gOffset = c - order + 1;
		if((c = strchr(order, 'b'))) bOffset = c - order + 1;
	}

	// Allocate space for LED data:
	if((!n_pixels) || ((pixels = (uint8_t *)malloc(n_pixels * 4)))) {
		if((self = (DotStarObject *)type->tp_alloc(type, 0))) {
			self->numLEDs    = n_pixels;
			self->dataMask   = 0;
			self->clockMask  = 0;
			self->bitrate    = bitrate;
			self->fd         = -1;
			self->pixels     = pixels; // NULL if 0 pixels
			self->pBuf       = NULL;   // alloc'd on 1st use
			self->dataPin    = dPin;
			self->clockPin   = cPin;
			self->brightness = 0;
			self->rOffset    = rOffset;
			self->gOffset    = gOffset;
			self->bOffset    = bOffset;
			Py_INCREF(self);
		} else if(pixels) {
			free(pixels);
		}
	}

	Py_INCREF(self);
        return (PyObject *)self;
}

// Initialize DotStar object
static int DotStar_init(DotStarObject *self, PyObject *arg) {
	uint32_t i;
	// Set first byte of each 4-byte pixel to 0xFF, rest to 0x00 (off)
	memset(self->pixels, 0, self->numLEDs * 4);
	for(i=0; i<self->numLEDs; i++) self->pixels[i * 4] = 0xFF;
	return 0;
}

// Detect Pi board type.  Doesn't return super-granular details,
// just the most basic distinction needed for GPIO compatibility:
// 0: Pi 1 Model B revision 1
// 1: Pi 1 Model B revision 2, Model A, Model B+, Model A+
// 2: Pi 2 Model B

static int boardType(void) {
	FILE *fp;
	char  buf[1024], *ptr;
	int   n, board = 1; // Assume Pi1 Rev2 by default

	// Relies on info in /proc/cmdline.  If this becomes unreliable
	// in the future, alt code below uses /proc/cpuinfo if any better.
#if 1
	if((fp = fopen("/proc/cmdline", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "mem_size=")) &&
			   (sscanf(&ptr[9], "%x", &n) == 1) &&
			   (n == 0x3F000000)) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "boardrev=")) &&
			          (sscanf(&ptr[9], "%x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#else
	char s[8];
	if((fp = fopen("/proc/cpuinfo", "r"))) {
		while(fgets(buf, sizeof(buf), fp)) {
			if((ptr = strstr(buf, "Hardware")) &&
			   (sscanf(&ptr[8], " : %7s", s) == 1) &&
			   (!strcmp(s, "BCM2709"))) {
				board = 2; // Appears to be a Pi 2
				break;
			} else if((ptr = strstr(buf, "Revision")) &&
			          (sscanf(&ptr[8], " : %x", &n) == 1) &&
			          ((n == 0x02) || (n == 0x03))) {
				board = 0; // Appears to be an early Pi
				break;
			}
		}
		fclose(fp);
	}
#endif

	return board;
}

// Initialize pins/SPI for output
static PyObject *begin(DotStarObject *self) {
	if(self->dataPin == 0xFF) { // Use hardware SPI
		if((self->fd = open("/dev/spidev0.0", O_RDWR)) < 0) {
			printf("Can't open /dev/spidev0.0 (try 'sudo')\n");
			return NULL;
		}
		uint8_t mode = SPI_MODE_0 | SPI_NO_CS;
		ioctl(self->fd, SPI_IOC_WR_MODE, &mode);
		// The actual data rate may be less than requested.
		// Hardware SPI speed is a function of the system core
		// frequency and the smallest power-of-two prescaler
		// that will not exceed the requested rate.
		// e.g. 8 MHz request: 250 MHz / 32 = 7.8125 MHz.
		ioctl(self->fd, SPI_IOC_WR_MAX_SPEED_HZ, self->bitrate);
	} else { // Use bitbang "soft" SPI (any 2 pins)
		if(gpio == NULL) { // First time accessing GPIO?
			int fd;

			if((fd = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
				printf("Can't open /dev/mem (try 'sudo')\n");
				return NULL;
			}
			isPi2 = (boardType() == 2);
			gpio  = (volatile unsigned *)mmap( // Memory-map I/O
			  NULL,                 // Any adddress will do
			  BLOCK_SIZE,           // Mapped block length
			  PROT_READ|PROT_WRITE, // Enable read+write
			  MAP_SHARED,           // Shared w/other processes
			  fd,                   // File to map
			  isPi2 ?
			   PI2_GPIO_BASE :      // -> GPIO registers
			   PI1_GPIO_BASE);
			close(fd);              // Not needed after mmap()
			if(gpio == MAP_FAILED) {
				err("Can't mmap()");
				return NULL;
			}
			gpioSet = &gpio[7];
			gpioClr = &gpio[10];
		}

		self->dataMask  = 1 << self->dataPin;
		self->clockMask = 1 << self->clockPin;

		// Set 2 pins as outputs.  Must use INP before OUT.
		INP_GPIO(self->dataPin);  OUT_GPIO(self->dataPin);
		INP_GPIO(self->clockPin); OUT_GPIO(self->clockPin);

		*gpioClr = self->dataMask | self->clockMask; // data+clock LOW
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Set strip data to 'off' (just clears buffer, does not write to strip)
static PyObject *clear(DotStarObject *self) {
	uint8_t *ptr;
	uint32_t i;
	for(ptr = self->pixels, i=0; i<self->numLEDs; i++, ptr += 4) {
		ptr[1] = 0x00; ptr[2] = 0x00; ptr[3] = 0x00;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

// Set global strip brightness.  This does not have an immediate effect;
// must be followed by a call to show().  Not a fan of this...for various
// reasons I think it's better handled in one's application, but it's here
// for parity with the Arduino NeoPixel library.
static PyObject *setBrightness(DotStarObject *self, PyObject *arg) {
	uint8_t b;
        if(!PyArg_ParseTuple(arg, "b", &b)) return NULL;

	// Stored brightness value is different than what's passed.  This
	// optimizes the actual scaling math later, allowing a fast multiply
	// and taking the MSB.  'brightness' is a uint8_t, adding 1 here may
	// (intentionally) roll over...so 0 = max brightness (color values
	// are interpreted literally; no scaling), 1 = min brightness (off),
	// 255 = just below max brightness.
	self->brightness = b + 1;

	Py_INCREF(Py_None);
	return Py_None;
}

// Valid syntaxes:
// x.setPixelColor(index, red, green, blue)
// x.setPixelColor(index, 0x00RRGGBB)
static PyObject *setPixelColor(DotStarObject *self, PyObject *arg) {
	uint32_t i, v;
	uint8_t  r, g, b;

	switch(PyTuple_Size(arg)) {
	   case 4: // Index, r, g, b
		if(!PyArg_ParseTuple(arg, "Ibbb", &i, &r, &g, &b))
			return NULL;
		break;
	   case 2: // Index, value
		if(!PyArg_ParseTuple(arg, "II", &i, &v))
			return NULL;
		r = v >> 16;
		g = v >>  8;
		b = v;
		break;
	   default:
		return NULL;
	}

	if(i < self->numLEDs) {
		uint8_t *ptr = &self->pixels[i * 4];
		ptr[self->rOffset] = r;
		ptr[self->gOffset] = g;
		ptr[self->bOffset] = b;
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Bitbang requires throttle on clock set/clear to avoid outpacing strip
static void clockPulse(uint32_t mask) {
	volatile uint8_t hi, lo;
	*gpioSet = mask;
	if(isPi2) {
		hi = 60; // These were found empirically
		lo = 50; // using 'Pi2' overclock setting and...
	} else {
		hi = 14; // ...'Medium' setting, respectively,
		lo = 2;  // driving a 2 meter x 144 LED strip.
	}
	while(hi--);
	*gpioClr = mask;
	while(lo--);
}

// Private method.  Writes pixel data without brightness scaling.
static void raw_write(DotStarObject *self, uint8_t *ptr, uint32_t len) {
	if(self->fd >= 0) { // Hardware SPI
		xfer[0].speed_hz = self->bitrate;
		xfer[1].speed_hz = self->bitrate;
		xfer[2].speed_hz = self->bitrate;
		xfer[1].tx_buf   = (unsigned long)ptr;
		xfer[1].len      = len;
		if(self->numLEDs) xfer[2].len = (self->numLEDs + 15) / 16;
		else              xfer[2].len = ((len / 4) + 15) / 16;
		// All that spi_ioc_transfer struct stuff earlier in
		// the code is so we can use this single ioctl to concat
		// the data & footer into one operation:
		(void)ioctl(self->fd, SPI_IOC_MESSAGE(3), xfer);
	} else if(self->dataMask) { // Bitbang
		unsigned char byte, bit,
		              headerLen = 32;
		uint32_t      footerLen;
		if(self->numLEDs) footerLen = (self->numLEDs + 1) / 2;
		else              footerLen = ((len / 4) + 1) / 2;
		*gpioClr = self->dataMask;
		while(headerLen--) clockPulse(self->clockMask);
		while(len--) { // Pixel data
			byte = *ptr++;
			for(bit = 0x80; bit; bit >>= 1) {
				if(byte & bit) *gpioSet = self->dataMask;
				else           *gpioClr = self->dataMask;
				clockPulse(self->clockMask);
			}
		}
		*gpioClr = self->dataMask;
		while(footerLen--) clockPulse(self->clockMask);
	}
}

// Issue data to strip.  Optional arg = raw bytearray to issue to strip
// (else object's pixel buffer is used).  If passing raw data, it must
// be in strip-ready format (4 bytes/pixel, 0xFF/B/G/R) and no brightness
// scaling is performed...it's all about speed (for POV, etc.)
static PyObject *show(DotStarObject *self, PyObject *arg) {
	if(PyTuple_Size(arg) == 1) { // Raw bytearray passed
		Py_buffer buf;
		if(!PyArg_ParseTuple(arg, "s*", &buf)) return NULL;
		raw_write(self, buf.buf, buf.len);
		PyBuffer_Release(&buf);
	} else { // Write object's pixel buffer
		if(self->brightness == 0) { // Send raw (no scaling)
			raw_write(self, self->pixels, self->numLEDs * 4);
		} else { // Adjust brightness during write
			uint32_t i;
			uint8_t *ptr   = self->pixels;
			uint16_t scale = self->brightness;
			if(self->fd >= 0) { // Hardware SPI
				// Allocate pBuf if using hardware
				// SPI and not previously alloc'd
				if((self->pBuf == NULL) && ((self->pBuf =
				  (uint8_t *)malloc(self->numLEDs * 4)))) {
					memset(self->pBuf, 0xFF,
					  self->numLEDs * 4); // Init MSBs
				}

				if(self->pBuf) {
					// Scale from 'pixels' buffer into
					// 'pBuf' (if available) and then
					// use a single efficient write
					// operation (thx Eric Bayer).
					uint8_t *pb = self->pBuf;
					for(i=0; i<self->numLEDs;
					  i++, ptr += 4, pb += 4) {
						pb[1] = (ptr[1] * scale) >> 8;
						pb[2] = (ptr[2] * scale) >> 8;
						pb[3] = (ptr[3] * scale) >> 8;
					}
					raw_write(self, self->pBuf,
					  self->numLEDs * 4);
				} else {
					// Fallback if pBuf not available
					// (just in case malloc fails),
					// also write() bugfix via Eric Bayer
					uint8_t x[4];
					// Header:
					x[0] = 0;
					i    = 4;
					while(i--) write(self->fd, x, 1);
					// Payload:
					x[0] = 0xFF;
					for(i=0; i<self->numLEDs;
					  i++, ptr += 4) {
						x[1] = (ptr[1] * scale) >> 8;
						x[2] = (ptr[2] * scale) >> 8;
						x[3] = (ptr[3] * scale) >> 8;
						write(self->fd, x, sizeof(x));
					}
					// Footer:
					x[0] = 0;
					i = (self->numLEDs + 15) / 16;
					while(i--) write(self->fd, x, 1);
				}
			} else if(self->dataMask) {
				uint32_t word, bit;
				// Header (32 bits)
				*gpioClr = self->dataMask;
				bit      = 32;
				while(bit--) clockPulse(self->clockMask);
				for(i=0; i<self->numLEDs; i++, ptr += 4) {
					word = 0xFF000000                   |
					 (((ptr[1] * scale) & 0xFF00) << 8) |
					 ( (ptr[2] * scale) & 0xFF00      ) |
					 ( (ptr[3] * scale)           >> 8);
					for(bit = 0x80000000; bit; bit >>= 1) {
						if(word & bit)
						  *gpioSet = self->dataMask;
						else
						  *gpioClr = self->dataMask;
						clockPulse(self->clockMask);
					}
				}
				// Footer (1/2 bit per LED)
				*gpioClr = self->dataMask;
				bit      = (self->numLEDs + 1) / 2;
				while(bit--) clockPulse(self->clockMask);
			}
		}
	}

	Py_INCREF(Py_None);
	return Py_None;
}

// Given separate R, G, B, return a packed 32-bit color.
// Meh, mostly here for parity w/Arduino library.
static PyObject *Color(DotStarObject *self, PyObject *arg) {
	uint8_t   r, g, b;
	PyObject *result;

	if(!PyArg_ParseTuple(arg, "bbb", &r, &g, &b)) return NULL;

	result = Py_BuildValue("I", (r << 16) | (g << 8) | b);
	Py_INCREF(result);
	return result;
}

// Return color of previously-set pixel (as packed 32-bit value)
static PyObject *getPixelColor(DotStarObject *self, PyObject *arg) {
	uint32_t  i;
	uint8_t   r=0, g=0, b=0;
	PyObject *result;

	if(!PyArg_ParseTuple(arg, "I", &i)) return NULL;

	if(i < self->numLEDs) {
		uint8_t *ptr = &self->pixels[i * 4];
		r = ptr[self->rOffset];
		g = ptr[self->gOffset];
		b = ptr[self->bOffset];
	}

	result = Py_BuildValue("I", (r << 16) | (g << 8) | b);
	Py_INCREF(result);
	return result;
}

// Return strip length
static PyObject *numPixels(DotStarObject *self) {
	PyObject *result = Py_BuildValue("I", self->numLEDs);
	Py_INCREF(result);
	return result;
}

// Return strip brightness
static PyObject *getBrightness(DotStarObject *self) {
	PyObject *result = Py_BuildValue("H", (uint8_t)(self->brightness - 1));
	Py_INCREF(result);
	return result;
}

// DON'T USE THIS.  One of those "parity with Arduino library" methods,
// but current'y doesn't work (and might never).  Supposed to return strip's
// pixel buffer, but doesn't seem to be an easy way to do this in Python 2.X.
// That's okay -- instead of 'raw' access to a strip's previously-allocated
// buffer, a Python program can instead allocate its own buffer and pass this
// to the show() method, basically achieving the same thing and then some.
static PyObject *getPixels(DotStarObject *self) {
	PyObject *result = Py_BuildValue("s#",
	  self->pixels, self->numLEDs * 4);
	Py_INCREF(result);
	return result;
}

static PyObject *_close(DotStarObject *self) {
	if(self->fd) {
		close(self->fd);
		self->fd = -1;
	} else {
		INP_GPIO(self->dataPin);
		INP_GPIO(self->clockPin);
		self->dataMask  = 0;
		self->clockMask = 0;
	}
	Py_INCREF(Py_None);
	return Py_None;
}

static void DotStar_dealloc(DotStarObject *self) {
	_close(self);
	if(self->pBuf)   free(self->pBuf);
	if(self->pixels) free(self->pixels);
	self->ob_type->tp_free((PyObject *)self);
}

// Method names are silly and inconsistent, but following NeoPixel
// and prior libraries, which formed through centuries of accretion.
static PyMethodDef methods[] = {
  { "begin"        , (PyCFunction)begin        , METH_NOARGS , NULL },
  { "clear"        , (PyCFunction)clear        , METH_NOARGS , NULL },
  { "setBrightness", (PyCFunction)setBrightness, METH_VARARGS, NULL },
  { "setPixelColor", (PyCFunction)setPixelColor, METH_VARARGS, NULL },
  { "show"         , (PyCFunction)show         , METH_VARARGS, NULL },
  { "Color"        , (PyCFunction)Color        , METH_VARARGS, NULL },
  { "getPixelColor", (PyCFunction)getPixelColor, METH_VARARGS, NULL },
  { "numPixels"    , (PyCFunction)numPixels    , METH_NOARGS , NULL },
  { "getBrightness", (PyCFunction)getBrightness, METH_NOARGS , NULL },
  { "getPixels"    , (PyCFunction)getPixels    , METH_NOARGS , NULL },
  { "close"        , (PyCFunction)_close       , METH_NOARGS , NULL },
  { NULL, NULL, 0, NULL }
};

static PyTypeObject DotStarObjectType = {
	PyObject_HEAD_INIT(NULL)
	0,                           // ob_size (not used, always set to 0)
	"dotstar.Adafruit_DotStar",  // tp_name (module name, object name)
	sizeof(DotStarObject),       // tp_basicsize
	0,                           // tp_itemsize
	(destructor)DotStar_dealloc, // tp_dealloc
	0,                           // tp_print
	0,                           // tp_getattr
	0,                           // tp_setattr
	0,                           // tp_compare
	0,                           // tp_repr
	0,                           // tp_as_number
	0,                           // tp_as_sequence
	0,                           // tp_as_mapping
	0,                           // tp_hash
	0,                           // tp_call
	0,                           // tp_str
	0,                           // tp_getattro
	0,                           // tp_setattro
	0,                           // tp_as_buffer
	Py_TPFLAGS_DEFAULT,          // tp_flags
	0,                           // tp_doc
	0,                           // tp_traverse
	0,                           // tp_clear
	0,                           // tp_richcompare
	0,                           // tp_weaklistoffset
	0,                           // tp_iter
	0,                           // tp_iternext
	methods,                     // tp_methods
	0,                           // tp_members
	0,                           // tp_getset
	0,                           // tp_base
	0,                           // tp_dict
	0,                           // tp_descr_get
	0,                           // tp_descr_set
	0,                           // tp_dictoffset
	(initproc)DotStar_init,      // tp_init
	0,                           // tp_alloc
	DotStar_new,                 // tp_new
	0,                           // tp_free
};

PyMODINIT_FUNC initdotstar(void) { // Module initialization function
	PyObject* m;

	if((m = Py_InitModule("dotstar", methods)) &&
	   (PyType_Ready(&DotStarObjectType) >= 0)) {
		Py_INCREF(&DotStarObjectType);
		PyModule_AddObject(m, "Adafruit_DotStar",
		  (PyObject *)&DotStarObjectType);
	}
}
