#!/usr/bin/python

# Persistence-of-vision (POV) example for Adafruit Dot Star RGB LED strip.
# Loads image, displays column-at-a-time on LEDs at very high speed,
# suitable for naked-eye illusions.
# See strandtest.py for a much simpler example script.
# See image-paint.py for a slightly simpler light painting example.

import Image
from dotstar import Adafruit_DotStar

filename  = "hello.png" # Image file to load

# Here's how to control the strip from any two GPIO pins:
datapin   = 23
clockpin  = 24
strip     = Adafruit_DotStar(0, datapin, clockpin)
# Notice the number of LEDs is set to 0.  This is on purpose...we're asking
# the DotStar module to NOT allocate any memory for this strip...we'll handle
# our own allocation and conversion and will feed it 'raw' data.

# So we'll write directly to the pixel data buffer.  Data is not necessarily
# in RGB order -- current DotStars use BRG order, and older ones are GBR.
# Additionally, byte 0 of each 4-byte pixel is always 0xFF, so the RGB
# offsets are always in range 1-3.
# Here's the offsets for current (2015+) strips:
rOffset = 2
gOffset = 3
bOffset = 1
# For older strips, change values to 3, 1, 2
# This is ONLY necessary because we're raw-writing; for normal setPixelColor
# use, offsets can be changed with the 'order' keyword (see strandtest.py).

strip.begin()           # Initialize pins for output

# Load image in RGB format and get dimensions:
print "Loading..."
img       = Image.open(filename).convert("RGB")
pixels    = img.load()
width     = img.size[0]
height    = img.size[1]
print "%dx%d pixels" % img.size

# Calculate gamma correction table, makes mid-range colors look 'right':
gamma = bytearray(256)
for i in range(256):
	gamma[i] = int(pow(float(i) / 255.0, 2.7) * 255.0 + 0.5)

# Allocate list of bytearrays, one for each column of image.
# Each pixel REQUIRES 4 bytes (0xFF, B, G, R).
print "Allocating..."
column = [0 for x in range(width)]
for x in range(width):
	column[x] = bytearray(height * 4)

# Convert entire RGB image into column-wise BGR bytearray list.
# The image-paint.py example proceeds in R/G/B order because it's counting
# on the library to do any necessary conversion.  Because we're preparing
# data directly for the strip, it's necessary to work in its native order.
print "Converting..."
for x in range(width):          # For each column of image...
	for y in range(height): # For each pixel in column...
		value             = pixels[x, y]    # Read pixel in image
		y4                = y * 4           # Position in raw buffer
		column[x][y4]     = 0xFF            # Pixel start marker
		column[x][y4 + rOffset] = gamma[value[0]] # Gamma-corrected R
		column[x][y4 + gOffset] = gamma[value[1]] # Gamma-corrected G
		column[x][y4 + bOffset] = gamma[value[2]] # Gamma-corrected B

print "Displaying..."
while True:                            # Loop forever

	for x in range(width):         # For each column of image...
		strip.show(column[x])  # Write raw data to strip
