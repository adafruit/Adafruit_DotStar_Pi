#!/usr/bin/python

# Light-painting example for Adafruit Dot Star RGB LED strip.
# Loads image, displays column-at-a-time on LEDs at a reasonable speed
# for long exposure photography.
# See strandtest.py for a much simpler example script.
# See image-pov.py for a faster persistence-of-vision example.

import Image
from dotstar import Adafruit_DotStar

numpixels = 30          # Number of LEDs in strip
filename  = "hello.png" # Image file to load

# Here's how to control the strip from any two GPIO pins:
datapin   = 23
clockpin  = 24
strip     = Adafruit_DotStar(numpixels, datapin, clockpin)

strip.begin()           # Initialize pins for output

# Load image in RGB format and get dimensions:
print "Loading..."
img       = Image.open(filename).convert("RGB")
pixels    = img.load()
width     = img.size[0]
height    = img.size[1]
print "%dx%d pixels" % img.size

if(height > strip.numPixels()): height = strip.numPixels()

# Calculate gamma correction table, makes mid-range colors look 'right':
gamma = bytearray(256)
for i in range(256):
	gamma[i] = int(pow(float(i) / 255.0, 2.7) * 255.0 + 0.5)

print "Displaying..."
while True:                              # Loop forever

	for x in range(width):           # For each column of image...
		for y in range(height):  # For each pixel in column...
			value = pixels[x, y]   # Read pixel in image
			strip.setPixelColor(y, # Set pixel in strip
			  gamma[value[0]],     # Gamma-corrected red
			  gamma[value[1]],     # Gamma-corrected green
			  gamma[value[2]])     # Gamma-corrected blue
		strip.show()             # Refresh LED strip
