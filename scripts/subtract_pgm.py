#!/usr/bin/python

from PIL import Image
import sys

effaced = Image.open(sys.argv[1])
clean = Image.open(sys.argv[2])
out = Image.new('L', effaced.size)

width, height = effaced.size
for x in range(200):
    for y in range(200):
        effaced_pixel = effaced.getpixel((x,y))
        clean_pixel = clean.getpixel((x, y))
        print effaced_pixel
        new = clean_pixel - effaced_pixel
        out.putpixel((x, y), new)

out.save(sys.argv[3])
