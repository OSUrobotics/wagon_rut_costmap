from PIL import Image

effaced = Image.open('three_rooms_full_effaced.pgm')
clean = Image.open('clean_three_rooms.pgm')
out = Image.new('L', effaced.size)

width, height = effaced.size
for x in range(200):
    for y in range(200):
        effaced_pixel = effaced.getpixel((x,y))
        clean_pixel = clean.getpixel((x, y))
        print effaced_pixel
        new = clean_pixel - effaced_pixel
        out.putpixel((x, y), new)

out.save('three_rooms_output.pgm')
