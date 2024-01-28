image_size = (720*4, 480*4)
image = [[[0, 0, 0] for x in range(image_size[0])] for y in range(image_size[1])]

for y in range(image_size[1]):
    for x in range(image_size[0]):
        image[y][x] = [x / image_size[0], y / image_size[1], 0]

print(image[10][10])