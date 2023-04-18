from PIL import Image
import numpy as np
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt


# def bmp_to_array(filename):
#     # Open the BMP image file
#     img = Image.open(filename)
    
#     # Convert the image to black and white (monochrome)
#     img = img.convert('1')
    
#     # Get the image dimensions
#     width, height = img.size
    
#     # Convert the image to a 2D array of 0s and 1s
#     pixels = img.load()
#     data = []
#     for y in range(height):
#         row = []
#         for x in range(width):
#             row.append(pixels[x,y])
#         data.append(row)
    
#     return data

# Read the bitmap image file and convert it to black and white
img = Image.open('atrium.bmp').convert('1')

# Convert the image to a 2D array of 0s and 1s
data = np.array(img)

print("hi\n")
# Read the BMP image and convert it to a 2D array of 0s and 1s
# data = np.array(bmp_to_array('atrium.bmp'))
print("loaded")
# Create a new figure with a size of 8x6 inches
fig = plt.figure(figsize=(8,6))
print("loaded 2")
# Plot the 2D array as a grid of 0s and 1s using Matplotlib
# plt.plot(data, cmap='Greys', interpolation='nearest')
plt.imshow(data, cmap='Greys', interpolation='nearest')
print("plot")
# Save the plot as a PDF file
plt.savefig('plot.pdf')
print("save")