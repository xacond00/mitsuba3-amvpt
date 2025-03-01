import os
import numpy as np
import openexr_numpy
import imageio
from PIL import Image
import sys

def read_exr(file_path):
    img = openexr_numpy.imread(file_path)
    return np.sqrt(np.clip(img,0,1))

def extract_views(exr_image, grid_width, grid_height):
    exr_image = read_exr(exr_image)
    img_height, img_width, _ = exr_image.shape
    camera_images = []
    cam_height = img_height // grid_height
    cam_width = img_width // grid_width
    
    for row in range(grid_height):
        for col in range(grid_width):
            # suppose flipped y axis
            rowf = grid_height - row - 1
            y_start = rowf * cam_height
            y_end = (rowf + 1) * cam_height
            x_start = col * cam_width
            x_end = (col + 1) * cam_width
            # slice the input image
            camera_img = exr_image[y_start:y_end, x_start:x_end, :]
            camera_images.append(camera_img)
    
    return camera_images

def save_as_gif(images, output_filename, duration=0.1):
    pil_images = [Image.fromarray((img * 255).astype(np.uint8)) for img in images]
    pil_images[0].save(output_filename, save_all=True, append_images=pil_images[1:], optimize=False, duration=duration, loop=0)
    print(f"GIF saved as {output_filename}")

def generate_gif(filename, grid_width, grid_height, time, output_filename="output.gif"):
    all_camera_images = extract_views(filename, grid_width, grid_height)
    num_frames = len(all_camera_images)
    animated_images = [all_camera_images[i] for i in range(num_frames)]
    save_as_gif(animated_images, output_filename, time)

if __name__ == "__main__":
    if len(sys.argv) < 5:
        print("Usage: python generate_gif.py <image_directory> <grid_width> <grid_height> <time> <output_filename (opt)>")
        sys.exit(1)

    filename = sys.argv[1]
    grid_width = int(sys.argv[2])
    grid_height = int(sys.argv[3])
    time = float(sys.argv[4])
    output_filename = sys.argv[5] if len(sys.argv) >= 6 else filename.rstrip('.', 1) + '.gif'

    generate_gif(filename, grid_width, grid_height, time, output_filename)
