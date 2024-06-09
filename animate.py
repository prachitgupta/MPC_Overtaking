import os
from PIL import Image
def generate_gif(input_dir, output_file, duration):
    # Get the current directory
    current_dir = os.getcwd()

    # Construct the full path to the input directory
    input_dir_path = os.path.join(current_dir, input_dir)

    # Get all image files in the input directory
    image_files = [f for f in os.listdir(input_dir_path) if f.endswith('.png')]

    # Sort the image files by name (assuming they are named in the correct order)
    image_files.sort()

    # Open each image and append it to a list of frames
    frames = []
    for file in image_files:
        image_path = os.path.join(input_dir_path, file)
        img = Image.open(image_path)
        frames.append(img)

    # Save the frames as an animated GIF
    frames[0].save(output_file, format='GIF', append_images=frames[1:], save_all=True, duration=duration, loop=0)

# Example usage
input_directory = 'Brts'
output_filename = 'brts.gif'
frame_duration = 100  # Duration of each frame in milliseconds

generate_gif(input_directory, output_filename, frame_duration)
