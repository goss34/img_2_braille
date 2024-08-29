import numpy as np
from PIL import Image
import louis

def image_to_braille(image_path, output_path, width=40, height=9):
    # Open the image and convert to grayscale
    img = Image.open(image_path).convert('L')
    
    # Resize the image to fit the Braille display dimensions
    img = img.resize((width * 2, height * 3))
    
    # Convert the image to a numpy array
    img_array = np.array(img)
    
    # Create a 2D list to store Braille characters
    braille_matrix = [[0 for _ in range(width)] for _ in range(height)]
    
    # Convert image to Braille dots
    for y in range(height):
        for x in range(width):
            dot_pattern = 0
            for i in range(3):
                for j in range(2):
                    if img_array[y*3 + i, x*2 + j] < 128:  # Threshold for black/white
                        dot_pattern |= 1 << (i * 2 + j)
            
            braille_matrix[y][x] = dot_pattern

    # Convert Braille dot patterns to Unicode Braille characters
    braille_text = '\n'.join(''.join(chr(0x2800 + cell) for cell in row) for row in braille_matrix)
    
    # Convert to BRF format
    brf_text = louis.translate(['braille-patterns.cti'], braille_text, mode=louis.dotsIO)[0]
    
    # Write to file
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(brf_text)

image_to_braille('test.jpg', 'test_output.brf')