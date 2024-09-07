import numpy as np
from PIL import Image
from skimage.filters import threshold_local

def image_to_braille(image_path, output_path, width=40, height=9):
    # Open the image and convert to grayscale
    img = Image.open(image_path).convert('L')
    
    # Resize the image to fit the Braille display dimensions (each cell is 2x3)
    img = img.resize((width * 2, height * 3), Image.LANCZOS)
    
    # Convert the image to a numpy array
    img_array = np.array(img)

    # # Apply adaptive thresholding
    # threshold = threshold_local(img_array, block_size=11, offset=10)
    # img_array = img_array > threshold

    # Inverse the image (True becomes False and vice versa)
    # img_array = ~img_array

    # Format for BRF
    brf_text = ''
    for y in range(0, height * 3, 3):
        for x in range(0, width * 2, 2):
            # Get the 2x3 subarray for this Braille cell
            cell = img_array[y:y+3, x:x+2]
            
            # Convert the cell to a Braille pattern
            dot_pattern = 0
            for i in range(3):
                for j in range(2):
                    if cell[i, j]:
                        dot_pattern |= 1 << (3*j + i)
            
            # Convert dot pattern to Braille character
            braille_char = dot_pattern_to_ascii_braille(dot_pattern)            
            brf_text += braille_char
        
        brf_text += '\n'
    
    # Write to file
    with open(output_path, 'w', encoding='ascii', errors='replace') as f:
        f.write(brf_text)
    
    print(f"BRF content (first 500 characters): {brf_text[:500]}")
    print(f"File written: {output_path}")

def dot_pattern_to_ascii_braille(pattern):
    return chr((pattern ^ 0x40) + 0x20)

# Example usage
image_to_braille('test.jpg', 'test5_output.brf')