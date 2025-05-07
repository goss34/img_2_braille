import numpy as np
from PIL import Image
from skimage.filters import threshold_otsu

def image_to_braille(image_path, output_path, width=40, height=9):
    # Open the image and convert to grayscale
    print(f"Processing {image_path}")
    img = Image.open(image_path).convert('L')
    
    # Convert to numpy array
    img_array = np.array(img)
    print(f"Image shape: {img_array.shape}")
    print(f"Image value range: {img_array.min()} to {img_array.max()}")
    
    # Simple thresholding for high contrast images - looking for white shapes
    binary = img_array > 128  # True for white pixels
    print(f"Number of white pixels: {np.sum(binary)}")
    
    # Calculate the scaling factor to fit within the Braille display dimensions
    # while preserving aspect ratio
    img_height = height * 3  # Braille is 3 dots high
    img_width = width * 2   # Braille is 2 dots wide
    
    scale = min(img_width / img_array.shape[1], img_height / img_array.shape[0])
    new_width = int(img_array.shape[1] * scale)
    new_height = int(img_array.shape[0] * scale)
    
    print(f"Scaling to: {new_width}x{new_height}")
    
    # Resize the image maintaining original position
    resized = np.zeros((img_height, img_width), dtype=bool)
    temp_img = Image.fromarray(binary.astype(np.uint8) * 255).resize(
        (new_width, new_height), 
        Image.LANCZOS
    )
    temp_array = np.array(temp_img) > 127
    
    # Place in top-left corner rather than centering
    resized[:new_height, :new_width] = temp_array
    
    # Format for BRF
    brf_text = ''
    for y in range(0, height * 3, 3):
        line = ''
        for x in range(0, width * 2, 2):
            # Get the 2x3 subarray for this Braille cell
            cell = resized[y:y+3, x:x+2]
            
            # Convert the cell to a Braille pattern
            if np.any(cell):  # If any white pixels
                dot_pattern = 0
                for i in range(3):
                    for j in range(2):
                        if cell[i, j]:
                            dot_pattern |= 1 << (3*j + i)
                braille_char = dot_pattern_to_ascii_braille(dot_pattern)
                line += braille_char
            else:
                line += ' '  # Space for black areas
        
        # Always add the line to preserve vertical positioning
        brf_text += line.rstrip() + '\n'
    
    # Write to file
    if not brf_text.strip():
        print("Warning: No content generated for BRF file!")
    else:
        print(f"Writing {len(brf_text.split('\n'))} lines to {output_path}")
        with open(output_path, 'w', encoding='ascii', errors='replace') as f:
            f.write(brf_text)

def dot_pattern_to_ascii_braille(pattern):
    return chr((pattern ^ 0x40) + 0x20)

# Example usage
if __name__ == "__main__":
    print("Starting conversion...")
    image_to_braille('star.jpg', 'star.brf')
    print("\n" + "="*50 + "\n")
    print("\nConversion complete.")