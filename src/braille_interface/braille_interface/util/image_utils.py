
import cv2
import numpy as np

def preprocess_and_pad(input_img, target_shape, border=True):
    h, w = input_img.shape
    target_h, target_w = target_shape
    input_aspect = w / h
    target_aspect = target_w / target_h

    if input_aspect > target_aspect:
        new_w = target_w
        new_h = int(target_w / input_aspect)
    else:
        new_h = target_h
        new_w = int(target_h * input_aspect)

    resized = cv2.resize(input_img, (new_w, new_h), interpolation=cv2.INTER_NEAREST)
    
    if border:
        cv2.rectangle(resized, (0,0), (new_w-1, new_h-1), 255, 1)

    padded = np.zeros(target_shape, dtype=np.uint8)
    y_offset = (target_h - new_h) // 2
    x_offset = (target_w - new_w) // 2
    padded[y_offset:y_offset+new_h, x_offset:x_offset+new_w] = resized
    return padded
