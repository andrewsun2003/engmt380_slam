import slamBotHD as sb       
import cv2 as cv 
import numpy as np

def read_img():
    depth_img = sb.imgDepth

    while depth_img is None:
        depth_img = sb.imgDepth

    imgRow = depth_img[200:240, :]

    if len(imgRow.shape) != 2:
        raise ValueError("img_rows does not have two dimensions")

    rows, cols = imgRow.shape
    averaged_slice = np.zeros(cols, dtype=float)

    for y in range(cols):
        total = 0
        count = 0

        for x in range(min(8, rows)):
            if imgRow[x, y] > 0:
                total += imgRow[x, y]
                count += 1

        averaged_slice[y] = total / count if count != 0 else 0
    
    imgRow = np.nan_to_num(averaged_slice)  # Handle NaN values 

    return imgRow
