import os
import glob
import cv2
import numpy as np


folder_path = "/app/ESVO/result/depth/"
inv_path = "/app/ESVO/result/inv/"

tiff_files = sorted(glob.glob(os.path.join(folder_path, "*.png")))
tiff_files = sorted([os.path.basename(f) for f in glob.glob(os.path.join(folder_path, "*.png"))])
inv_files = sorted([os.path.basename(f) for f in glob.glob(os.path.join(inv_path, "*.png"))])



for file in tiff_files:

    img = cv2.imread(folder_path + file, cv2.IMREAD_UNCHANGED) 
    img_inv = cv2.imread(inv_path + file, cv2.IMREAD_UNCHANGED) 

    print(f"Reading: {file}, Shape: {img.shape}, Dtype: {img.dtype}")

    
    
    # if img.dtype == np.float32 or img.dtype == np.float64:
    #     img_disp = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    # else:
    #     img_disp = img
    img_disp = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    img_col = cv2.applyColorMap(img_disp, cv2.COLORMAP_JET)
    mask = (img_disp == 0)
    img_col[mask] = [0, 0, 0]
    
    combined = np.hstack((img_inv, img_col))
    
    cv2.imshow("TIFF Image", combined)
    cv2.waitKey(1000)

cv2.destroyAllWindows()
