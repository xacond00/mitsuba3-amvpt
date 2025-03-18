import OpenEXR
import Imath
import numpy as np
from openexr_numpy import imread, imwrite
import argparse
import math

def read_exr(exr_file):
    rgb = imread(exr_file)
    # Tonemap (very roughly)
    return np.sqrt(np.clip(rgb, 0, 1))

def compute_mse(image1, image2):
    assert image1.shape == image2.shape, "Input images must have the same dimensions"
    return np.mean((image1 - image2) ** 2)

def compute_psnr(mse, max_pixel_value=1.0):
    if mse == 0:
        return float('inf')
    return 20 * math.log10(max_pixel_value / math.sqrt(mse))

def main():
    parser = argparse.ArgumentParser(description="Compute MSE and PSNR between two EXR images")
    parser.add_argument('image1', type=str, help="Path to the first EXR image")
    parser.add_argument('image2', type=str, help="Path to the second EXR image")
    args = parser.parse_args()
    print(args.image1, args.image2)

    img1 = read_exr(args.image1)
    img2 = read_exr(args.image2)
    print(np.max(img2))

    mse = compute_mse(img1, img2)
    print(f"MSE: {mse}")

    psnr = compute_psnr(mse)
    print(f"PSNR: {psnr} dB")

if __name__ == "__main__":
    main()
