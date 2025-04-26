import OpenEXR
import Imath
import numpy as np
from openexr_numpy import imread, imwrite
from skimage.metrics import structural_similarity as get_ssim
from skimage.metrics import peak_signal_noise_ratio as get_psnr
from skimage.metrics import mean_squared_error as get_mse
import argparse
import math

def read_exr(exr_file):
    rgb = imread(exr_file)
    # Tonemap (very roughly)
    return 255 * np.sqrt(np.clip(rgb, 0, 1))

def compute_ssim(img1, img2):
    return get_ssim(img1, img2, channel_axis=-1, full=True)

def compare(img1, img2):
    mse = get_mse(img1, img2)
    psnr = get_psnr(img1, img2)
    ssim = compute_ssim(img1, img2)
    return mse,psnr,ssim

def main():
    parser = argparse.ArgumentParser(description="Compute MSE and PSNR between two EXR images")
    parser.add_argument('image1', type=str, help="Path to the first EXR image")
    parser.add_argument('image2', type=str, help="Path to the second EXR image")
    args = parser.parse_args()
    print(args.image1, args.image2)

    img1 = read_exr(args.image1)
    img2 = read_exr(args.image2)
    print(np.max(img2))
    mse,psnr,ssim = compare(img1, img2)
    print(f"MSE: {mse} | PSNR: {psnr}dB | SSIM: {ssim}")

if __name__ == "__main__":
    main()
