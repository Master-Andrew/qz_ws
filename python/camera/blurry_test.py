# import the necessary packages
#https://zhuanlan.zhihu.com/p/248275907

from imutils import paths
import argparse
import cv2
import os
import glob
import numpy as np
import matplotlib.pyplot as plt


def mkdir(path):
    folder = os.path.exists(path)
    if not folder:  # 判断是否存在文件夹如果不存在则创建为文件夹
        os.makedirs(path)


def variance_of_laplacian(image):
    # compute the Laplacian of the image and then return the focus
    # measure, which is simply the variance of the Laplacian
    return cv2.Laplacian(image, cv2.CV_64F).var()


# construct the argument parse and parse the arguments
ap = argparse.ArgumentParser()
ap.add_argument("-i", "--images", required=True,
                help="path to input directory of images")
ap.add_argument("-t", "--threshold", type=float, default=100.0,
                help="focus measures that fall below this value will be considered 'blurry'")
args = vars(ap.parse_args())
print(args)
mkdir(args["images"][:-5] + "/blurry/")
# print(args["images"][:-5]+"/blurry/")

fm_list = []
image_paths = glob.glob(args["images"])
for imagePath in image_paths:
    # load the image, convert it to grayscale, and compute the
    # focus measure of the image using the Variance of Laplacian
    # method
    print(imagePath)
    image = cv2.imread(imagePath)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    fm = variance_of_laplacian(gray)
    fm_list.append(fm)

fm_mean = np.mean(fm_list)
fm_std = np.std(fm_list)
factor = 2
print(len(fm_list))
for i in range(len(fm_list)):
    fm = fm_list[i]
    image = cv2.imread(image_paths[i])
    text = "Not Blurry"
    plt.scatter(0, fm, alpha=0.3, c="b")
    # if the focus measure is less than the supplied threshold,
    # then the image should be considered "blurry"
    if fm > fm_mean + factor * fm_std:
        text = "Blurry"
    # show the image
    cv2.putText(image, "{}: {:.2f}".format(text, fm), (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 3)
    # cv2.imshow("Image", image)
    name = args["images"][:-5] + "blurry/" + image_paths[i].split("/")[-1]
    print(name)
    cv2.imwrite(name, image)
    # key = cv2.waitKey(0)

plt.scatter(0, fm_mean, alpha=1, c="r")
plt.show()

print(fm_mean, fm_std, fm_mean + factor * fm_std)
