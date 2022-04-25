import cv2
import numpy as np
import glob
import os
import sys
import yaml


def load_calibrate_result(calibrate_result_file):
    f = open(calibrate_result_file, "r", encoding="utf-8")
    cfg = f.read()
    # print(cfg)
    calibrate_result = yaml.load(cfg, Loader=yaml.FullLoader)
    # print(calibrate_result)

    mtx = calibrate_result["camera_matrix"]["data"]
    dist = calibrate_result["distortion_coefficients"]["data"]
    w = calibrate_result["image_width"]
    h = calibrate_result["image_height"]

    K = np.zeros((3, 3))
    D = np.zeros((1, 14))

    K[0][0] = mtx[0]
    K[0][2] = mtx[2]
    K[1][1] = mtx[4]
    K[1][2] = mtx[5]
    K[2][2] = 1.0

    for i in range(8):
        D[0][i] = dist[i]
    print(K, "\n", D)

    return K, D, w, h


def undistort_img(img_name, mtx, dist, alpha):
    img = cv2.imread(img_name)
    h, w = img.shape[:2]

    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha, (w, h))
    # print(newcameramtx, roi)

    undistorted_img = cv2.undistort(img, mtx, dist, None, newcameramtx)

    # cv2.namedWindow("undistorted", 0)
    # cv2.resizeWindow("undistorted", 1500, 1000)
    # cv2.imshow("undistorted", undistorted_img)
    # cv2.waitKey(1000)

    return undistorted_img, roi


def find_chessboard(img, row, col):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE  # 11

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, (row, col), flags)

    if ok:  # 如果找到，添加目标点，图像点
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 获取更精确的角点位置

        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (row, col), corners, ok)
        cv2.imshow('img', img)
        cv2.waitKey(0)


if __name__ == '__main__':
    # img_name = sys.argv[1]
    img_name = "/home/qcraft/camera_calibration/image_data/h130s_e03220700/left-0001.png"
    # calibrate_result_file = sys.argv[2]
    calibrate_result_file = "/home/qcraft/camera_calibration/image_data/h130s_e03220700/ost.yaml"
    alpha = 1
    row = 8
    col = 8

    mtx, dist, w, h = load_calibrate_result(calibrate_result_file)
    undistorted_img, roi = undistort_img(img_name, mtx, dist, alpha)
    img_chessboard = find_chessboard(undistorted_img, row, col)
