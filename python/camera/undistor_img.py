import cv2
import numpy as np
import glob
import time
import os
import sys
import yaml


def mkdir(path):
    folder = os.path.exists(path)
    if not folder:  # 判断是否存在文件夹如果不存在则创建为文件夹
        os.makedirs(path)


def write_calibration_result(path, result):
    with open(path + "intrinsic.yaml", "w") as intrinsic_file:
        intrinsic = {}
        intrinsic["image_width"] = result[""]
        yaml.dump(intrinsic, intrinsic_file)


def save_origin_undistor_image(origin_img, undistor_img, image_path, save_path):
    mkdir(save_path)
    # 照片 / 添加的文字 / 左上角坐标 / 字体 / 字体大小 / 颜色 / 字体粗细
    cv2.putText(origin_img, "origin", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)
    cv2.putText(undistor_img, "undistorted", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)

    result = cv2.vconcat([origin_img, undistor_img])

    image_name = image_path.split('/')
    image_name.insert(-1, "result")
    new_name = save_path + image_name[-1]
    print("write to ", new_name)
    cv2.imwrite(new_name, result)

    cv2.namedWindow("undistorted", 0)
    cv2.resizeWindow("undistorted", 1500, 1000)
    cv2.imshow("undistorted", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def save_images(image_list, save_name, save_path, name_list=None):
    mkdir(save_path)
    # 照片 / 添加的文字 / 左上角坐标 / 字体 / 字体大小 / 颜色 / 字体粗细
    if name_list == None:
        for i in range(len(image_list)):
            cv2.putText(image_list[i], "img_{}".format(i), (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)
    else:
        for i in range(len(image_list)):
            cv2.putText(image_list[i], name_list[i], (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)

    result = image_list[0]
    for i in range(len(image_list) - 1):
        result = cv2.vconcat([result, image_list[i + 1]])

    new_name = save_path + save_name
    cv2.imwrite(new_name, result)

    cv2.namedWindow("result", 0)
    cv2.resizeWindow("result", 1500, 1000)
    cv2.imshow("result", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def load_calibrate_result(calibrate_result_file):
    f = open(calibrate_result_file, "r", encoding="utf-8")
    cfg = f.read()
    # print(cfg)
    calibrate_result = yaml.load(cfg)
    # print(calibrate_result)

    mtx = calibrate_result["camera_matrix"]["data"]
    dist = calibrate_result["distortion_coefficients"]["data"]

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

    return K, D


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


if __name__ == '__main__':
    calibrate_result_file = sys.argv[1]
    img_name = sys.argv[2]
    save_path = sys.argv[3]

    calibrate_result_file_1 = "/home/qcraft/123/calibrationdata/ost.yaml"
    calibrate_result_file_2 = "/home/qcraft/234/calibrationdata/ost.yaml"
    img_name = "/home/qcraft/123/calibrationdata/left-0000.png"
    save_path = "/home/qcraft/123/calibrationdata/undistort/"

    alpha = 0
    img = cv2.imread(img_name)

    mtx, dist = load_calibrate_result(calibrate_result_file_1)
    undistorted_img_1, roi = undistort_img(img_name, mtx, dist, alpha)

    mtx, dist = load_calibrate_result(calibrate_result_file_2)
    undistorted_img_2, roi = undistort_img(img_name, mtx, dist, alpha)

    # save_origin_undistor_image([undistorted_img_1, undistorted_img_2], img_name, save_path)

    x, y, w, h = roi

    save_images([img[y:y + h, x:x + w], undistorted_img_1[y:y + h, x:x + w], undistorted_img_2[y:y + h, x:x + w]],
                "calibration_result_00.jpg", save_path, ["origin", "123", "234"])

    # save_images([img, undistorted_img_1, undistorted_img_2],
    #             "calibration_result_2.jpg", save_path, ["origin", "123", "234"])
