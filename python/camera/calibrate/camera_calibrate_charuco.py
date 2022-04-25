# -*- coding: utf-8 -*-
###相机标定
import cv2
import numpy as np
import os
import glob

dirpath = os.getcwd()
print("当前路径是%s" % dirpath)
# 创建一个标准板子
num = 9  # 9*9的板子
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
board = cv2.aruco.CharucoBoard_create(num, num, 0.1, 0.075, dictionary)  # 0.025单位是米
img = board.draw((200 * num, 200 * num))
cv2.imwrite(dirpath + "\\" + "{}.png".format(num), img)
# 打印板子，如贴在墙上，用相机不同角度拍摄若干张照片
# 标定
demo_path = r'/home/qcraft/camera_calibration/image_data/30s_2/*.png'  # 文件夹
save_path = r'/home/qcraft/camera_calibration/image_data/30s_2/marsk/'
# if len(os.listdir(demo_path)) > 40:
#     assert ("相机拍摄照片少于40张")
allCorners = []
allIds = []
# os.chdir(demo_path)  # 改变路径，变换到文件夹中

# images = glob.glob(demo_path)
# #
# cv2.namedWindow("marsk", 0)
# cv2.resizeWindow("marsk", 1300, 900)
#
# for i in range(len(images)):  # 这里也可以用webcamera 测试，把标准板子在webcamera 前移动
#     im = cv2.imread(images[i], 0)
#     corners, ids, rejected = cv2.aruco.detectMarkers(im, dictionary)
#     if corners == None or len(corners) == 0:
#         continue
#     ret, charucoCorners, charucoIds = cv2.aruco.interpolateCornersCharuco(corners, ids, im,
#                                                                           board)  # 其中的参数依赖于detectMarkers检测的初始值
#     print(charucoCorners.shape)
#     print(charucoIds.shape)
#     print(charucoCorners[0])
#     print(charucoIds[0])
#     if corners is not None and charucoIds is not None:
#         allCorners.append(charucoCorners)
#         allIds.append(charucoIds)
#     cv2.aruco.drawDetectedMarkers(im, corners, ids)
#     # cv2.aruco.drawDetectedCornersCharuco(im, charucoCorners, charucoIds)
#
#     img_name = save_path + images[i].split("/")[-1]
#     print(img_name)
#     cv2.imwrite(img_name, im)
#
#     cv2.imshow("marsk", im)
#     key = cv2.waitKey(1000)
#     if key == ord('q'):
#         break
#
# w, h = im.shape[1], im.shape[0]
#
# np.savez("/home/qcraft/camera_calibration/python/result/charuco_result.npz", allCorners=allCorners, allIds=allIds, w=w, h=h)

detect = np.load("/home/qcraft/camera_calibration/python/result/charuco_result.npz", allow_pickle=True)
allCorners = detect["allCorners"]
allIds = detect["allIds"]
w = int(detect["w"])
h = int(detect["h"])

K = np.zeros((3, 3))
D = np.zeros((8, 1))
# RR = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
# TT = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
print(allIds.shape)
print(allCorners.shape)
print(allIds[0][0])
print(allCorners[0][0])
ret, K, dist_coef, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(allCorners, allIds, board, (w, h), K, D, flags=cv2.CALIB_RATIONAL_MODEL)

# rms_ = cv::aruco::calibrateCameraCharuco(
#     charuco_corners_mat_, charuco_ids_mat_, charuco_board_, image_size_,
#     camera_matrix_, dist_coeffs_, rvecs, tvecs, cv::CALIB_RATIONAL_MODEL);
# save results
cali_results = np.savez(demo_path + "\\" + "camera.npz", k=K, d=dist_coef)  # cali_results['k']和cali_results['d']可以可视化结果
