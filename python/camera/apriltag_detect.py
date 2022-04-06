# -*- coding:utf-8 -*-

import pupil_apriltags as apriltag
import cv2
import numpy as np
import sys
import math


def circle_detect(fname):
    Nx_cor = 8
    Ny_cor = 8
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # The image must be a grayscale image of type numpy.uint8
    flag = cv2.CALIB_CB_ASYMMETRIC_GRID
    flag = cv2.CALIB_CB_SYMMETRIC_GRID
    ok, corners = cv2.findCirclesGrid(gray, (Nx_cor, Ny_cor), flags=flag)
    print(corners)


def aruco_detect(fname, dict=cv2.aruco.DICT_4X4_100):
    cv2.namedWindow("img", 0)
    cv2.resizeWindow("img", 1300, 900)

    img = cv2.imread(fname,0)

    # gray = cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 2)
    # cv2.imshow("img", gray)
    # cv2.waitKey(1000)

    # ret1, th1 = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
    # cv2.imshow("img", th1)
    # cv2.waitKey(10000)

    # Otsu 滤波
    ret2, img = cv2.threshold(img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # cv2.imshow("img", gray)
    # cv2.waitKey(10000)

    arucoDict = cv2.aruco.Dictionary_get(dict)

    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_16h5)
    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_50)
    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    # arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_APRILTAG_36h10)

    # https: // docs.opencv.org / 3.4 / dc / df7 / dictionary_8hpp.html
    arucoParams = cv2.aruco.DetectorParameters_create()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, arucoDict,
                                                       parameters=arucoParams)
    for corner in corners:
        for i in range(4):
            cv2.circle(img, tuple(corner[0][i].astype(int)), 4, (0, 0, 255), 2)
            # right-bottom, left-top, right-top,left-bottom
    print("find {} corners".format(len(corners)))

    cv2.imshow("img", img)
    cv2.waitKey(1000)


def chessbaord_detect(fname):
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.namedWindow("img", 0)
    cv2.resizeWindow("img", 1300, 900)

    # Nx_cor = 17
    # Ny_cor = 9

    Nx_cor = 8
    Ny_cor = 8

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE  # 11
    ok, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), flags)
    print(corners)

    objpoints = []  # 在世界坐标系中的三维点
    imgpoints = []  # 在图像平面的二维点
    image_name = []

    CheckerboardSize = 0.05

    count = 0
    objp = np.zeros((1, Nx_cor * Ny_cor, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2) * CheckerboardSize

    if ok:  # 如果找到，添加目标点，图像点
        count += 1
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 获取更精确的角点位置
        imgpoints.append(corners)
        image_name.append(fname)

        # 将角点在图像上显示
        cv2.drawChessboardCorners(img, (Nx_cor, Ny_cor), corners, ok)
        cv2.imshow('img', img)
        cv2.waitKey(5000)
    else:
        print("can not find corner points!!")


def apriltag_detect(fname, families="tag16h5"):
    # tag16h5, tag25h9, tag36h11, tagCircle21h7, tagCircle49h12, tagCustom48h12, tagStandard41h12, tagStandard52h13,

    cameraParams_Intrinsic = [987, 992.8, 1007.4, 1007]  # camera_fx, camera_fy, camera_cx, camera_cy
    camera_matrix = np.array(([987, 0, 1007.4],
                              [0, 992.8, 1007],
                              [0, 0, 1.0]), dtype=np.double)

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # The image must be a grayscale image of type numpy.uint8

    tag_detector = apriltag.Detector(families=families)  # Build a detector for apriltag
    # tag_detector = apriltag.Detector()  # Build a detector for apriltag,families="Tag36h11"

    # https: // pyimagesearch.com / 2020 / 11 / 02 / apriltag - with-python /
    tags = tag_detector.detect(gray,
                               estimate_tag_pose=True,
                               camera_params=cameraParams_Intrinsic,
                               tag_size=30)  # Perform apriltag detection to get a list of detected apriltag

    print("%d apriltags have been detected." % len(tags))

    for tag in tags:
        cv2.circle(img, tuple(tag.corners[0].astype(int)), 4, (0, 0, 255), 2)  # right-bottom
        cv2.circle(img, tuple(tag.corners[1].astype(int)), 4, (0, 0, 255), 2)  # left-top
        cv2.circle(img, tuple(tag.corners[2].astype(int)), 4, (0, 0, 255), 2)  # right-top
        cv2.circle(img, tuple(tag.corners[3].astype(int)), 4, (0, 0, 255), 2)  # left-bottom

    cv2.namedWindow("img", 0)
    cv2.resizeWindow("img", 1300, 900)
    cv2.imshow("img", img)
    cv2.waitKey(1000)


def RotateByZ(Cx, Cy, thetaZ):
    rz = thetaZ * math.pi / 180.0
    outX = math.cos(rz) * Cx - math.sin(rz) * Cy
    outY = math.sin(rz) * Cx + math.cos(rz) * Cy
    return outX, outY


def RotateByY(Cx, Cz, thetaY):
    ry = thetaY * math.pi / 180.0
    outZ = math.cos(ry) * Cz - math.sin(ry) * Cx
    outX = math.sin(ry) * Cz + math.cos(ry) * Cx
    return outX, outZ


def RotateByX(Cy, Cz, thetaX):
    rx = thetaX * math.pi / 180.0
    outY = math.cos(rx) * Cy - math.sin(rx) * Cz
    outZ = math.sin(rx) * Cy + math.cos(rx) * Cz
    return outY, outZ


def detect_tags(img, tags):
    for tag in tags:
        cv2.circle(img, tuple(tag.corners[0].astype(int)), 4, (0, 0, 255), 2)  # right-bottom
        cv2.circle(img, tuple(tag.corners[1].astype(int)), 4, (0, 0, 255), 2)  # left-top
        cv2.circle(img, tuple(tag.corners[2].astype(int)), 4, (0, 0, 255), 2)  # right-top
        cv2.circle(img, tuple(tag.corners[3].astype(int)), 4, (0, 0, 255), 2)  # left-bottom
        cv2.imshow("debug", img)
        cv2.waitKey(1000)
        # tags info
        print("family:", tag.tag_family)
        print("id:", tag.tag_id)
        print("conners:", tag.corners)
        print("homography:", tag.homography)
        print("pose_R:%s\npose_T:%s\npose_err:%s" % (tag.pose_R, tag.pose_t, tag.pose_err))


def solve(tags, camera_matrix):
    '''
    Func: When only one apriltag is detected, the PnP method is used to solve the problem.
    Args:
    Return:
    '''
    object_3d_points = np.array(([0, 0, 0],
                                 [0, 200, 0],
                                 [150, 0, 0],
                                 [150, 200, 0]),
                                dtype=np.double)  # Apriltag coordinates in the World coordinate system

    object_2d_point = np.array((tags[0].corners[0].astype(int),
                                tags[0].corners[1].astype(int),
                                tags[0].corners[2].astype(int),
                                tags[0].corners[3].astype(int)),
                               dtype=np.double)  # Apriltag coordinates in the Image pixel system

    dist_coefs = np.array([0, 0, 0, 0, 0], dtype=np.double)  # Distortion coefficient: k1, k2, p1, p2, k3

    # The function solvepnp receives a set of corresponding 3D and 2D coordinates
    # and calculates the geometric transformation corresponding to the two sets
    # of coordinates (rotation matrix rvec, translation matrix tvec).
    found, rvec, tvec = cv2.solvePnP(object_3d_points, object_2d_point, camera_matrix, dist_coefs)
    rotM = cv2.Rodrigues(rvec)[0]
    camera_postion = -np.matrix(rotM).T * np.matrix(tvec)
    # print(camera_postion.T)
    thetaZ = math.atan2(rotM[1, 0], rotM[0, 0]) * 180.0 / math.pi
    thetaY = math.atan2(-1.0 * rotM[2, 0], math.sqrt(rotM[2, 1] ** 2 + rotM[2, 2] ** 2)) * 180.0 / math.pi
    thetaX = math.atan2(rotM[2, 1], rotM[2, 2]) * 180.0 / math.pi
    # camera coordinates
    x = tvec[0]
    y = tvec[1]
    z = tvec[2]
    (x, y) = RotateByZ(x, y, -1.0 * thetaZ)
    (x, z) = RotateByY(x, z, -1.0 * thetaY)
    (y, z) = RotateByX(y, z, -1.0 * thetaX)
    Cx = x * -1
    Cy = y * -1
    Cz = z * -1

    print("camera position:", Cx, Cy, Cz)
    print("camera rotation:", thetaX, thetaY, thetaZ)

    # Extra points for debug the accuracy
    '''
    Out_matrix = np.concatenate((rotM, tvec), axis=1)
    pixel = np.dot(camera_matrix, Out_matrix)
    pixel1 = np.dot(pixel, np.array([0, 100, 105, 1], dtype=np.double))
    pixel2 = pixel1/pixel1[2]
    print("test point coordinate:", pixel2) 
    '''


# detect_tags()
# solve()
# cv2.waitKey(0)

fname = "/home/qcraft/python/apriltag_2.jpg"
# fname = "/home/qcraft/python/apriltag_7.png"
# fname = "/home/qcraft/python/aruco_2.jpg"
# fname = "/home/qcraft/python/apriltag_1.png"


# fname = "2022-03-01_210423_01.bmp"

# aruco_detect(fname, dict=cv2.aruco.DICT_4X4_50)  # find all points
# aruco_detect(fname, dict=cv2.aruco.DICT_4X4_100)
# aruco_detect(fname, dict=cv2.aruco.DICT_4X4_250)
# aruco_detect(fname, dict=cv2.aruco.DICT_4X4_1000)
# aruco_detect(fname, dict=cv2.aruco.DICT_6X6_50)
# aruco_detect(fname, dict=cv2.aruco.DICT_6X6_100)
# aruco_detect(fname, dict=cv2.aruco.DICT_6X6_250)
# aruco_detect(fname, dict=cv2.aruco.DICT_6X6_1000)
# aruco_detect(fname, dict=cv2.aruco.DICT_APRILTAG_16h5)
# aruco_detect(fname, dict=cv2.aruco.DICT_APRILTAG_36h10)
# aruco_detect(fname, dict=cv2.aruco.DICT_APRILTAG_36h11)
# apriltag_detect(fname)  # can not find all points, can get error
apriltag_detect(fname, "tag36h11")
apriltag_detect(fname, "tag16h5")
# chessbaord_detect(fname) #cellphone image will crash
