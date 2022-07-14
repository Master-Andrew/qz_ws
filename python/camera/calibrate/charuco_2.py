import cv2 as cv
import numpy as np

# 创建ChArUco标定板
dictionary = cv.aruco.getPredefinedDictionary(dict=cv.aruco.DICT_6X6_250)
input_ids = [i for i in range(17, 34)]
board = cv.aruco.CharucoBoard_create(squaresY=7,
                                     squaresX=5,
                                     squareLength=0.04,
                                     markerLength=0.02,
                                     dictionary=dictionary)
board.ids = input_ids
img_board = board.draw(outSize=(600, 500), marginSize=10, borderBits=1)

cv.imwrite(filename='charuco.png', img=img_board, params=None)

camera_matrix = np.array([[532.79536562, 0, 342.4582516],
                          [0, 532.91928338, 233.90060514],
                          [0, 0, 1.]])
dist_coefs = np.array([-2.81086258e-01, 2.72581018e-02, 1.21665908e-03, -1.34204275e-04, 1.58514022e-01])

# 主要用于图形的绘制与显示
img_color = cv.cvtColor(src=img_board,
                        code=cv.COLOR_GRAY2BGR,
                        dstCn=None)

# 查找标志块的左上角点
corners, ids, rejectedImgPoints = cv.aruco.detectMarkers(image=img_board,
                                                         dictionary=dictionary,
                                                         parameters=None,
                                                         cameraMatrix=camera_matrix,
                                                         distCoeff=dist_coefs)
# print(corners)
# print(ids)
if len(ids) > 0:
    print((int(corners[0][0][0][0]), int(corners[0][0][0][1])))
    print((int(corners[1][0][0][0]), int(corners[1][0][0][1])))
    cv.circle(img_color, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), 8, [0, 255, 0])
    cv.circle(img_color, (int(corners[1][0][0][0]), int(corners[1][0][0][1])), 8, [0, 255, 0])
    cv.circle(img_color, (int(corners[2][0][0][0]), int(corners[2][0][0][1])), 8, [0, 255, 0])
    # 绘制标志块的左上角点与对应的ID
    cv.aruco.drawDetectedMarkers(image=img_color, corners=corners, ids=ids, borderColor=None)
    cv.imshow("out", img_color)
    cv.waitKey()

    # 棋盘格黑白块内角点
    retval, charucoCorners, charucoIds = cv.aruco.interpolateCornersCharuco(markerCorners=corners,
                                                                            markerIds=ids,
                                                                            image=img_board,
                                                                            board=board,
                                                                            cameraMatrix=camera_matrix,
                                                                            distCoeffs=dist_coefs)
    print(charucoCorners)
    print( )
    if len(charucoIds) > 0:
        # 绘制棋盘格黑白块内角点
        cv.aruco.drawDetectedCornersCharuco(img_color, charucoCorners, charucoIds, [0, 0, 255])
        cv.imshow("out", img_color)
        cv.waitKey()

        rvec = None
        tvec = None
        retval, rvec, tvec = cv.aruco.estimatePoseCharucoBoard(charucoCorners, charucoIds, board, camera_matrix,
                                                               dist_coefs, rvec, tvec)
        if retval:
            cv.aruco.drawAxis(img_color, camera_matrix, dist_coefs, rvec, tvec, 0.01)

cv.imshow("out", img_color)
cv.waitKey()
