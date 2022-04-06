import cv2
import numpy as np
import glob
import time


def get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file):
    images = glob.glob(file_path)

    # 找棋盘格角点(角点精准化迭代过程的终止条件)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE  # 11

    if not read_from_file:
        # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
        objp = np.zeros((1, Nx_cor * Ny_cor, 3), np.float32)
        objp[0, :, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2) * CheckerboardSize
        print(objp)

        # 储存棋盘格角点的世界坐标和图像坐标对
        objpoints = []  # 在世界坐标系中的三维点
        imgpoints = []  # 在图像平面的二维点
        gray = []
        image_name = []
        image_size = [0, 0]

        count = 0  # 用来标志成功检测到的棋盘格画面数量

        cv2.namedWindow("frame", 0)
        cv2.resizeWindow("frame", 1300, 900)
        image_index = 0

        while (image_index < len(images)):
            fname = images[image_index]
            image_index = image_index + 1
            frame = cv2.imread(fname)
            # cv2.imshow('frame', frame)
            print('NO.', image_index, ":", fname)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 寻找棋盘格模板的角点
            ok, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), flags)

            if ok:  # 如果找到，添加目标点，图像点
                count += 1
                objpoints.append(objp)
                cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 获取更精确的角点位置
                imgpoints.append(corners)
                image_name.append(fname)

                # 将角点在图像上显示
                cv2.drawChessboardCorners(frame, (Nx_cor, Ny_cor), corners, ok)
                cv2.imshow('frame', frame)
                cv2.waitKey(50)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()

        if count:
            image_size = gray.shape[:2][::-1]
            np.savez(calibrate_point_file, objpoints=objpoints, imgpoints=imgpoints, image_name=image_name,
                     image_size=image_size)
        else:
            print("can not find corner points!! get more correct image!!")

    else:
        calibrate_point = np.load(calibrate_point_file)

        objpoints = calibrate_point['objpoints']
        imgpoints = calibrate_point['imgpoints']
        image_name = calibrate_point['image_name']
        image_size = calibrate_point['image_size']

    return objpoints, imgpoints, image_name, image_size


def caculate_reproject_error(objpoints, imgpoints, mtx, dist, rvecs, tvecs, fisheye_flag=False):
    # 计算反投影误差
    mean_error = 0
    img_reproject_points = []
    img_undistor_points = []
    error_list = []
    # print(dist.shape, type(dist))
    # print(dist)
    dist_zero = np.copy(dist)
    for i in range(dist_zero.shape[0]):
        dist_zero[i] = [0]

    for i in range(len(objpoints)):
        if fisheye_flag:
            imgpoints2, _ = cv2.fisheye.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            imgpoints2 = np.reshape(imgpoints2, (-1, 1, 2))

            imgpoints3 = cv2.fisheye.undistortPoints(imgpoints, mtx, dist, None, mtx)

        else:
            imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
            imgpoints3 = cv2.undistortPoints(imgpoints2, mtx, dist, None, mtx)

        img_reproject_points.append(imgpoints2)
        img_undistor_points.append(imgpoints3)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
        mean_error += error
        error_list.append(error)
        # print("error: ", error)
    mean_error /= len(objpoints)
    std = 0
    for error in error_list:
        std += (error - mean_error) ** 2
    std /= len(objpoints)
    print("avery error: ", mean_error , std)

    return img_reproject_points, img_undistor_points


def common_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file,
                     calibrate_result_file,
                     flags=None):
    # flags
    # 可能为零或下列值的组合的不同标志:
    #
    # CALIB_USE_INTRINSIC_GUESS
    # cameraMatrix包含有效的初始值fx, fy, cx, cy进一步优化。否则，(cx, cy)
    # 初始设置为图像中心(使用imageSize)，并以最小二乘方式计算焦距。注意，如果已知内部参数，则不需要使用此函数来估计外部参数。而不是使用solvePnP.
    #
    # CALIB_FIX_PRINCIPAL_POINT
    # 在全局优化过程中，主点没有改变。当CALIB_USE_INTRINSIC_GUESS也被设置时，它会停留在指定的中心或不同的位置.
    #
    # CALIB_FIX_ASPECT_RATIO
    # 这些函数只将fy作为一个自由参数。fx / fy的比例与输入的camera
    # amatrix保持一致。当CALIB_USE_INTRINSIC_GUESS未设置时，fx和fy的实际输入值被忽略，只计算它们的比值并进一步使用.
    #
    # CALIB_ZERO_TANGENT_DIST
    # 切向畸变系数(p1, p2),设置为零并保持为零.
    #
    # CALIB_FIX_K1, ..., CALIB_FIX_K6
    # 优化过程中不改变相应的径向畸变系数。如果设置CALIB_USE_INTRINSIC_GUESS，则使用提供的distCoeffs矩阵的系数。否则设置为0.
    #
    # CALIB_RATIONAL_MODEL
    # 启用系数k4、k5和k6。为了提供向后兼容性，应该显式地指定这个额外的标志，以使校准函数使用rational模型并返回8个系数。如果没有设置该标志，该函数计算并只返回5个失畸变数.
    #
    # CALIB_THIN_PRISM_MODEL
    # 启用了系数s1、s2、s3和s4。为了提供向后兼容性，应该明确指定这个额外的标记，使校准功能使用瘦棱镜模型并返回12个系数。如果没有设置该标志，该函数计算并只返回5个畸变系数.
    #
    # CALIB_FIX_S1_S2_S3_S4
    # 优化过程中不改变薄棱镜畸变系数。如果设置CALIB_USE_INTRINSIC_GUESS, 则使用提供的distCoeffs矩阵的系数。否则设置为0.
    #
    # CALIB_TILTED_MODEL
    # 启用了系数tauX和tauY。为了提供向后兼容性，应该明确指定这个额外的标志，使校准功能使用倾斜的传感器模型并返回14个系数。如果没有设置该标志，该函数计算并只返回5个畸变系数.
    #
    # CALIB_FIX_TAUX_TAUY
    # 优化过程中不改变倾斜传感器模型的系数。如果设置CALIB_USE_INTRINSIC_GUESS，则使用提供的distCoeffs矩阵的系数。否则设置为0.
    objpoints, imgpoints, image_name, image_size = get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor,
                                                                     read_from_file,
                                                                     calibrate_point_file)

    # print(objpoints)
    # print(image_size)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

    print("flags = ", flags)

    # K = np.zeros((3, 3))
    # D = np.zeros((12, 1))
    # RR = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
    # TT = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
    #
    # K = np.ndarray([])
    # D = np.ndarray([])
    # RR = np.ndarray([])
    # TT = np.ndarray([])

    mtx = np.array([])
    dist = np.array([])
    rvecs = np.array([])
    tvecs = np.array([])

    # rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    #     objpoints, imgpoints, image_size, K, D, RR, TT, flags=flags, criteria=criteria)

    # rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    #     objpoints, imgpoints, image_size, mtx, dist, rvecs, tvecs, flags=flags, criteria=criteria)

    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None, flags=flags)

    # print(objpoints.shape)
    # print(imgpoints.shape)
    # print(len(rvecs))
    # print(len(tvecs))

    # calibrate_result = np.load(calibrate_result_file)
    # mtx = calibrate_result['mtx']
    # dist = calibrate_result['dist']
    # imgpoints = calibrate_result['imgpoints']
    # images = calibrate_result['image_name']
    # image_reproject_points = calibrate_result['image_reproject_points']
    # img_undistor_points = calibrate_result['img_undistor_points']

    # 摄像头内参mtx = [[f_x,0,c_x][0,f_y,c_y][0,0,1]]
    print('mtx= ', mtx)
    # 畸变系数dist = (k1,k2,p1,p2,k3)
    print('dist= ', dist)

    # ret, mtx, dist, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.calibrateCameraExtended(
    #     objpoints, imgpoints, image_size, flags, criteria)
    # print('mtx=' , str(mtx.tolist()) )
    # print('dist=' , str(dist.tolist()))
    # print("stdDeviationsIntrinsics=", stdDeviationsIntrinsics)
    # print("stdDeviationsExtrinsics=", stdDeviationsExtrinsics)
    # print("perViewErrors=", perViewErrors)

    image_reproject_points, img_undistor_points = caculate_reproject_error(objpoints, imgpoints, mtx, dist,
                                                                           rvecs, tvecs)

    np.savez(calibrate_result_file, rms=rms, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs,
             objpoints=objpoints, imgpoints=imgpoints, image_reproject_points=image_reproject_points,
             img_undistor_points=img_undistor_points, image_name=image_name)

    return rms, mtx, dist, rvecs, tvecs


def fisheye_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file,
                      calibrate_result_file):
    objpoints, imgpoints, image_name, image_size = get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor,
                                                                     read_from_file,
                                                                     calibrate_point_file)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
    # flags_fisheye = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW  # 14
    flags_fisheye = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW  # 14

    # 鱼眼/大广角镜头的单目标定
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    RR = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
    TT = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]

    rms, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(
        objpoints, imgpoints, image_size, K, D, RR, TT, flags_fisheye, criteria)

    # 摄像头内参mtx = [[f_x,0,c_x][0,f_y,c_y][0,0,1]]
    print('mtx=np.array( ' + str(mtx.tolist()) + " )")
    # 畸变系数dist = (k1,k2,k3,k4)
    print('dist=np.array( ' + str(dist.tolist()) + " )")
    # 同上
    # print("K=np.array( " + str(K.tolist()) + " )")
    # print("D=np.array( " + str(D.tolist()) + " )")

    image_reproject_points, img_undistor_points = caculate_reproject_error(objpoints, imgpoints, mtx, dist,
                                                                           rvecs, tvecs, True)

    np.savez(calibrate_result_file, rms=rms, mtx=mtx, dist=dist, rvecs=rvecs, tvecs=tvecs,
             objpoints=objpoints, imgpoints=imgpoints, image_reproject_points=image_reproject_points,
             img_undistor_points=img_undistor_points, image_name=image_name)

    return rms, mtx, dist, rvecs, tvecs


def save_origin_undistor_image(origin_img, undistor_img, image_path):
    # 照片 / 添加的文字 / 左上角坐标 / 字体 / 字体大小 / 颜色 / 字体粗细
    cv2.putText(origin_img, "origin", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)
    cv2.putText(undistor_img, "undistorted", (100, 100), cv2.FONT_HERSHEY_COMPLEX, 2, (0, 0, 255), 2)

    result = cv2.vconcat([origin_img, undistor_img])

    image_name = image_path.split('/')
    image_name.insert(-1, "result")
    new_name = image_name[-1]
    print("write to ", new_name)
    cv2.imwrite(new_name, result)

    cv2.namedWindow("undistorted", 0)
    cv2.resizeWindow("undistorted", 1500, 1000)
    cv2.imshow("undistorted", result)
    cv2.waitKey(100)
    cv2.destroyAllWindows()


def plot_reproject_point(img, points, color=[0, 0, 255], radius=5, thickness=2):
    img_size = img.shape
    # print(points.shape)
    for point in points:
        # print(point)
        row = int(point[0][0])
        col = int(point[0][1])
        if 0 <= row < img_size[1] and 0 <= col < img_size[0]:
            cv2.circle(img, (row, col), radius, color, thickness)


# fisheye
def undistor_image_1(K, D, file_path):
    images = glob.glob(file_path)

    img = cv2.imread(images[0])
    image_size = img.shape[:2][::-1]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, image_size, cv2.CV_16SC2)

    for i in range(len(images)):
        img = cv2.imread(images[i])
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        # print(undistorted_img.shape)
        save_origin_undistor_image(img, undistorted_img, images[i])


def undistort_image_2(calibrate_result_file, balance=0.0, dim2=None, dim3=None):
    calibrate_result = np.load(calibrate_result_file)
    K = calibrate_result['mtx']
    D = calibrate_result['dist']
    imgpoints = calibrate_result['imgpoints']
    images = calibrate_result['image_name']
    image_reproject_points = calibrate_result['image_reproject_points']
    img_undistor_points = calibrate_result['img_undistor_points']

    img = cv2.imread(images[0])
    DIM = img.shape[:2][::-1]
    dim1 = img.shape[:2][::-1]  # dim1 is the dimension of input image to un-distort
    # print(dim1)
    assert dim1[0] / dim1[1] == DIM[0] / DIM[
        1], "Image to undistort needs to have same aspect ratio as the ones used in calibration"
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1

    scaled_K = K * dim1[0] / DIM[0]  # The values of K is to scale with image dimension.
    scaled_K[2][2] = 1.0  # Except that K[2][2] is always 1.0

    print("k = ", K)
    print("scaled_K = ", scaled_K)
    print("D = ", D)

    # This is how scaled_K, dim2 and balance are used to determine the final K used to un-distort image. OpenCV document failed to make this clear!
    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_K, D, dim2, np.eye(3), balance=balance)
    print("new_k", new_K)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_K, D, np.eye(3), new_K, dim3, cv2.CV_16SC2)
    print(map1.shape, map2.shape)
    # print(map1[0], map2[0])

    for i in range(len(images)):
        img = cv2.imread(images[i])
        undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

        plot_reproject_point(img, imgpoints[i], [0, 0, 255], 3, -1)
        plot_reproject_point(img, image_reproject_points[i], [255, 0, 0], 5, 2)
        # plot_reproject_point(img, img_undistor_points[i], [0, 255, 0], 8, 2)

        undistor_points = []
        # for row in range(map1.shape[1]):
        #     for col in range(map1.shape[0]):
        #         if map1[col, row] in image_reproject_points[i]:
        #             undistor_points.append([[row, col]])

        # for corner in image_reproject_points[i]:
        #     row = int(corner[0][1])
        #     col = int(corner[0][0])
        #
        #     if 0 <= row < map1.shape[0] and 0 <= col < map1.shape[1]:
        #         # print("map", map1[row, col])
        #         undistor_points.append([[map1[row, col][0], map1[row, col][1]]])

        undistor_points = cv2.fisheye.undistortPoints(imgpoints[i], K, D, None, new_K)

        # plot_reproject_point(undistorted_img, img_reproject_undistor_points[i])
        plot_reproject_point(undistorted_img, undistor_points, [255, 0, 0])

        save_origin_undistor_image(img, undistorted_img, images[i])


# common camera
def undistor_image_3(calibrate_result_file):
    calibrate_result = np.load(calibrate_result_file)
    mtx = calibrate_result['mtx']
    dist = calibrate_result['dist']
    imgpoints = calibrate_result['imgpoints']
    images = calibrate_result['image_name']
    image_reproject_points = calibrate_result['image_reproject_points']
    img_undistor_points = calibrate_result['img_undistor_points']

    print(mtx)
    print(dist)

    for i in range(imgpoints.shape[0]):
        # print(img_undistor_points[i][0])
        img_undistor_points[i] = cv2.undistortPoints(imgpoints[i], mtx, dist, None, mtx)
        # print(img_undistor_points[i][0])
        # print("____________________")

    img = cv2.imread(images[0])
    h, w = img.shape[:2]
    # print([h, w])
    # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    print("newcameramtx=", newcameramtx)
    # print("------------------使用undistort函数-------------------")

    for i in range(len(images)):
        img = cv2.imread(images[i])
        undistorted_img = cv2.undistort(img, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        # dst1 = undistorted_img[y:y + h, x:x + w]
        #
        # cv2.imshow("dist", dst1)
        # cv2.waitKey(1000)

        plot_reproject_point(img, imgpoints[i], [0, 0, 255])
        # plot_reproject_point(img, image_reproject_points[i], [255, 0, 0], 5)

        undistor_points = cv2.undistortPoints(imgpoints[i], mtx, dist, None, newcameramtx)

        plot_reproject_point(undistorted_img, undistor_points)

        save_origin_undistor_image(img, undistorted_img, images[i])
        # save_origin_undistor_image(img[y:y + h, x:x + w], undistorted_img[y:y + h, x:x + w], images[i])


def write_calibrate_result():
    calibrate_result_file_old = "./common_calibrate.npz"
    calibrate_result_file_new = "./common_calibrate_sensing.npz"

    calibrate_result = np.load(calibrate_result_file_old)
    mtx = calibrate_result['mtx']
    dist = calibrate_result['dist']

    print(mtx)
    print(dist)

    mtx = np.array(
        [[1415.5686049218, 0.0, 1454.0765525772], [0.0, 1416.2189271160, 931.2262888285], [0.0, 0.0, 1.0]])
    dist = np.array(
        [[1.757492378], [2.7324359198], [-0.0000043238], [0.0000465382], [0.2252449931], [2.0551572513],
         [3.25692085224], [0.853026973]])

    print(mtx)
    print(dist)

    np.savez(calibrate_result_file_new, rms=calibrate_result['rms'], mtx=mtx, dist=dist,
             rvecs=calibrate_result['rvecs'], tvecs=calibrate_result['tvecs'],
             objpoints=calibrate_result['objpoints'], imgpoints=calibrate_result['imgpoints'],
             image_reproject_points=calibrate_result['image_reproject_points'],
             img_undistor_points=calibrate_result['img_undistor_points'],
             image_name=calibrate_result['image_name'])
    print("save!!")


def undistor_image_4(mtx, dist, file_path):
    images = glob.glob(file_path)
    img = cv2.imread(images[0])
    h, w = img.shape[:2]
    # print([h, w])
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
    # print(newcameramtx)

    mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, newcameramtx, (w, h), 5)  # 获取映射方程

    for i in range(len(images)):
        img = cv2.imread(images[i])
        undistorted_img = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)  # 重映射
        # dst = cv2.remap(img, mapx, mapy, cv2.INTER_CUBIC)  # 重映射后，图像变小了
        # x, y, w, h = roi
        # dst2 = dst[y:y + h, x:x + w]
        # cv2.imwrite('calibresult11_2.jpg', dst2)
        # print("方法二:dst的大小为:", dst2.shape)  # 图像比方法一的小

        save_origin_undistor_image(img, undistorted_img, images[i])


def solve_pnp(calibrate_result_file):
    calibrate_result = np.load(calibrate_result_file)
    cameraMatrix = calibrate_result['mtx']
    distCoeffs = calibrate_result['dist']
    imagePoints = calibrate_result['imgpoints']
    objectPoints = calibrate_result['objpoints']
    rvecs = calibrate_result['rvecs']
    tvecs = calibrate_result['tvecs']

    print(type(cameraMatrix))
    print(cameraMatrix.shape)
    print(distCoeffs.shape)
    print(imagePoints.shape)
    print(objectPoints.shape)

    retval, rvec, tvec, inliers = cv2.solvePnPRansac(objectPoints[0].reshape(-1, 1, 3),
                                                     imagePoints[0].reshape(-1, 1, 2),
                                                     cameraMatrix, distCoeffs)
    print(retval, rvec, tvec)

    retval, rvec, tvec, inliers = cv2.solvePnPRansac(objectPoints[0].reshape(-1, 1, 3),
                                                     imagePoints[0].reshape(-1, 1, 2),
                                                     cameraMatrix, distCoeffs)
    print(retval, rvec, tvec)

    print(rvecs[0], tvecs[0])


if __name__ == '__main__':
    write_calibrate_result()

    # file_path = "/home/qcraft/python/phone/*"
    file_path = "/home/qcraft/new/*.bmp"

    file_path = "/home/qcraft/calibrationdata/*.png"
    CheckerboardSize = 0.055
    max_iter = 30

    # Ny_cor = 9
    # Nx_cor = 17

    Ny_cor = 8
    Nx_cor = 8

    read_from_file = True
    # read_from_file = False
    # calibrate_point_file = "./calibratie_points.npz"
    calibrate_point_file = "./calibratie_points_50.npz"
    # calibrate_point_file = "./calibratie_points_phnoe.npz"
    # calibrate_point_file = "./calibratie_points_0001.npz"

    FISHEYE = 1
    COMMON = 0

    model = None
    model = FISHEYE
    # model = COMMON

    # calibrate_result_file = "./common_calibrate.npz"
    # solve_pnp(calibrate_result_file)
    if model == FISHEYE:
        calibrate_result_file = "./fisheye_calibrate.npz"
        rms, mtx, dist, rvecs, tvecs = fisheye_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file,
                                                         calibrate_point_file, calibrate_result_file)

        # undistor_image_1(mtx, dist, file_path)
        undistort_image_2(calibrate_result_file, balance=1, dim2=None, dim3=None)
    elif model == COMMON:
        # flags_list = [None, cv2.CALIB_RATIONAL_MODEL, cv2.CALIB_ZERO_TANGENT_DIST, cv2.CALIB_FIX_K3,
        #               cv2.CALIB_FIX_ASPECT_RATIO, cv2.CALIB_THIN_PRISM_MODEL, cv2.CALIB_TILTED_MODEL]
        # for flags in flags_list:
        #     ret, mtx, dist, rvecs, tvecs = common_calibrate(file_path, max_iter, Nx_cor, Ny_cor, read_from_file,
        #                                                     calibrate_point_file, flags)

        calibrate_result_file = "./common_calibrate_sensing.npz"
        # calibrate_result_file = "./common_calibrate.npz"
        # calibrate_result_file = "./common_calibrate_phone.npz"
        # calibrate_result_file = "./common_calibrate_0001.npz"

        flags = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO
        # flags = cv2.CALIB_RATIONAL_MODEL
        # flags = cv2.CALIB_THIN_PRISM_MODEL + cv2.CALIB_FIX_ASPECT_RATIO
        # flags=None
        ret, mtx, dist, rvecs, tvecs = common_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file,
                                                        calibrate_point_file, calibrate_result_file, flags)
        # undistor_image_3(calibrate_result_file)
        # undistor_image_4(calibrate_result_file)
