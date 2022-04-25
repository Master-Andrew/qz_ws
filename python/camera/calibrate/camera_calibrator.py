import cv2
import numpy as np
import glob
import time
import os
import yaml

FISHEYE = 1
COMMON = 0


def mkdir(path):
  folder = os.path.exists(path)
  if not folder:  # 判断是否存在文件夹如果不存在则创建为文件夹
    os.makedirs(path)


def write_calibration_result(path, calibrate_result):
  with open(path + "intrinsic.yaml", "w") as intrinsic_file:
    intrinsic = {}
    intrinsic["image_width"] = int(calibrate_result["image_size"][0])
    intrinsic["image_height"] = int(calibrate_result["image_size"][1])

    intrinsic["camera_name"] = "narrow_stereo"

    intrinsic["camera_matrix"] = {}
    intrinsic["camera_matrix"]["rows"] = 3
    intrinsic["camera_matrix"]["cols"] = 3
    intrinsic["camera_matrix"]["data"] = calibrate_result["mtx"].tolist()

    intrinsic["camera_model"] = "rational_polynomial"

    intrinsic["distortion_coefficients"] = {}
    intrinsic["distortion_coefficients"]["rows"] = 1
    intrinsic["distortion_coefficients"]["cols"] = 8
    intrinsic["distortion_coefficients"]["data"] = calibrate_result["dist"][0][:8].tolist()

    intrinsic["rectification_matrix"] = {}
    intrinsic["rectification_matrix"]["rows"] = 3
    intrinsic["rectification_matrix"]["cols"] = 3
    intrinsic["rectification_matrix"]["data"] = np.eye(3).tolist()

    intrinsic["projection_matrix"] = {}
    intrinsic["projection_matrix"]["rows"] = 3
    intrinsic["projection_matrix"]["cols"] = 3
    intrinsic["projection_matrix"]["data"] = calibrate_result["new_mtx"].tolist()

    yaml.dump(intrinsic, intrinsic_file, sort_keys=False)

  # f = open(path + "intrinsic.yaml", "r", encoding="utf-8")
  # cfg = f.read()
  # print(cfg)
  # calibrate_result = yaml.load(cfg, Loader=yaml.FullLoader)
  # print(calibrate_result)


def get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file):
  images = glob.glob(file_path)

  # 找棋盘格角点(角点精准化迭代过程的终止条件)
  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
  flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE  # 11

  if not read_from_file:
    # 世界坐标系中的棋盘格点,例如(0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
    objp = np.zeros((1, Nx_cor * Ny_cor, 3), np.float32)
    objp[0, :, :2] = np.mgrid[0:Nx_cor, 0:Ny_cor].T.reshape(-1, 2) * CheckerboardSize
    # print(objp)

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
      # cv2.waitKey(100)
      print('NO.', image_index, ":", fname)

      gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
      # cv2.imshow('gray', gray)
      # cv2.waitKey(100)
      # 寻找棋盘格模板的角点

      print(Nx_cor, Ny_cor)
      ok, corners = cv2.findChessboardCorners(gray, (Nx_cor, Ny_cor), flags)
      print(corners)

      if ok:  # 如果找到，添加目标点，图像点
        count += 1
        objpoints.append(objp)
        cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)  # 获取更精确的角点位置
        imgpoints.append(corners)
        image_name.append(fname)

        # 将角点在图像上显示
        cv2.drawChessboardCorners(frame, (Nx_cor, Ny_cor), corners, ok)
        # cv2.imshow('frame', frame)
        # cv2.waitKey(100)

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
    # print("error: ", error)
  print("avery error: ", mean_error / len(objpoints))

  return img_reproject_points, img_undistor_points


def plot_reproject_point(img, points, color=[0, 0, 255], radius=5, thickness=2):
  img_size = img.shape
  # print(points.shape)
  for point in points:
    # print(point)
    row = int(point[0][0])
    col = int(point[0][1])
    if 0 <= row < img_size[1] and 0 <= col < img_size[0]:
      cv2.circle(img, (row, col), radius, color, thickness)


def common_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file,
                     calibrate_result_file, save_path, flags=None):
  objpoints, imgpoints, image_name, image_size = get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor,
                                                                   read_from_file, calibrate_point_file)

  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)

  print("flags = ", flags)

  rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, image_size, None, None, flags=flags, criteria=criteria)

  print(type(mtx), type(dist))
  print(mtx.shape, dist.shape)

  # 摄像头内参mtx = [[f_x,0,c_x][0,f_y,c_y][0,0,1]]
  print('mtx= ', mtx)
  # 畸变系数dist = (k1,k2,p1,p2,k3)
  print('dist= ', dist)

  w, h = image_size
  new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

  # ret, mtx, dist, rvecs, tvecs, stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors = cv2.calibrateCameraExtended(
  #     objpoints, imgpoints, image_size, flags, criteria)
  # print('mtx=' , str(mtx.tolist()) )
  # print('dist=' , str(dist.tolist()))
  # print("stdDeviationsIntrinsics=", stdDeviationsIntrinsics)
  # print("stdDeviationsExtrinsics=", stdDeviationsExtrinsics)
  # print("perViewErrors=", perViewErrors)

  image_reproject_points, img_undistor_points = caculate_reproject_error(objpoints, imgpoints, mtx, dist,
                                                                         rvecs, tvecs)

  np.savez(calibrate_result_file, rms=rms, mtx=mtx, dist=dist, new_mtx=new_mtx, roi=roi, rvecs=rvecs, tvecs=tvecs,
           objpoints=objpoints, imgpoints=imgpoints, image_reproject_points=image_reproject_points,
           img_undistor_points=img_undistor_points, image_name=image_name, image_size=image_size)

  calibrate_result = np.load(calibrate_result_file)
  write_calibration_result(save_path, calibrate_result)

  return rms, mtx, dist, rvecs, tvecs


def fisheye_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file,
                      calibrate_result_file, flags):
  objpoints, imgpoints, image_name, image_size = get_img_obj_point(file_path, CheckerboardSize, Nx_cor, Ny_cor,
                                                                   read_from_file, calibrate_point_file)

  criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)  # (3,27,1e-6)
  flags_fisheye = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
  # flags_fisheye = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

  # 鱼眼/大广角镜头的单目标定
  K = np.zeros((3, 3))
  D = np.zeros((4, 1))
  RR = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]
  TT = [np.zeros((1, 1, 3), dtype=np.float64) for _ in range(len(objpoints))]

  mtx = []
  dist = []
  rvecs = []
  tvecs = []
  print(objpoints.shape, imgpoints.shape, type(objpoints), type(imgpoints))
  rms, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(
    objpoints, imgpoints, image_size, K, D, RR, TT, flags_fisheye, criteria)

  # rms, mtx, dist, rvecs, tvecs = cv2.fisheye.calibrate(
  #     objpoints, imgpoints, image_size, K, D, RR, TT, flags_fisheye, criteria)
  # calibrate(objectPoints, imagePoints, image_size, K, D, rvecs=None, tvecs=None, flags=None, criteria=None)

  print('mtx=np.array( ' + str(mtx.tolist()) + " )")
  print('dist=np.array( ' + str(dist.tolist()) + " )")

  new_mtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx, dist, image_size, np.eye(3), balance=1)
  roi = 0, 0, image_size[0], image_size[1]

  image_reproject_points, img_undistor_points = caculate_reproject_error(objpoints, imgpoints, mtx, dist,
                                                                         rvecs, tvecs, True)

  np.savez(calibrate_result_file, rms=rms, mtx=mtx, dist=dist, new_mtx=new_mtx, roi=roi, rvecs=rvecs, tvecs=tvecs,
           objpoints=objpoints, imgpoints=imgpoints, image_reproject_points=image_reproject_points,
           img_undistor_points=img_undistor_points, image_name=image_name, image_size=image_size)

  return rms, mtx, dist, rvecs, tvecs


def fisheye_calibrate_2(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file, calibrate_point_file,
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
  new_mtx = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(mtx, dist, image_size, np.eye(3), balance=1)
  roi = 0, 0, image_size[0], image_size[1]

  image_reproject_points, img_undistor_points = caculate_reproject_error(objpoints, imgpoints, mtx, dist,
                                                                         rvecs, tvecs, True)

  np.savez(calibrate_result_file, rms=rms, mtx=mtx, dist=dist, new_mtx=new_mtx, roi=roi, rvecs=rvecs, tvecs=tvecs,
           objpoints=objpoints, imgpoints=imgpoints, image_reproject_points=image_reproject_points,
           img_undistor_points=img_undistor_points, image_name=image_name, image_size=image_size)

  return rms, mtx, dist, rvecs, tvecs


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
  # cv2.waitKey(100)
  cv2.destroyAllWindows()


# fisheye

def undistort_image_fisheye(calibrate_result_file, save_path, balance=0.0, dim2=None, dim3=None):
  calibrate_result = np.load(calibrate_result_file)
  K = calibrate_result['mtx']
  D = calibrate_result['dist']
  imgpoints = calibrate_result['imgpoints']
  images = calibrate_result['image_name']
  image_reproject_points = calibrate_result['image_reproject_points']
  img_undistor_points = calibrate_result['img_undistor_points']

  print(K, D)

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

  # cv2.fisheye.undistortImage()

  for i in range(len(images)):
    img = cv2.imread(images[i])
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # undistorted_img_2 = cv2.fisheye.undistortImage(img, K, D, None, new_K)

    plot_reproject_point(img, imgpoints[i], [0, 0, 255], 3, -1)
    plot_reproject_point(img, image_reproject_points[i], [255, 0, 0], 5, 2)

    undistor_points = cv2.fisheye.undistortPoints(imgpoints[i], K, D, None, new_K)

    plot_reproject_point(undistorted_img, undistor_points, [255, 0, 0])

    save_origin_undistor_image(img, undistorted_img, images[i], save_path)


# common camera
def undistor_image_common(calibrate_result_file, save_path):
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
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 0, (w, h))
  # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
  print("newcameramtx=", newcameramtx)
  # print("------------------使用undistort函数-------------------")

  # map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, np.eye(3), newcameramtx, (w, h), cv2.CV_16SC2)
  # undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

  for i in range(len(images)):
    img = cv2.imread(images[i])
    undistorted_img = cv2.undistort(img, mtx, dist, None, newcameramtx)
    # x, y, w, h = roi
    # dst1 = dst[y:y + h, x:x + w]

    plot_reproject_point(img, imgpoints[i], [0, 0, 255])
    # plot_reproject_point(img, image_reproject_points[i], [255, 0, 0], 5)

    undistor_points = cv2.undistortPoints(imgpoints[i], mtx, dist, None, newcameramtx)

    plot_reproject_point(undistorted_img, undistor_points)

    save_origin_undistor_image(img, undistorted_img, images[i], save_path)


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
           image_name=calibrate_result['image_name'], image_size=calibrate_result['image_size'])
  print("save!!")


def calibrator(file_path, CheckerboardSize, max_iter, Ny_cor, Nx_cor, read_from_file, calibrate_point_file, model,
               calibrate_result_file, flags, save_path):
  if model == FISHEYE:
    fisheye_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file,
                      calibrate_point_file, calibrate_result_file, flags)

    undistort_image_fisheye(calibrate_result_file, save_path, balance=1, dim2=None, dim3=None)

  elif model == COMMON:
    common_calibrate(file_path, CheckerboardSize, Nx_cor, Ny_cor, read_from_file,
                     calibrate_point_file, calibrate_result_file, save_path, flags)

    undistor_image_common(calibrate_result_file, save_path)


def reproject(calibrate_point_file, mtx, dist, save_path):
  calibrate_point = np.load(calibrate_point_file)

  objpoints = calibrate_point['objpoints']
  imgpoints = calibrate_point['imgpoints']
  image_name = calibrate_point['image_name']
  image_size = calibrate_point['image_size']

  w, h = image_size
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))
  # print("newcameramtx=", newcameramtx)

  for i in range(len(image_name)):
    img = cv2.imread(image_name[i])

    retval, rvec, tvec, inliers = cv2.solvePnPRansac(objpoints[i].reshape(-1, 1, 3),
                                                     imgpoints[i].reshape(-1, 1, 2),
                                                     mtx, dist)
    reprojected_point, _ = cv2.projectPoints(objpoints[i], rvec, tvec, mtx, dist)
    undistorted_point = cv2.undistortPoints(reprojected_point, mtx, dist, None, newcameramtx)

    plot_reproject_point(img, imgpoints[i], [255, 0, 0], 3, -1)
    plot_reproject_point(img, reprojected_point)

    undistorted_img = cv2.undistort(img, mtx, dist, None, newcameramtx)
    plot_reproject_point(undistorted_img, undistorted_point, [0, 255, 0], 8)
    print(retval, rvec, tvec)

    save_origin_undistor_image(img, undistorted_img, image_name[i], save_path)


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
  # write_calibrate_result()

  flags = cv2.CALIB_RATIONAL_MODEL + cv2.CALIB_FIX_ASPECT_RATIO
  # flags = cv2.CALIB_RATIONAL_MODEL
  # flags=None

  calibrator("/home/qcraft/camera_calibration/image_data/2/*.png", 0.05, 50, 8, 8, False, "calibratie_points_2.npz",
             COMMON, "common_calibrate_2.npz", flags, "/home/qcraft/camera_calibration/image_data/2/result/")

  # calibrator("/home/qcraft/lidar_camera_ws/resource/M1/*.jpg", 0.05, 50, 5, 7, False, "./calibratie_points_m1.npz",
  #            COMMON, "./common_calibrate_ms.npz", flags, "/home/qcraft/lidar_camera_ws/resource/M1/result")

  # calibrator("/home/qcraft/camera_calibration/image_data/sensing/*.bmp",
  #            0.05, 30, 9, 17, True, "./result/calibratie_points_50.npz", COMMON,
  #            "./result/common_calibrate.npz", flags,
  #            "/home/qcraft/camera_calibration/image_data/sensing/common/")

  # flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_FIX_SKEW
  # calibrator("/home/qcraft/camera_calibration/image_data/sensing/*.bmp",
  #            0.05, 30, 9, 17, True, "./result/calibratie_points_50.npz", FISHEYE,
  #            "./result/fisheye_calibrate.npz", flags,
  #            "/home/qcraft/camera_calibration/image_data/sensing/fisheye/")

  # calibrator("/home/qcraft/calibrationdata/*.png", 0.055, 30, 8, 8, True, "./calibratie_points_0001.npz", COMMON,
  #            "./common_calibrate_0001.npz", flags, "/home/qcraft/Documents/camera_calibration/0001/")

  # calibrate_point_file = "./calibratie_points_50.npz"
  # mtx = np.array(
  #     [[1415.56860492181, 0.0, 1454.0765525772], [0.0, 1416.2189271160, 931.2262888285], [0.0, 0.0, 1.0]])
  # dist = np.array([[1.7574923784, 2.7324359198, -0.0000043238, 0.0000465382, 0.2252449931, 2.0551572513,
  #                   3.2569208522, 0.8530269734]])
  # save_path = "./"
  # reproject(calibrate_point_file, mtx, dist, save_path)
