import cv2
import numpy as np
import glob
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
  print("save image : " + new_name)

  # cv2.namedWindow("result", 0)
  # cv2.resizeWindow("result", 1500, 1000)
  # cv2.imshow("result", result)
  # cv2.waitKey(0)
  # cv2.destroyAllWindows()


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


def undistort_img(img_name, alpha=0):
  calibrate_result_file = img_fold + "ost.yaml"
  save_path = img_fold + "undistort/"

  mtx, dist, w, h = load_calibrate_result(calibrate_result_file)

  img = cv2.imread(img_name)
  h, w = img.shape[:2]

  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), alpha, (w, h))
  # print(newcameramtx, roi)

  undistorted_img = cv2.undistort(img, mtx, dist, None, newcameramtx)
  save_images([img, undistorted_img], img_name, save_path, image_name_list)

  # cv2.namedWindow("undistorted", 0)
  # cv2.resizeWindow("undistorted", 1500, 1000)
  # cv2.imshow("undistorted", undistorted_img)
  # cv2.waitKey(1000)

  return undistorted_img, roi


def undistort_imgs(img_fold):
  img_file = img_fold + "*.png"
  calibrate_result_file = img_fold + "ost.yaml"
  save_path = img_fold + "undistort/"

  mtx, dist, w, h = load_calibrate_result(calibrate_result_file)

  new_mtx_list = []
  roi_list = []
  map1_list = []
  map2_list = []

  balance_list = [0.1 * i for i in range(11)]

  for balance in balance_list:
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), balance, (w, h))
    map1, map2 = cv2.initUndistortRectifyMap(mtx, dist, np.eye(3), new_mtx, (w, h), cv2.CV_16SC2)
    new_mtx_list.append(new_mtx)
    roi_list.append(roi)
    map1_list.append(map1)
    map2_list.append(map2)

    print(new_mtx)
    print(roi)

  image_name_list = ["origin", ]
  for balance in balance_list:
    name = "undistorted_" + str(balance)
    image_name_list.append(name)

  for file in glob.glob(save_path + "*"):
    os.remove(file)

  img_paths = glob.glob(img_file)[:10]
  for img_path in img_paths:
    img_name = img_path.split("/")[-1]
    img = cv2.imread(img_path)
    undistorted_img_list = [img, ]
    for i in range(len(new_mtx_list)):
      undistorted_img = cv2.remap(img, map1_list[i], map2_list[i], interpolation=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT)
      undistorted_img_list.append(undistorted_img)
    save_images(undistorted_img_list, img_name, save_path, image_name_list)


if __name__ == '__main__':
  input = sys.argv[1]
  if input[-1] == "/":
    undistort_imgs(input)
  elif input[-3:] == "jpg" or input[-3:] == "png" or input[-3:] == "bmp":
    undistort_img(input)
  else:
    img_fold = input + "/"
    undistort_imgs(input)


