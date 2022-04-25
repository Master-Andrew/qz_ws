from math import atan2, tan
import os
import sys
from camera_intrinsic import *


def distor(x, y, k1, k2, k3, p1, p2, k4=0, k5=0, k6=0):
  r = (x ** 2 + y ** 2) ** 0.5
  rate = (1 + k1 * r ** 2 + k2 * r ** 4 + k3 * r ** 6) / (1 + k4 * r ** 2 + k5 * r ** 4 + k6 * r ** 6)
  x_distor = x * rate + (2 * p1 * x * y + p2 * (r ** 2 + 2 * x ** 2))
  y_distor = y * rate + (p1 * (r ** 2 + 2 * y ** 2) + 2 * p2 * x * y)
  # print("distor : ", x, y, x_distor, y_distor, rate)
  return x_distor, y_distor, rate


def find_distor(x, y, k1, k2, k3, p1, p2, k4=0, k5=0, k6=0):
  # print(x, y, k1, k2, k3, p1, p2, k4, k5, k6)
  rate = 0.5
  x_diff = x
  y_diff = y
  x_undistor = x + rate * x_diff
  y_undistor = y + rate * y_diff

  while x_diff > 1 or y_diff > 1:
    x_undistor -= rate * x_diff
    y_undistor -= rate * y_diff
    x_distor, y_distor, rate = distor(x_undistor, y_undistor, k1, k2, k3, p1, p2, k4, k5, k6)
    x_diff = x_distor - x
    y_diff = y_distor - y

  # print(x, y, x_undistor, y_undistor)


def calculateFOV(sn):
  intrinsic = decodeIntrinsic(sn)
  extrinsic = decodeExtrinsic(sn)

  if intrinsic == {} or extrinsic == {} or \
    not "intrinsics" in intrinsic:
    return
  # print(intrinsic)
  fx = float(intrinsic["intrinsics"]["camera_matrix"]["fx"])
  fy = float(intrinsic["intrinsics"]["camera_matrix"]["fy"])
  cx = float(intrinsic["intrinsics"]["camera_matrix"]["cx"])
  cy = float(intrinsic["intrinsics"]["camera_matrix"]["cy"])

  if "rotate_90_ccw" in extrinsic:
    if extrinsic["rotate_90_ccw"] == "true":
      fx, fy = fy, fx
      cx, cy = cy, cx
  else:
    extrinsic["rotate_90_ccw"] = "false"

  if "full_undistort_fov" not in extrinsic:
    extrinsic["full_undistort_fov"] = "false"

  origin_hfov = rad2degree(atan2(cx, fx)) * 2
  origin_vfov = rad2degree(atan2(cy, fy)) * 2
  origin_rfov = rad2degree(atan2((cy ** 2 + cx ** 2) ** 0.5, fy)) * 2

  if "roi" in extrinsic:
    x = int(extrinsic["roi"]["x"])
    y = int(extrinsic["roi"]["y"])
    width = int(extrinsic["roi"]["width"])
    height = int(extrinsic["roi"]["height"])

    roi_hfov = rad2degree(atan2(cx - x, fx)) + rad2degree(atan2(x + width - cx, fx))
    roi_vfov = rad2degree(atan2(cy - y, fy)) + rad2degree(atan2(y + height - cy, fy))
    roi_rfov = rad2degree(atan2(((cy - y) ** 2 + (cx - x) ** 2) ** 0.5, fy)) \
               + rad2degree(atan2(((y + height - cy) ** 2 + (x + width - cx) ** 2) ** 0.5, fy))
  else:
    roi_hfov = 0
    roi_vfov = 0
    roi_rfov = 0

  k1 = float(intrinsic["intrinsics"]["distort_coeffs"]["k1"])
  k2 = float(intrinsic["intrinsics"]["distort_coeffs"]["k2"])
  k3 = float(intrinsic["intrinsics"]["distort_coeffs"]["k3"])

  if "k4" in intrinsic["intrinsics"]["distort_coeffs"]:
    k4 = float(intrinsic["intrinsics"]["distort_coeffs"]["k4"])
    k5 = float(intrinsic["intrinsics"]["distort_coeffs"]["k5"])
    k6 = float(intrinsic["intrinsics"]["distort_coeffs"]["k6"])
  else:
    k4 = 0
    k5 = 0
    k6 = 0

  p1 = float(intrinsic["intrinsics"]["distort_coeffs"]["p1"])
  p2 = float(intrinsic["intrinsics"]["distort_coeffs"]["p2"])

  # find_distor(cx, 0, k1, k2, k3, p1, p2, k4, k5, k6)
  # find_distor(0, cy, k1, k2, k3, p1, p2, k4, k5, k6)
  # find_distor(cx, cy, k1, k2, k3, p1, p2, k4, k5, k6)

  print("{:20s}{:20s}{:^15.2f}{:^15.2f}{:^15s}{:^20s}{:^15.2f}{:^15.2f}"
        .format(sn, extrinsic["camera_id"], origin_hfov, origin_vfov,
                extrinsic["rotate_90_ccw"], extrinsic["full_undistort_fov"], roi_hfov, roi_vfov))


def showVechicleFOV(car_number):
  param_file = param_path + car_number + ".pb.txt"
  if not os.path.isfile(param_file):
    print(param_file, "do not exit!!")
    return

  with open(param_file, "r") as f:
    lines = f.readlines()
    index = 0
    while index + 3 < len(lines):
      if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_CAMERA\n":
        camera_sn = lines[index + 2].split()[1][1:-1]
        calculateFOV(camera_sn)
      index += 1


def print_header():
  print("{:20s}{:20s}{:^15s}{:^15s}{:^15s}{:^20s}{:^15s}{:^15s}"
        .format("camera_sn", "camera_id", "origin_hfov", "origin_vfov", "rotate_90_ccw",
                "full_undistort_fov", "roi_hfov", "roi_vfov"))


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("please input vehicle number Qxxxx or camera sn Hxxx_xxxxxxx")
    sys.exit()

  print_header()
  for i in range(1, len(sys.argv)):
    sn = sys.argv[i]
    if sn[0] == "Q":
      showVechicleFOV(sn)
    elif sn[0] == "H":
      calculateFOV(sn)

