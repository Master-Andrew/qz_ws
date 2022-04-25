import sys

from camera_intrinsic import *


def showIntrinsic(sn):
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

  print("{:20s}{:20s}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}".format(sn, extrinsic["camera_id"], fx, fy, cx, cy))


def showVechicleIntrinsic(car_number):
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
        showIntrinsic(camera_sn)
      index += 1


def print_header():
  print("{:20s}{:20s}{:^15s}{:^15s}{:^15s}{:^20s}"
        .format("camera_sn", "camera_id", "fx", "fy", "cx", "cy"))


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("please input vehicle number Qxxxx or camera sn Hxxx_xxxxxxx")
    sys.exit()

  print_header()
  for i in range(1, len(sys.argv)):
    sn = sys.argv[i]
    if sn[0] == "Q":
      showVechicleIntrinsic(sn)
    elif sn[0] == "H":
      showIntrinsic(sn)
