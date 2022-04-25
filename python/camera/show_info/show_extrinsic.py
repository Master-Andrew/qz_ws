import sys
from camera_intrinsic import *

lidar_intrinsic_path = "/home/qcraft/vehicles/v2/ldiar/inherent/"
lidar_extrinsic_path = "/home/qcraft/vehicles/v2/lidar/installation/"

radar_intrinsic_path = "/home/qcraft/vehicles/v2/radar/inherent/"
radar_extrinsic_path = "/home/qcraft/vehicles/v2/radar/installation/"


def showRadarExtrinsic(radar_extrinsic):
  sn = radar_extrinsic["sn"]
  id = radar_extrinsic["radar_id"]
  yaw = rad2degree(float(radar_extrinsic["extrinsics"]["yaw"]))
  pitch = rad2degree(float(radar_extrinsic["extrinsics"]["pitch"]))
  roll = rad2degree(float(radar_extrinsic["extrinsics"]["roll"]))
  x = float(radar_extrinsic["extrinsics"]["x"])
  y = float(radar_extrinsic["extrinsics"]["y"])
  z = float(radar_extrinsic["extrinsics"]["z"])

  print("{:20s}{:20s}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}".format(sn, id, yaw, pitch, roll, x, y, z))


def showLidaraExtrinsic(lidar_extrinsic):
  sn = lidar_extrinsic["sn"]
  id = lidar_extrinsic["lidar_id"]
  yaw = rad2degree(float(lidar_extrinsic["extrinsics"]["yaw"]))
  pitch = rad2degree(float(lidar_extrinsic["extrinsics"]["pitch"]))
  roll = rad2degree(float(lidar_extrinsic["extrinsics"]["roll"]))
  x = float(lidar_extrinsic["extrinsics"]["x"])
  y = float(lidar_extrinsic["extrinsics"]["y"])
  z = float(lidar_extrinsic["extrinsics"]["z"])

  print("{:20s}{:20s}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}".format(sn, id, yaw, pitch, roll, x, y, z))


def showCameraExtrinsic(camera_extrinsic, lidar_extrinsic):
  sn = camera_extrinsic["sn"]
  id = camera_extrinsic["camera_id"]
  yaw = rad2degree(float(camera_extrinsic["extrinsics"]["yaw"]) + float(lidar_extrinsic["extrinsics"]["yaw"]))
  pitch = rad2degree(float(camera_extrinsic["extrinsics"]["pitch"]) + float(lidar_extrinsic["extrinsics"]["pitch"]))
  roll = rad2degree(float(camera_extrinsic["extrinsics"]["roll"]) + float(lidar_extrinsic["extrinsics"]["roll"]))
  x = float(camera_extrinsic["extrinsics"]["x"]) + float(lidar_extrinsic["extrinsics"]["x"])
  y = float(camera_extrinsic["extrinsics"]["y"]) + float(lidar_extrinsic["extrinsics"]["y"])
  z = float(camera_extrinsic["extrinsics"]["z"]) + float(lidar_extrinsic["extrinsics"]["z"])
  ref_lidar_id = camera_extrinsic["ref_lidar_id"]
  device_path =  camera_extrinsic["device_path"]

  print("{:20s}{:20s}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^20s}{:^20s}".format(sn, id, yaw, pitch, roll, x, y, z,
                                                                                    ref_lidar_id,device_path))


def showVechicleExtrinsic(car_number):
  param_file = param_path + car_number + ".pb.txt"
  if not os.path.isfile(param_file):
    print(param_file, "do not exit!!")
    return

  lidar_extrinsic_dict = {}

  with open(param_file, "r") as f:
    lines = f.readlines()
    index = 0
    while index + 3 < len(lines):
      if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_LIDAR\n":
        lidar_sn = lines[index + 2].split()[1][1:-1]
        lidar_extrinsic = decodeExtrinsic(lidar_sn, lidar_extrinsic_path)
        lidar_extrinsic["sn"] = lidar_sn
        lidar_extrinsic_dict[lidar_extrinsic["lidar_id"]] = lidar_extrinsic
        showLidaraExtrinsic(lidar_extrinsic)
      index += 1

    print("-"*150)
    index = 0
    while index + 3 < len(lines):
      if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_CAMERA\n":
        # print(index,lines[index],lines[index+2])
        camera_sn = lines[index + 2].split()[1][1:-1]
        camera_extrinsic = decodeExtrinsic(camera_sn)
        camera_extrinsic["sn"] = camera_sn
        # print(camera_extrinsic)
        showCameraExtrinsic(camera_extrinsic, lidar_extrinsic_dict[camera_extrinsic["ref_lidar_id"]])
      index += 1

    print("-"*150)
    index = 0
    while index + 3 < len(lines):
      if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_RADAR\n":
        radar_sn = lines[index + 2].split()[1][1:-1]
        radar_extrinsic = decodeExtrinsic(radar_sn,radar_extrinsic_path)
        radar_extrinsic["sn"] = radar_sn
        showRadarExtrinsic(radar_extrinsic)
      index += 1


def print_header():
  print("{:20s}{:20s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}"
        .format("sn", "id", "yaw", "pitch", "roll", "x", "y", "z", "ref_lidar_id"))


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("please input vehicle number Qxxxx ")
    sys.exit()

  print_header()
  for i in range(1, len(sys.argv)):
    sn = sys.argv[i]
    if sn[0] == "Q":
      showVechicleExtrinsic(sn)

