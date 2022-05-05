import sys
import os
from load_csv import *

camera_extrinsic_path = "/home/qcraft/vehicles/v2/camera/installation/"
camera_device_path = {"CAM_L_LEFT": 2,
                      "CAM_L_FRONT_LEFT": 1,
                      "CAM_L_FRONT": 0,
                      "CAM_L_FRONT_RIGHT": 5,
                      "CAM_R_RIGHT": 4,
                      "CAM_R_REAR_RIGHT": 2,
                      "CAM_L_REAR_LEFT": 0,
                      "CAM_R_FRONT_RIGHT": 5,
                      "CAM_R_FRONT_DIM": 1,
                      "CAM_R_FRONT_HD": 6}


def show_file(file):
  with open(file, "r") as f:
    for line in f.readlines():
      print(line, end="")
    print("-" * 30)


def write_camera_extrinsic(camera_id, camera_sn, camera_extrinsic):
  camera_extrinsic_file = camera_extrinsic_path + camera_sn + ".pb.txt"
  if os.path.isfile(camera_extrinsic_file):
    print(camera_extrinsic_file, " exit!!")
    overwrite=input("do you want to overwrite:y/n")

  else:
    with open(camera_extrinsic_file, "w") as f:
      f.write("camera_id: {}\n".format(camera_id))
      f.write("extrinsics {\n")
      f.write("  calibration_time: \"\"\n")
      f.write("  x: {}\n".format(camera_extrinsic["x"]))
      f.write("  y: {}\n".format(camera_extrinsic["y"]))
      f.write("  z: {}\n".format(camera_extrinsic["z"]))
      f.write("  yaw: {}\n".format(camera_extrinsic["yaw"]))
      f.write("  pitch: {}\n".format(camera_extrinsic["pitch"]))
      f.write("  roll: {}\n".format(camera_extrinsic["roll"]))
      f.write("  calibration_engineer: \"\"\n")
      f.write("}\n")
      f.write("ref_lidar_id: {}\n".format(camera_extrinsic["ref_lidar_id"]))
      f.write("device_path: \"/dev/video{}\"\n".format(camera_device_path[camera_id]))
      f.write("hardware_trigger: true\n")
      f.write("rotate_90_ccw: false\n")
      f.write("expected_fps: 10\n")
    print("write camera extrinsic file : ", camera_extrinsic_file, " !!!!")

  show_file(camera_extrinsic_file)


def main(position_file, extrinsic_file, car_number):
  position = load_position_file(position_file)[car_number]
  extrinsic = load_position_file(extrinsic_file)

  for camera_id in position:
    write_camera_extrinsic(camera_id, position[camera_id], extrinsic[camera_id])


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print("please input vehicle number Qxxxx")
    sys.exit()
  position_file = "config/camera_position.csv"
  extrinsic_file = "config/MKZ_DBQ3.csv"

  for i in range(1, len(sys.argv)):
    sn = sys.argv[i]
    if sn[0] == "Q":
      main(position_file, extrinsic_file, sn)
