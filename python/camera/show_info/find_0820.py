import os
from show_param import *

param_path = "/home/qcraft/vehicles/v2/"
device_list = ["camera", "gnss", "lidar", "obc", "omc", "radar", "v2x", "vehicle"]
camera_inherent_path = "/home/qcraft/vehicles/v2/camera/inherent/"


def ReadVechicleParam(param_file):
  # param_file = param_path + car_number + ".pb.txt"
  car_number = param_file.split("/")[-1].split(".")[0]
  if not os.path.isfile(param_file):
    print(param_file, "do not exit!!")
    return
  with open(param_file, "r") as f:
    lines = f.readlines()
    index = 0
    while index + 3 < len(lines):
      device = lines[index + 3].split("_")[-1][:-1].lower()
      if device == "camera":
        sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        # print(model, end="\t")
        if True :  #model.split("_")[1] == "H30" and (car_number[1] in ["1", "2"])
          camera_file = "{}{}/installation/{}.pb.txt".format(param_path, "camera", sn)
          extrinsic = decodePbTxt(camera_file)
          camera_file = "{}{}/inherent/{}.pb.txt".format(param_path, "camera", sn)
          intrinsic = decodePbTxt(camera_file)
          print(car_number, model, sn, extrinsic["camera_id"], intrinsic["intrinsics"]["camera_matrix"]["fx"])
          for key in ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", ]:
            try:
              print(intrinsic["intrinsics"]["distort_coeffs"][key], end=",")
            except:
              print("#############", end=",")
          print()

      index += 1


def find_from_vehicle():
  file_list = [file for file in os.listdir(param_path) if file[-6:] == "pb.txt"]
  for file in file_list:
    if file[:3] == "Q60":
      ReadVechicleParam(param_path + file)


def find_from_camera():
  file_list = [file for file in os.listdir(camera_inherent_path) if file[-6:] == "pb.txt"]
  for file in file_list:
    if file[:3] == "H30":
      intrinsic = decodePbTxt(camera_inherent_path + file)
      if "k4" not in intrinsic:
        print(file[:-7])


if __name__ == "__main__":
  find_from_vehicle()
  # find_from_camera()
