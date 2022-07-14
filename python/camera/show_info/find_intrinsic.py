import csv
import os

camera_intrinsic_path = "/home/qcraft/vehicles/v2/camera/inherent"


def GetCameraList(csv_file, intrinsic_list):
  camera_list = []
  with open(csv_file, 'r', newline='', encoding='UTF-8-SIG') as fi:
    fr = csv.reader(fi)
    counter = 0
    for row in fr:
      # print(row)
      print(row[0], " , ", row[1], end="")
      for sn in row[2: len(row)]:
        if len(sn) != 0 :#len(sn) != 0 and sn not in intrinsic_list
          print(" , ", sn, end="")
          counter += 1
          camera_list.append(sn)
      print("")
      print(counter)
      counter = 0
      # print(len(camera_list), camera_list)
      # for index, sn in enumerate(camera_list):
      #   print(index, sn, end="/ ")


def GetCameraIntrinsicList():
  intrinsic_list = [file[:-7] for file in os.listdir(camera_intrinsic_path) if file[-6:] == "pb.txt" and file[0] == "H"]
  print(intrinsic_list)
  return intrinsic_list


if __name__ == "__main__":
  csv_file = "/home/qcraft/Documents/camera_sn.csv"
  intrinsic_list = GetCameraIntrinsicList()
  camera_list = GetCameraList(csv_file, intrinsic_list)
