import os
from show_param import *

# car_list = [file[:-7] for file in os.listdir(param_path) if file[0] == "Q"]
# for car in car_list:
#   print(car)
#   showVechicleExtrinsic(car)


devices = ["camera", "gnss", "lidar", "obc", "occ", "omc", "radar", "v2x", "vehicle"]
param_path = "/home/qcraft/vehicles/v2/"


def get_list_first_item(input):
  if type(input) == list:
    return input[0]
  else:
    return input


def merge_dict(dict_1, dict_2):
  # print(dict_1)
  # print(dict_2)
  for key in dict_2:
    if key in dict_1:
      if type(dict_1[key]) == dict and type(dict_2[key]) == dict:
        dict_1[key] = merge_dict(dict_1[key], dict_2[key])
      elif type(dict_1[key]) == list or type(dict_2[key]) == list:
        if type(dict_2[key]) == list:
          dict_1[key] = dict_2[key]
    else:
      dict_1[key] = dict_2[key]
  return dict_1


def print_dict(d, i=0):
  def print_sub_dict(sub_dict, key, i=0):
    print(" " * i * 2, key, " {")
    print_dict(sub_dict, i + 1)
    print(" " * i * 2, "}")

  for key in d:
    if type(d[key]) == dict:
      print_sub_dict(d[key], key, i + 1)
    elif type(d[key]) == list:
      for iter in d[key]:
        if type(iter) == dict:
          print_sub_dict(iter, key, i + 1)
        else:
          print(" " * i * 2, key, ": ", iter)
    else:
      print(" " * i * 2, key, ": ", d[key])


for device in devices:
  extrinsic_dir = "{}{}/installation/".format(param_path, device)
  file_list = [file for file in os.listdir(extrinsic_dir) if file[-6:] == "pb.txt"]
  extrinsic_sum = {}
  for file in file_list:
    extrinsic = decodePbTxt(extrinsic_dir + file)
    extrinsic_sum = merge_dict(extrinsic_sum, extrinsic)
  # print(device," : ",extrinsic_sum)
  print_dict(extrinsic_sum)
  print("\n" * 2)
