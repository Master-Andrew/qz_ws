import os
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Ellipse, Circle

camera_intrinsic_origin = {'SN': '', 'calibration_time': '',
                           'k1': '0', 'k2': '0', 'p1': '0', 'p2': '0', 'k3': '0', 'k4': '0', 'k5': '0', 'k6': '0',
                           'fx': '0', 'fy': '0', 'cx': '0', 'cy': '0',
                           'serial_no': '0', 'calibration_engineer': '0', 'rms': '0',
                           'max_fps': '0', 'mid_row_time_since_trigger': '0', 'rolling_elapse_time': '0',
                           'camera_vendor': ''}


def decode_camera_intrinsic(file):
  camera_intrinsic = camera_intrinsic_origin.copy()
  camera_intrinsic["SN"] = file.split("/")[-1]
  with open(file, "r") as f:
    lines = f.readlines()
    for line in lines:
      if line == "\n":
        continue
      data = line.split('=')
      name = data[0]
      value = data[1].split()[0]
      camera_intrinsic[name] = value
  # print(camera_intrinsic)
  return camera_intrinsic


def decode_camera_intrinsic_2(file):
  camera_intrinsic = camera_intrinsic_origin.copy()
  camera_intrinsic["SN"] = file.split("/")[-1]
  with open(file, "r") as f:
    lines = f.readlines()
    for line in lines:
      data = line.split()
      if len(data) == 0:
        continue
      if data[-1] == "{" or data[-1] == "}":
        continue
      name = data[0][:-1]
      value = data[1]
      camera_intrinsic[name] = value
  # print(camera_intrinsic)
  return camera_intrinsic


def load_all_camera_intrinsic(path, head):
  name_list = os.listdir(path)
  camera_intrinsic_list = []
  for name in name_list:
    if name[:len(head)] == head:
      camera_intrinsic_list.append(decode_camera_intrinsic_2(path + name))
  print("find {} intrinsic files.".format(len(camera_intrinsic_list)))
  for i in range(min(10, len(camera_intrinsic_list))):
    print(camera_intrinsic_list[i])
  return camera_intrinsic_list


def plot_camera_intrinsic_distribute(camera_intrinsic_list):
  factor = 3

  fx_list = []
  fy_list = []
  cx_list = []
  cy_list = []

  k1_list = []
  k2_list = []
  k3_list = []
  k4_list = []
  k5_list = []
  k6_list = []

  p1_list = []
  p2_list = []

  for camera_intrinsic in camera_intrinsic_list:
    fx_list.append(float(camera_intrinsic["fx"]))
    fy_list.append(float(camera_intrinsic["fy"]))
    cx_list.append(float(camera_intrinsic["cx"]))
    cy_list.append(float(camera_intrinsic["cy"]))

    k1_list.append(float(camera_intrinsic["k1"]))
    k2_list.append(float(camera_intrinsic["k2"]))
    k3_list.append(float(camera_intrinsic["k3"]))
    k4_list.append(float(camera_intrinsic["k4"]))
    k5_list.append(float(camera_intrinsic["k5"]))
    k6_list.append(float(camera_intrinsic["k6"]))

    p1_list.append(float(camera_intrinsic["p1"]))
    p2_list.append(float(camera_intrinsic["p2"]))

  fig = plt.figure()
  ax1 = fig.add_subplot(131)
  ax2 = fig.add_subplot(132)

  ax3 = fig.add_subplot(133)

  fx_std = np.std(fx_list)
  fy_std = np.std(fy_list)
  cx_std = np.std(cx_list)
  cy_std = np.std(cy_list)

  fx_mean = np.mean(fx_list)
  fy_mean = np.mean(fy_list)
  cx_mean = np.mean(cx_list)
  cy_mean = np.mean(cy_list)

  print(fx_mean, fy_mean, cx_mean, cy_mean)

  ax1.set_title("mean:({:.2f},{:.2f}) std:({:.2f},{:.2f})".format(fx_mean, fy_mean, fx_std, fy_std))
  ax2.set_title("mean:({:.2f},{:.2f}) std:({:.2f},{:.2f})".format(cx_mean, cy_mean, cx_std, cy_std))

  # radius1 = (fx_std ** 2 + fy_std ** 2) ** 0.5 * 3
  # circle1 = plt.Circle((fx_mean, fy_mean), radius1, color='r', alpha=0.3)
  # ax1.add_patch(circle1)
  ell1 = Ellipse(xy=(fx_mean, fy_mean), width=fx_std * factor * 2, height=fy_std * factor * 2, angle=0, facecolor='r',
                 alpha=0.3)
  ax1.add_patch(ell1)

  # radius2 = (cx_std ** 2 + cy_std ** 2) ** 0.5 * 3
  # circle2 = plt.Circle((cx_mean, cy_mean), radius2, color='r', alpha=0.3)
  # ax2.add_patch(circle2)
  ell2 = Ellipse(xy=(cx_mean, cy_mean), width=cx_std * factor * 2, height=cy_std * factor * 2, angle=0, facecolor='r',
                 alpha=0.3)
  ax2.add_patch(ell2)

  ax1.set_xlabel("fx")
  ax1.set_ylabel("fy")
  ax2.set_xlabel("cx")
  ax2.set_ylabel("cy")

  ax1.set_aspect(1)
  ax2.set_aspect(1)

  plt.xticks

  # ax3.set_xticks(ticks=[1, 2, 3, 4, 5, 6, 7, 8], labels=["k1", "k2", "k3", "k4", "k5", "k6", "p1", "p2", ])

  for i in range(len(fx_list)):
    ax1.scatter(fx_list[i], fy_list[i])
    ax2.scatter(cx_list[i], cy_list[i])

    ax3.scatter(1, k1_list[i])
    ax3.scatter(2, k2_list[i])
    ax3.scatter(3, k3_list[i])
    ax3.scatter(4, k4_list[i])
    ax3.scatter(5, k5_list[i])
    ax3.scatter(6, k6_list[i])

    ax3.scatter(7, p1_list[i])
    ax3.scatter(8, p2_list[i])

    if (fx_list[i] - fx_mean) ** 2 / (fx_std * factor) ** 2 + (fy_list[i] - fy_mean) ** 2 / (
      fy_std * factor) ** 2 > 1 or \
      (cx_list[i] - cx_mean) ** 2 / (cx_std * factor) ** 2 + (cy_list[i] - cy_mean) ** 2 / (
      cy_std * factor) ** 2 > 1:
      print(camera_intrinsic_list[i]["SN"], fx_list[i], fy_list[i], cx_list[i], cy_list[i])

  plt.show()


def mian(path, head):
  camera_intrinsic_list = load_all_camera_intrinsic(path, head)
  plot_camera_intrinsic_distribute(camera_intrinsic_list)


if __name__ == "__main__":
  # path = "/home/qcraft/Downloads/camera_intrinsic/H110/"
  path = "/home/qcraft/vehicles/v2/camera/inherent/"
  mian(path, "H30S1")
