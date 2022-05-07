from show_extrinsic import *
import csv

extrinsics = {
  "x": "",
  "y": "",
  "z": "",
  "yaw": "",
  "pitch": "",
  "roll": "",
  "calibration_engineer": "",
  "calibration_time": "",
  "calibration_run": "",
}

camera_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "camera_id": "",
  "extrinsics": extrinsics,
  "ref_lidar_id": "",
  "device_path": "",
  "hardware_trigger": "",
  "rotate_90_ccw": "",
  "expected_fps": "",
  "roi": {
    "x": "",
    "y": "",
    "width": "",
    "height": "",
  },
  "full_undistort_fov": "",
  "warp_perspective": "",
  "warped_size": {
    "width": "",
    "height": ""
  },
  "camera_to_vehicle_extrinsics": {
    "x": "",
    "y": "",
    "z": "",
    "yaw": "",
    "pitch": "",
    "roll": "",
  },
}

camera_intrinsic_tmp = {
  "intrinsics": {
    "calibration_time": "",
    "distort_coeffs": {
      "k1": "",
      "k2": "",
      "p1": "",
      "p2": "",
      "k3": "",
      "k4": "",
      "k5": "",
      "k6": "",
    },
    "camera_matrix": {
      "fx": "",
      "fy": "",
      "cx": "",
      "p2": "",
      "cy": "",
      "k4": "",
      "k5": "",
      "k6": "",
    },
    "serial_no": "",
    "calibration_engineer": "",
    "rms": ""
  },
  "max_fps": "",
  "mid_row_time_since_trigger": "",
  "rolling_elapse_time": "",
  "camera_vendor": "",
  "serial_no": "",
}

lidar_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "lidar_id": "",
  "extrinsics": extrinsics,
  "enabled": "",
  "valid_point_min_range": "",
  "rs_lidar_params": {
    "difop_port": "",
    "ip_address": ""
  },
  "hesai_lidar_params": {
    "intensity_nonlinear_mapping": ""
  },
  "ouster_lidar_params": {
    "ip_address": ""
  },
  "port": "",
  "ip": "",
  # "ignored_scan_azimuth_ranges_in_degree": {
  #   "first": "",
  #   "last": "",
  #   "first_beam": "",
  #   "last_beam": "",
  # },
}

radar_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "radar_id": "",
  "extrinsics": extrinsics,
  "socket_can_id": "",
  "sensor_id": "",
}

gnss_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "antenna_extrinsics": {
    "id": "",
    "x": "",
    "y": "",
    "z": "",
  },
  "format": "",
  "data_port": {
    "udp":
      {
        "address": "",
        "port": "",
        "src_port": "",
      },
    "tcp": {
      "address": "",
      "port": "",
    },
  },
  "command_port": {
    "udp":
      {
        "address": "",
        "port": "",
        "src_port": "",
      },
  },
  "imu_extrinsics": extrinsics,
  "gnss_data_rate_hz": "",
  "imu_data_rate_hz": "",
}

omc_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "can_params": {
    "type": "",
    "bit_rate": "",
    "socketcan_ifname": "",
    "enable": "",
  },
  "omc_config": {
    "gpu_id_to_device_id": {
      "gpu_id": "",
      "device_id": "",
      "gpu_type": "",
    },
    "ip_address": "",
    "main_display": "",
  },
  "nodes_run_config": {
    "nodes_scheme_name": "",
    "nodes":
      {
        "node_name": "",
        "name_space": "",
        "node_ip": "",
        "node_port": "",
        "lite_launch_config_filename": "",
        "timeout": "",
      },
  },
}

obc_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "nodes_run_config": {
    "nodes": {
      "node_name": "",
      "name_space": "",
      "node_ip": "",
      "node_port": "",
      "lite_launch_config_filename": "",
      "timeout": "",
    },
    "nodes_scheme_name": "",
  },
  "obc_config": {
    "ip_address": "",
    "main_display": "",
    "obc_type": "",
    "node_name_to_camera_id": {
      "node_name": "",
      "camera_id": "",
    },
  },
}

v2x_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "obu_entity": {
    "ip": "",
    "port": "",
    "is_send": "",
    "enabled": "",
    "sub_type": "",
  },
}

occ_extrinsic_tmp = {
  "sn": "",
  "model": "",
  "is_valid": "",
  "occ_device": {
    "addr_ip": "",
  },
  "brake_params": {
    "brake_value": "",
  },
}


def dict_to_lsit(input_dict):
  key_list = []
  value_list = []

  for key in input_dict:
    if type(input_dict[key]) == dict:
      key_list.append(key)
      value_list.append("{")
      tmp = dict_to_lsit(input_dict[key])
      key_list.extend(tmp[0])
      value_list.extend(tmp[1])
      key_list.append(key)
      value_list.append("}")
    elif type(input_dict[key]) == list:
      if type(input_dict[key][0]) == dict:
        for sub_dict in input_dict[key]:
          key_list.append(key)
          value_list.append("{")
          tmp = dict_to_lsit(sub_dict)
          key_list.extend(tmp[0])
          value_list.extend(tmp[1])
          key_list.append(key)
          value_list.append("}")
      elif type(input_dict[key][0]) == str:
        for item in input_dict[key]:
          key_list.append(key)
          value_list.append(item)
    elif type(input_dict[key]) == str:
      key_list.append(key)
      value_list.append(input_dict[key])

    # print("\n", key_list, "\n", value_list)

  # print("\n", key_list, "\n", value_list)
  return key_list, value_list


def write_dict_to_csv(input_dict, fw):
  if type(input_dict) == list:
    key_list, _ = dict_to_lsit(input_dict[0])
    fw.writerow(key_list)
    for sub_dict in input_dict:
      _, value_list = dict_to_lsit(sub_dict)
      fw.writerow(value_list)
  else:
    key_list, value_list = dict_to_lsit(input_dict)
    fw.writerow(key_list)
    fw.writerow(value_list)
  fw.writerow([])


def merge_dict(dict_1, dict_2):
  result = dict_1.copy()
  for key in result:
    if key in dict_2:
      if type(result[key]) == dict and type(dict_2[key]) == dict:
        result[key] = merge_dict(result[key], dict_2[key])
      elif type(dict_2[key]) == list:
        if type(dict_2[key][0]) == dict:
          tmp = []
          for sub_dict in dict_2[key]:
            tmp.append(merge_dict(result[key], sub_dict))
          result[key] = tmp
        else:
          result[key] = ["", "", "", "", "", "", "", "", ]
          for i in range(len(dict_2[key])):
            result[key][i] = dict_2[key][i]
      else:
        result[key] = dict_2[key]
  return result


def showVechicleExtrinsic(car_number):
  param_file = param_path + car_number + ".pb.txt"
  if not os.path.isfile(param_file):
    print(param_file, "do not exit!!")
    return

  with open(param_file, "r") as f:
    lines = f.readlines()

    camera_extrinsic_list = []
    radar_extrinsic_list = []
    lidar_extrinsic_list = []
    gnss_imu_extrinsic = {}
    obc_extrinsic_list = []
    v2x_extrinsic = {}
    omc_extrinsic = {}
    vehicle_intrinsic = {}

    index = 0
    while index + 3 < len(lines):
      if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_CAMERA\n":
        camera_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        camera_extrinsic = decodeExtrinsic(camera_sn, "camera")
        camera_extrinsic["sn"] = camera_sn
        camera_extrinsic["model"] = model
        camera_extrinsic = merge_dict(camera_extrinsic_tmp, camera_extrinsic)
        camera_extrinsic_list.append(camera_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_LIDAR\n":
        lidar_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        lidar_extrinsic = decodeExtrinsic(lidar_sn, "lidar")
        lidar_extrinsic["sn"] = lidar_sn
        lidar_extrinsic["model"] = model
        lidar_extrinsic = merge_dict(lidar_extrinsic_tmp, lidar_extrinsic)
        lidar_extrinsic_list.append(lidar_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_GNSS\n":
        gnss_imu_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        gnss_imu_extrinsic = decodeExtrinsic(gnss_imu_sn, "gnss")
        gnss_imu_extrinsic["sn"] = gnss_imu_sn
        gnss_imu_extrinsic["model"] = model
        gnss_imu_extrinsic = merge_dict(gnss_extrinsic_tmp, gnss_imu_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_RADAR\n":
        radar_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        radar_extrinsic = decodeExtrinsic(radar_sn, "radar")
        radar_extrinsic["sn"] = radar_sn
        radar_extrinsic["model"] = model
        radar_extrinsic = merge_dict(radar_extrinsic_tmp, radar_extrinsic)
        radar_extrinsic_list.append(radar_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OMC\n":
        omc_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        omc_extrinsic = decodeExtrinsic(omc_sn, "omc")
        omc_extrinsic["sn"] = omc_sn
        omc_extrinsic["model"] = model
        omc_extrinsic = merge_dict(omc_extrinsic_tmp, omc_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OBC\n":
        obc_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        obc_extrinsic = decodeExtrinsic(obc_sn, "obc")
        obc_extrinsic["sn"] = obc_sn
        obc_extrinsic["model"] = model
        obc_extrinsic = merge_dict(obc_extrinsic_tmp, obc_extrinsic)
        obc_extrinsic_list.append(obc_extrinsic)

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_VEHICLE\n":
        vehicle_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        vehicle_extrinsic = decodeIntrinsic(vehicle_sn, "vehicle")
        vehicle_extrinsic["sn"] = vehicle_sn
        vehicle_extrinsic["model"] = model

      elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_V2X\n":
        v2x_sn = lines[index + 2].split()[1][1:-1]
        model = lines[index + 1].split()[1][1:-1]
        v2x_extrinsic = decodeExtrinsic(v2x_sn, "v2x")
        v2x_extrinsic["sn"] = v2x_sn
        v2x_extrinsic["model"] = model
        v2x_extrinsic = merge_dict(v2x_extrinsic_tmp, v2x_extrinsic)
      index += 1

  extrinsics_list = [camera_extrinsic_list,
                     radar_extrinsic_list,
                     lidar_extrinsic_list,
                     gnss_imu_extrinsic,
                     omc_extrinsic,
                     obc_extrinsic_list,
                     v2x_extrinsic,
                     vehicle_intrinsic]

  with open(car_number + ".csv", 'w', newline='', encoding='UTF-8-SIG') as f:
    fw = csv.writer(f)
    for extrinsics in extrinsics_list:
      write_dict_to_csv(extrinsics, fw)


if __name__ == "__main__":
  # camera_sn = "H60S-E01250557"
  # camera_intrinsic = decodeExtrinsic(camera_sn, "camera")
  # print(camera_intrinsic)
  # camera_intrinsic = merge_dict(camera_extrinsic_tmp, camera_intrinsic)
  # print(camera_intrinsic)
  #
  # with open('temp.csv', 'w', newline='', encoding='UTF-8-SIG') as f:
  #   fw = csv.writer(f)
  #
  #   write_dict_to_csv(camera_intrinsic, fw)

  if len(sys.argv) < 2:
    print("please input vehicle number Qxxxx ")
    sys.exit()

  if len(sys.argv) == 3:
    print("set param_path as {}".format(sys.argv[2]))
    param_path = sys.argv[2]

  judegeVehiclesExist()

  for i in range(1, len(sys.argv)):
    sn = sys.argv[i]
    if sn[0] == "Q":
      showVechicleExtrinsic(sn)
