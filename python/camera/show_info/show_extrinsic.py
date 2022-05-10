import sys
import os

param_path = "/home/qcraft/vehicles/v2/"
param_path_2 = "/home/qcraft/qcraft_data/vehicles/v2/"


def split_line(line):
    if "{" in line:
        return line.split()
    else:
        result = line.split(":", 1)
        result[0] = result[0].split()[0]
        if len(result) == 2:
            result[1] = result[1][1:-1]
        return result


def decode_lines(result, lines, index):
    while index < len(lines):
        # print(index, lines[index])
        words = split_line(lines[index])
        if words[0] == "}":
            break
        elif words[-1] == "{":
            key = words[0]
            value = dict()
            value, index = decode_lines(value, lines, index + 1)

        else:
            key = words[0]
            value = words[1]

        if key in result:
            if type(result[key]) != list:
                value_exit = result[key]
                result[key] = [value_exit, value]
            else:
                result[key].append(value)
        else:
            result[key] = value

        index += 1

    return result, index


def decodePbTxt(file):
    result = {}

    if not os.path.isfile(file):
        print(file, "do not exit!!")
    else:
        with open(file, "r") as f:
            lines = f.readlines()
            index = 0
            result, index = decode_lines(result, lines, index)
    return result


def decodeIntrinsic(sn, device):
    intrinsic_file = "{}{}/inherent/{}.pb.txt".format(param_path, device, sn)
    print("decode : {}".format(intrinsic_file))
    return decodePbTxt(intrinsic_file)


def decodeExtrinsic(sn, device):
    extrinsic_file = "{}{}/installation/{}.pb.txt".format(param_path, device, sn)
    print("decode : {}".format(extrinsic_file))
    return decodePbTxt(extrinsic_file)


def rad2degree(rad):
    return rad / 3.14159 * 180
    # return rad


def getDictValue(dict, key, default=""):
    try:
        if type(key) == list:
            result = dict
            for k in key:
                result = result[k]
            return result
        return dict[key]
    except:
        return default


def printCameraHheader():
    number = 320
    print("-" * number)
    print("{:20s}{:25s}{:20s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^20s}{:^20s}{:^20s}"
          "{:^15s}{:^15s}{:^15s}{:^10s}{:^10s}{:^10s}{:^10s}{:^20s}{:^20s}{:^12s}{:^12s}".
          format("camera_sn", "id", "model", "yaw", "pitch", "roll", "x", "y", "z", "ref_lidar_id", "obc",
                 "device_path",
                 "hardware_trigger", "rotate_90_ccw", "expected_fps", "roi_x", "roi_y", "roi_width", "roi_height",
                 "full_undistort_fov",
                 "warp_perspective", "warp_width", "warp_height"))
    print("-" * number)


def showCameraExtrinsic(camera_extrinsic, lidar_extrinsic, obc_camera_extrinsic):
    sn = camera_extrinsic["sn"]
    id = camera_extrinsic["camera_id"]
    model = camera_extrinsic["model"]
    if id == "CAM_INSIDE_DRIVER":
        yaw = pitch = roll = x = y = z = 0
        ref_lidar_id = ""
    else:
        # yaw = rad2degree(float(camera_extrinsic["extrinsics"]["yaw"]) + float(lidar_extrinsic["extrinsics"]["yaw"]))
        # pitch = rad2degree(float(camera_extrinsic["extrinsics"]["pitch"]) + float(lidar_extrinsic["extrinsics"]["pitch"]))
        # roll = rad2degree(float(camera_extrinsic["extrinsics"]["roll"]) + float(lidar_extrinsic["extrinsics"]["roll"]))
        # x = float(camera_extrinsic["extrinsics"]["x"]) + float(lidar_extrinsic["extrinsics"]["x"])
        # y = float(camera_extrinsic["extrinsics"]["y"]) + float(lidar_extrinsic["extrinsics"]["y"])
        # z = float(camera_extrinsic["extrinsics"]["z"]) + float(lidar_extrinsic["extrinsics"]["z"])

        yaw = rad2degree(float(getDictValue(camera_extrinsic, ["extrinsics", "yaw"], "0")))
        pitch = rad2degree(float(getDictValue(camera_extrinsic, ["extrinsics", "pitch"], "0")))
        roll = rad2degree(float(getDictValue(camera_extrinsic, ["extrinsics", "roll"], "0")))
        x = float(getDictValue(camera_extrinsic, ["extrinsics", "x"], "0"))
        y = float(getDictValue(camera_extrinsic, ["extrinsics", "y"], "0"))
        z = float(getDictValue(camera_extrinsic, ["extrinsics", "z"], "0"))

        ref_lidar_id = camera_extrinsic["ref_lidar_id"]

    device_path = camera_extrinsic["device_path"]
    hardware_trigger = getDictValue(camera_extrinsic, ["hardware_trigger"])
    rotate_90_ccw = getDictValue(camera_extrinsic, "rotate_90_ccw")
    expected_fps = getDictValue(camera_extrinsic, ["expected_fps", ])
    warp_perspective = getDictValue(camera_extrinsic, ["warp_perspective", ])
    warp_width = getDictValue(camera_extrinsic, ["warped_size", "width"])
    warp_height = getDictValue(camera_extrinsic, ["warped_size", "height"])
    full_undistort_fov = getDictValue(camera_extrinsic, ["full_undistort_fov", ])

    if "roi" in camera_extrinsic:
        roi_x = camera_extrinsic["roi"]["x"]
        roi_y = camera_extrinsic["roi"]["y"]
        roi_width = camera_extrinsic["roi"]["width"]
        roi_height = camera_extrinsic["roi"]["height"]
    else:
        roi_x = "0"
        roi_y = "0"
        roi_width = "-1"
        roi_height = "-1"

    if id in obc_camera_extrinsic:
        obc = obc_camera_extrinsic[id]
    else:
        obc = "not define"

    print("{:20s}{:25s}{:20s}{:^10.3f}{:^10.3f}{:^10.3f}{:^10.3f}{:^10.3f}{:^10.3f}{:^20s}{:^20s}{:^20s}"
          "{:^15s}{:^15s}{:^15s}{:^10s}{:^10s}{:^10s}{:^10s}{:^20s}{:^20s}{:^12s}{:^12s}".
          format(sn, id, model, yaw, pitch, roll, x, y, z, ref_lidar_id, obc, device_path,
                 hardware_trigger, rotate_90_ccw, expected_fps, roi_x, roi_y, roi_width, roi_height, full_undistort_fov,
                 warp_perspective, warp_width, warp_height))


def printCameraIntrinsicHheader():
    number = 300
    print("-" * number)
    print("{:20s}{:20s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}"
          "{:^15s}{:^15s}{:^15s}{:^10s}{:^10s}{:^30s}{:^20s}".
          format("camera_sn", "model", "k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6", "fx",
                 "fy", "cx", "cy", "rms", "max_fps", "mid_row_time_since_trigger", "rolling_elapse_time"))
    print("-" * number)


def showCameraIntrinsic(camera_intrinsic):
    camera_sn = camera_intrinsic["sn"]
    model = camera_intrinsic["model"]
    k1 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k1"], "0"))
    k2 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k2"], "0"))
    p1 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "p1"], "0"))
    p2 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "p2"], "0"))
    k3 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k3"], "0"))
    k4 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k4"], "0"))
    k5 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k5"], "0"))
    k6 = float(getDictValue(camera_intrinsic, ["intrinsics", "distort_coeffs", "k6"], "0"))

    fx = float(getDictValue(camera_intrinsic, ["intrinsics", "camera_matrix", "fx"], "0"))
    fy = float(getDictValue(camera_intrinsic, ["intrinsics", "camera_matrix", "fy"], "0"))
    cx = float(getDictValue(camera_intrinsic, ["intrinsics", "camera_matrix", "cx"], "0"))
    cy = float(getDictValue(camera_intrinsic, ["intrinsics", "camera_matrix", "cy"], "0"))

    rms = float(getDictValue(camera_intrinsic, ["intrinsics", "rms"], "0"))

    max_fps = getDictValue(camera_intrinsic, ["max_fps", ])
    mid_row_time_since_trigger = getDictValue(camera_intrinsic, ["mid_row_time_since_trigger", ])
    rolling_elapse_time = getDictValue(camera_intrinsic, ["rolling_elapse_time", ])

    print("{:20s}{:20s}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}{:^15.6f}"
          "{:^15.6f}{:^15.6f}{:^15.6f}{:^10.6}{:^10s}{:^30s}{:^20s}".
          format(camera_sn, model, k1, k2, p1, p2, k3, k4, k5, k6, fx,
                 fy, cx, cy, rms, max_fps, mid_row_time_since_trigger, rolling_elapse_time))


def printLidarHeader():
    number = 240
    print("-" * number)
    print("{:20s}{:25s}{:20s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}"
          "{:^10s}{:^20s}{:^20s}{:^10s}{:^10s}".format("lidar_sn", "id", "model", "yaw", "pitch", "roll", "x", "y", "z",
                                                       "enabled", "ip_address", "difop_port", "port",
                                                       "valid_point_min_range"))
    print("-" * number)


def showLidaraExtrinsic(lidar_extrinsic):
    sn = lidar_extrinsic["sn"]
    id = lidar_extrinsic["lidar_id"]
    model = lidar_extrinsic["model"]
    yaw = rad2degree(float(lidar_extrinsic["extrinsics"]["yaw"]))
    pitch = rad2degree(float(lidar_extrinsic["extrinsics"]["pitch"]))
    roll = rad2degree(float(lidar_extrinsic["extrinsics"]["roll"]))
    x = float(lidar_extrinsic["extrinsics"]["x"])
    y = float(lidar_extrinsic["extrinsics"]["y"])
    z = float(lidar_extrinsic["extrinsics"]["z"])

    enabled = lidar_extrinsic["enabled"]
    port = lidar_extrinsic["port"]
    valid_point_min_range = getDictValue(lidar_extrinsic, "valid_point_min_range")

    if "rs_lidar_params" in lidar_extrinsic:
        difop_port = lidar_extrinsic["rs_lidar_params"]["difop_port"]
        ip_address = getDictValue(lidar_extrinsic, ["rs_lidar_params", "ip_address"])

    else:
        ip_address = getDictValue(lidar_extrinsic, "ip")
        difop_port = lidar_extrinsic["port"]

    print("{:20s}{:25s}{:20s}{:^15.4f}{:^15.4f}{:^15.4f}{:^15.3f}{:^15.3f}{:^15.3f}"
          "{:^10s}{:^20s}{:^20s}{:^10s}{:^10s}".format(sn, id, model, yaw, pitch, roll, x, y, z,
                                                       enabled, ip_address, difop_port, port,
                                                       valid_point_min_range))


def printRadarHeader():
    number = 180
    print("-" * number)
    print("{:20s}{:25s}{:20s}{:^15s}{:^15s}{:^15s}{:^15s}{:^15s}"
          "{:^15s}{:^15s}{:^15s}".format("radar_sn", "id", "model ", "yaw", "pitch", "roll", "x", "y", "z",
                                         "socket_can_id",
                                         "sensor_id"))
    print("-" * number)


def showRadarExtrinsic(radar_extrinsic):
    sn = radar_extrinsic["sn"]
    id = radar_extrinsic["radar_id"]
    model = radar_extrinsic["model"]
    yaw = rad2degree(float(radar_extrinsic["extrinsics"]["yaw"]))
    pitch = rad2degree(float(radar_extrinsic["extrinsics"]["pitch"]))
    roll = rad2degree(float(radar_extrinsic["extrinsics"]["roll"]))
    x = float(radar_extrinsic["extrinsics"]["x"])
    y = float(radar_extrinsic["extrinsics"]["y"])
    z = float(radar_extrinsic["extrinsics"]["z"])
    socket_can_id = radar_extrinsic["socket_can_id"]
    sensor_id = radar_extrinsic["sensor_id"]

    print("{:20s}{:25s}{:20s}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}{:^15.2f}"
          "{:^15s}{:^15s}".format(sn, id, model, yaw, pitch, roll, x, y, z, socket_can_id, sensor_id))


def printGNSSIMUHeader():
    number = 270
    print("-" * number)
    print("{:20s}{:20s}{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:20s}{:10s}{:15s}{:10s}{:15s}"
          "{:10s}{:10s}{:10s}{:10s}{:10s}{:10s}{:20s}{:20s}"
          .format("gnss_imu_sn", "model", "ant1_x", "ant1_y", "ant1_z", "ant2_x", "ant2_y", "ant2_z", "ip_address",
                  "data_port", "data_src_port", "cmd_port", "cmd_src_port", "imu_x", "imu_y", "imu_z", "imu_yaw",
                  "imu_pitch", "imu_roll", "gnss_data_rate_hz", "imu_data_rate_hz"))
    print("-" * number)


def showGNSSIMUExtrinsic(gnss_imu_extrinisc):
    sn = gnss_imu_extrinisc["sn"]
    model = gnss_imu_extrinisc["model"]
    if "antenna_extrinsics" in gnss_imu_extrinisc:
        if type(gnss_imu_extrinisc["antenna_extrinsics"]) == list:
            ant1_x = gnss_imu_extrinisc["antenna_extrinsics"][0]["x"]
            ant1_y = gnss_imu_extrinisc["antenna_extrinsics"][0]["y"]
            ant1_z = gnss_imu_extrinisc["antenna_extrinsics"][0]["z"]

            ant2_x = gnss_imu_extrinisc["antenna_extrinsics"][1]["x"]
            ant2_y = gnss_imu_extrinisc["antenna_extrinsics"][1]["y"]
            ant2_z = gnss_imu_extrinisc["antenna_extrinsics"][1]["z"]
        else:
            ant1_x = gnss_imu_extrinisc["antenna_extrinsics"]["x"]
            ant1_y = gnss_imu_extrinisc["antenna_extrinsics"]["y"]
            ant1_z = gnss_imu_extrinisc["antenna_extrinsics"]["z"]

            ant2_x = "-1"
            ant2_y = "-1"
            ant2_z = "-1"


    else:
        ant1_x = "-1"
        ant1_y = "-1"
        ant1_z = "-1"

        ant2_x = "-1"
        ant2_y = "-1"
        ant2_z = "-1"
    if "udp" in gnss_imu_extrinisc["data_port"]:
        ip_address = gnss_imu_extrinisc["data_port"]["udp"]["address"]
        data_port = gnss_imu_extrinisc["data_port"]["udp"]["port"]
    elif "tcp " in gnss_imu_extrinisc["data_port"]:
        ip_address = gnss_imu_extrinisc["data_port"]["tcp"]["address"]
        data_port = gnss_imu_extrinisc["data_port"]["tcp"]["port"]
    else:
        ip_address = ""
        data_port = ""
    data_src_port = getDictValue(gnss_imu_extrinisc, ["data_port", "udp", "src_port"])
    cmd_port = getDictValue(gnss_imu_extrinisc, ["command_port", "udp", "port"])
    cmd_src_port = getDictValue(gnss_imu_extrinisc, ["command_port", "udp", "src_port"])

    imu_x = float(gnss_imu_extrinisc["imu_extrinsics"]["x"])
    imu_y = float(gnss_imu_extrinisc["imu_extrinsics"]["y"])
    imu_z = float(gnss_imu_extrinisc["imu_extrinsics"]["z"])
    imu_yaw = rad2degree(float(gnss_imu_extrinisc["imu_extrinsics"]["yaw"]))
    imu_pitch = rad2degree(float(gnss_imu_extrinisc["imu_extrinsics"]["pitch"]))
    imu_roll = rad2degree(float(gnss_imu_extrinisc["imu_extrinsics"]["roll"]))

    gnss_data_rate_hz = gnss_imu_extrinisc["gnss_data_rate_hz"]
    imu_data_rate_hz = gnss_imu_extrinisc["imu_data_rate_hz"]

    print("{:20s}{:20s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^10s}{:^20s}{:^10s}{:^15s}{:^10s}{:^15s}"
          "{:^10.4f}{:^10.4f}{:^10.4f}{:^10.2f}{:^10.2f}{:^10.2f}{:^20s}{:^20s}"
          .format(sn, model, ant1_x, ant1_y, ant1_z, ant2_x, ant2_y, ant2_z, ip_address, data_port, data_src_port,
                  cmd_port, cmd_src_port, imu_x, imu_y, imu_z, imu_yaw, imu_pitch, imu_roll, gnss_data_rate_hz,
                  imu_data_rate_hz))


def printOMCHeader():
    number = 230
    print("-" * number)
    print("{:20s}{:20s}{:^15s}{:^15s}{:^25s}{:^15s}{:^10s}{:^10s}"
          "{:^20s}{:^20s}{:^20s}{:^40s}".format("omc_sn", "model ", "can_type", "can_bit_rate", "can_socketcan_ifname",
                                                "can_enable", "gpu_id", "device_id", "gpu_type", "node_ip", "node_port",
                                                "lite_launch_config_filename"))
    print("-" * number)


def showOMCSn(omc_extrinsic, i):
    if i == 0:
        sn = omc_extrinsic["sn"]
        model = omc_extrinsic["model"]
        print("{:20}{:20}".format(sn, model), end="")
    else:
        print("{:20}{:20}".format("", ""), end="")


def showCANExtrinsc(omc_extrinsic, i):
    if i == 0:
        can_type = omc_extrinsic["can_params"]["type"]
        can_bit_rate = omc_extrinsic["can_params"]["bit_rate"]
        can_socketcan_ifname = omc_extrinsic["can_params"]["socketcan_ifname"]
        can_enable = omc_extrinsic["can_params"]["enable"]

        print("{:^15}{:^15}{:^25}{:^15}"
              .format(can_type, can_bit_rate, can_socketcan_ifname, can_enable), end="")
    else:
        print("{:^15}{:^15}{:^25}{:^15}".format("", "", "", ""), end="")


def showGPUExtrinsc(omc_extrinsic, i):
    # print(omc_extrinsic)
    if type(omc_extrinsic["omc_config"]["gpu_id_to_device_id"]) == list:
        if len(omc_extrinsic["omc_config"]["gpu_id_to_device_id"]) > i:
            gpu = omc_extrinsic["omc_config"]["gpu_id_to_device_id"][i]
            print("{:^10}{:^10}{:^20}".format(gpu["gpu_id"], gpu["device_id"], gpu["gpu_type"]), end="")
            return
    elif i == 0:
        gpu = omc_extrinsic["omc_config"]["gpu_id_to_device_id"]
        print("{:^10}{:^10}{:^20}".format(gpu["gpu_id"], gpu["device_id"], gpu["gpu_type"]), end="")
        return

    print("{:^10}{:^10}{:^20}".format("", "", "", ""), end="")


def showNodeExtrinsc(omc_extrinsic, i):
    if type(omc_extrinsic["nodes_run_config"]["nodes"]) == list:
        if len(omc_extrinsic["nodes_run_config"]["nodes"]) > i:
            node = omc_extrinsic["nodes_run_config"]["nodes"][i]
            print("{:^20}{:^20}{:^40}"
                  .format(node["node_ip"], node["node_port"], node["lite_launch_config_filename"]))
            return
    elif i == 0:
        node = omc_extrinsic["nodes_run_config"]["nodes"]
        print("{:^20}{:^20}{:^40}"
              .format(node["node_ip"], node["node_port"], node["lite_launch_config_filename"]))
        return

    print("{:^20}{:^20}{:^40}".format("", "", ""))


def showOMCExtrinsic(omc_extrinsic):
    num = 1
    if type(omc_extrinsic["nodes_run_config"]["nodes"]) == list:
        num = len(omc_extrinsic["nodes_run_config"]["nodes"])
    if type(omc_extrinsic["omc_config"]["gpu_id_to_device_id"]) == list:
        num = len(omc_extrinsic["omc_config"]["gpu_id_to_device_id"]) if len(
            omc_extrinsic["omc_config"]["gpu_id_to_device_id"]) > num else num

    for i in range(num):
        showOMCSn(omc_extrinsic, i)
        showCANExtrinsc(omc_extrinsic, i)
        showGPUExtrinsc(omc_extrinsic, i)
        showNodeExtrinsc(omc_extrinsic, i)


def obcCameraExtrinsic(obc_extrinsic_list):
    result = {}
    for obc_extrinsic in obc_extrinsic_list:
        if "camera_id" not in obc_extrinsic["obc_config"]["node_name_to_camera_id"]:
            continue
        for camera_id in obc_extrinsic["obc_config"]["node_name_to_camera_id"]["camera_id"]:
            result[camera_id] = obc_extrinsic["obc_config"]["node_name_to_camera_id"]["node_name"]
    return result


def printOBCHheader():
    number = 100
    print("-" * number)
    print("{:20s}{:25s}{:20s}{:^20s}{:^20s}".
          format("obc_sn", "node_name", "model", "ip_address", "node_ip"))
    print("-" * number)


def showOBCExtrinsic(obc_extrinsic):
    sn = obc_extrinsic["sn"]
    model = obc_extrinsic["model"]
    node_name = obc_extrinsic["obc_config"]["node_name_to_camera_id"]["node_name"]
    ip_address = getDictValue(obc_extrinsic["obc_config"], "ip_address")
    node_ip = getDictValue(obc_extrinsic, ["nodes_run_config", "nodes", "node_ip"])
    print("{:20s}{:25s}{:20s}{:^20s}{:^20s}".format(sn, node_name, model, ip_address, node_ip))


def printVehicleHheader():
    number = 100
    print("-" * number)
    print("{:20s}{:30s}{:^25s}{:^25s}".
          format("vehicle_sn", "model", "steering_direction", "wheel_rolling_radius"))
    print("-" * number)


def showVehicleIntrinsic(vehicle_extrinsic):
    sn = vehicle_extrinsic["sn"]
    model = vehicle_extrinsic["model"]
    steering_direction = getDictValue(vehicle_extrinsic, "steering_direction")
    wheel_rolling_radius = vehicle_extrinsic["wheel_rolling_radius"]
    print("{:20s}{:30s}{:^25s}{:^25s}".
          format(sn, model, steering_direction, wheel_rolling_radius))


def printV2XHeader():
    number = 110
    print("-" * number)
    print("{:20s}{:20s}{:^20s}{:^10s}{:^10s}{:^10s}{:^20s}".
          format("v2x_sn", "model", "ip", "port", "is_send", "enabled", "sub_type"))
    print("-" * number)


def showV2XExtrinsic(v2x_extrinsic):
    first_line = True
    if "obu_entity" not in v2x_extrinsic:
        return

    for obu in v2x_extrinsic["obu_entity"]:
        if first_line:
            sn = v2x_extrinsic["sn"]
            model = v2x_extrinsic["model"]
            print("{:20s}{:20s}".format(sn, model), end="")
            first_line = False
        else:
            print("{:20s}{:20s}".format("", ""), end="")

        ip = obu["ip"]
        port = obu["port"]
        is_send = obu["is_send"]
        enabled = obu["enabled"]
        sub_type = getDictValue(obu, "sub_type")
        print("{:^20s}{:^10s}{:^10s}{:^10s}{:^20s}".
              format(ip, port, is_send, enabled, sub_type))


def showVechicleExtrinsic(car_number):
    param_file = param_path + car_number + ".pb.txt"
    if not os.path.isfile(param_file):
        print(param_file, "do not exit!!")
        return

    with open(param_file, "r") as f:
        lines = f.readlines()

        camera_extrinsic_list = []
        camera_intrinsic_list = []
        obc_extrinsic_list = []
        radar_extrinsic_list = []
        lidar_extrinsic_dict = {}
        gnss_imu_extrinsic = {}
        v2x_extrinsic = {}
        index = 0
        while index + 3 < len(lines):
            if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_CAMERA\n":
                camera_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                camera_extrinsic = decodeExtrinsic(camera_sn, "camera")
                camera_extrinsic["sn"] = camera_sn
                camera_extrinsic["model"] = model
                camera_extrinsic_list.append(camera_extrinsic)

                camera_intrinsic = decodeIntrinsic(camera_sn, "camera")
                camera_intrinsic["sn"] = camera_sn
                camera_intrinsic["model"] = model
                camera_intrinsic_list.append(camera_intrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_LIDAR\n":
                lidar_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                lidar_extrinsic = decodeExtrinsic(lidar_sn, "lidar")
                lidar_extrinsic["sn"] = lidar_sn
                lidar_extrinsic["model"] = model
                lidar_extrinsic_dict[lidar_extrinsic["lidar_id"]] = lidar_extrinsic

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_GNSS\n":
                gnss_imu_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                gnss_imu_extrinsic = decodeExtrinsic(gnss_imu_sn, "gnss")
                gnss_imu_extrinsic["sn"] = gnss_imu_sn
                gnss_imu_extrinsic["model"] = model

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_RADAR\n":
                radar_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                radar_extrinsic = decodeExtrinsic(radar_sn, "radar")
                radar_extrinsic["sn"] = radar_sn
                radar_extrinsic["model"] = model
                radar_extrinsic_list.append(radar_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OMC\n":
                omc_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                omc_extrinsic = decodeExtrinsic(omc_sn, "omc")
                omc_extrinsic["sn"] = omc_sn
                omc_extrinsic["model"] = model

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OBC\n":
                obc_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                obc_extrinsic = decodeExtrinsic(obc_sn, "obc")
                obc_extrinsic["sn"] = obc_sn
                obc_extrinsic["model"] = model
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

            index += 1

        obc_camera_extrinsic = obcCameraExtrinsic(obc_extrinsic_list)

        print("vehicle_sn : ", car_number)
        print("extrinsic")

        if gnss_imu_extrinsic:
            printGNSSIMUHeader()
            showGNSSIMUExtrinsic(gnss_imu_extrinsic)

        printCameraHheader()
        for camera_extrinsic in camera_extrinsic_list:
            if camera_extrinsic["ref_lidar_id"] not in lidar_extrinsic_dict:
                for i in range(5):
                    print("!" * 100)
                print(camera_extrinsic["sn"],
                      ", can not find {} in lidar list!!!".format(camera_extrinsic["ref_lidar_id"]))
                for i in range(5):
                    print("!" * 100)
            else:
                showCameraExtrinsic(camera_extrinsic, lidar_extrinsic_dict[camera_extrinsic["ref_lidar_id"]],
                                    obc_camera_extrinsic)

        printLidarHeader()
        for id in lidar_extrinsic_dict:
            showLidaraExtrinsic(lidar_extrinsic_dict[id])

        printRadarHeader()
        for radar_extrinsic in radar_extrinsic_list:
            showRadarExtrinsic(radar_extrinsic)

        printOMCHeader()
        showOMCExtrinsic(omc_extrinsic)

        printOBCHheader()
        for obc_extrinsic in obc_extrinsic_list:
            showOBCExtrinsic(obc_extrinsic)

        printVehicleHheader()
        showVehicleIntrinsic(vehicle_extrinsic)

        printV2XHeader()
        showV2XExtrinsic(v2x_extrinsic)

        print("\n\nintrinsic")
        printCameraIntrinsicHheader()
        for camera_intrinsic in camera_intrinsic_list:
            showCameraIntrinsic(camera_intrinsic)


def judegeVehiclesExist():
    global param_path, param_path_2
    if not os.path.isdir(param_path):
        print("{} do not exist!!!".format(param_path))
    else:
        return

    if not os.path.isdir(param_path_2):
        print("{} do not exist!!!".format(param_path_2))
    elif os.path.isdir(param_path_2):
        param_path = param_path_2


if __name__ == "__main__":
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
