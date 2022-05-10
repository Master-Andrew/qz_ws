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
    "type": "",
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
    "type": "",
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
    "ignored_scan_azimuth_ranges_in_degree": {
        "first": "",
        "last": "",
        "first_beam": "",
        "last_beam": "",
    },
}

radar_extrinsic_tmp = {
    "sn": "",
    "model": "",
    "type": "",
    "radar_id": "",
    "extrinsics": extrinsics,
    "socket_can_id": "",
    "sensor_id": "",
}

gnss_extrinsic_tmp = {
    "sn": "",
    "model": "",
    "type": "",
    "antenna_extrinsics": {
        "id": "",
        "x": "",
        "y": "",
        "z": "",
        "calibration_time": "",
        "calibration_engineer": "",
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
    "type": "",
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
    "type": "",
    "obc_config": {
        "ip_address": "",
        "node_name_to_camera_id": {
            "node_name": "",
            "camera_id": "",
        },

        "main_display": "",
        "obc_type": "",
    },
    "nodes_run_config": {
        "nodes_scheme_name": "",
        "nodes": {
            "node_name": "",
            "name_space": "",
            "node_ip": "",
            "node_port": "",
            "lite_launch_config_filename": "",
            "timeout": "",
        },
    },
}

v2x_extrinsic_tmp = {
    "sn": "",
    "model": "",
    "type": "",
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
    "type": "",
    "is_valid": "",
    "occ_device": {
        "addr_ip": "",
    },
    "brake_params": {
        "brake_value": "",
    },
}

vehicle_intrinsic_tmp = {
    "sn": "",
    "model": "",
    "type": "",
    "steering_direction": "",
    "wheel_rolling_radius": "",
    "serial_no": "",
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

    return key_list, value_list


def get_dict_len(input_dict):
    result = 0
    for key in input_dict:
        if type(input_dict[key]) == dict:
            result += get_dict_len(input_dict[key])
        elif type(input_dict[key]) == list:
            if type(input_dict[key][0]) == dict:
                for sub_dict in input_dict[key]:
                    result += get_dict_len(sub_dict)
            else:
                result += len(input_dict[key])
        else:
            result += 1
    return result


def write_dict_to_csv(input_dict, fw):
    if type(input_dict) == list:
        dict_len = 0
        key_list, _ = dict_to_lsit(input_dict[0])
        for sub_dict in input_dict:
            sub_dict_len = get_dict_len(sub_dict)
            if sub_dict_len > dict_len:
                dict_len = sub_dict_len
                key_list, _ = dict_to_lsit(sub_dict)

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
                    # print(dict_2[key])
                    tmp = []
                    for sub_dict in dict_2[key]:
                        tmp.append(merge_dict(result[key], sub_dict))
                    result[key] = tmp
                    # print("output : ",result[key])
                else:
                    result[key] = ["", "", "", "", "", "", "", "", ]
                    for i in range(len(dict_2[key])):
                        result[key][i] = dict_2[key][i]
            else:
                result[key] = dict_2[key]
    # print("*"*100)
    # print(result)
    # print("*"*100)
    return result


def writeVechicleExtrinsic(car_number):
    param_file = param_path + car_number + ".pb.txt"
    if not os.path.isfile(param_file):
        print(param_file, "do not exit!!")
        return

    with open(param_file, "r") as f:
        lines = f.readlines()

        camera_extrinsic_list = []
        radar_extrinsic_list = []
        lidar_extrinsic_list = []
        gnss_imu_extrinsic_list = []
        obc_extrinsic_list = []
        v2x_extrinsic_list = []
        omc_extrinsic_list = []
        vehicle_intrinsic_list = []

        index = 0
        while index + 3 < len(lines):
            if lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_CAMERA\n":
                camera_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                camera_extrinsic = decodeExtrinsic(camera_sn, "camera")
                camera_extrinsic["sn"] = camera_sn
                camera_extrinsic["model"] = model
                camera_extrinsic["type"] = type
                camera_extrinsic = merge_dict(camera_extrinsic_tmp, camera_extrinsic)
                camera_extrinsic_list.append(camera_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_LIDAR\n":
                lidar_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                lidar_extrinsic = decodeExtrinsic(lidar_sn, "lidar")
                lidar_extrinsic["sn"] = lidar_sn
                lidar_extrinsic["model"] = model
                lidar_extrinsic["type"] = type
                lidar_extrinsic = merge_dict(lidar_extrinsic_tmp, lidar_extrinsic)
                lidar_extrinsic_list.append(lidar_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_GNSS\n":
                gnss_imu_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                gnss_extrinsic = decodeExtrinsic(gnss_imu_sn, "gnss")
                gnss_extrinsic["sn"] = gnss_imu_sn
                gnss_extrinsic["model"] = model
                gnss_extrinsic["type"] = type
                gnss_extrinsic = merge_dict(gnss_extrinsic_tmp, gnss_extrinsic)
                gnss_imu_extrinsic_list.append(gnss_extrinsic)


            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_RADAR\n":
                radar_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                radar_extrinsic = decodeExtrinsic(radar_sn, "radar")
                radar_extrinsic["sn"] = radar_sn
                radar_extrinsic["model"] = model
                radar_extrinsic["type"] = type
                radar_extrinsic = merge_dict(radar_extrinsic_tmp, radar_extrinsic)
                radar_extrinsic_list.append(radar_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OMC\n":
                omc_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                omc_extrinsic = decodeExtrinsic(omc_sn, "omc")
                omc_extrinsic["sn"] = omc_sn
                omc_extrinsic["model"] = model
                omc_extrinsic["type"] = type
                omc_extrinsic = merge_dict(omc_extrinsic_tmp, omc_extrinsic)
                omc_extrinsic_list.append(omc_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_OBC\n":
                obc_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                obc_extrinsic = decodeExtrinsic(obc_sn, "obc")
                obc_extrinsic["sn"] = obc_sn
                obc_extrinsic["model"] = model
                obc_extrinsic["type"] = type
                obc_extrinsic = merge_dict(obc_extrinsic_tmp, obc_extrinsic)
                obc_extrinsic_list.append(obc_extrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_VEHICLE\n":
                vehicle_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                vehicle_intrinsic = decodeIntrinsic(vehicle_sn, "vehicle")
                vehicle_intrinsic["sn"] = vehicle_sn
                vehicle_intrinsic["model"] = model
                vehicle_intrinsic["type"] = type
                vehicle_intrinsic = merge_dict(vehicle_intrinsic_tmp, vehicle_intrinsic)
                vehicle_intrinsic_list.append(vehicle_intrinsic)

            elif lines[index] == "hardwares {\n" and lines[index + 3] == "  type: HARDWARE_V2X\n":
                v2x_sn = lines[index + 2].split()[1][1:-1]
                model = lines[index + 1].split()[1][1:-1]
                type = lines[index + 3].split()[1]
                v2x_extrinsic = decodeExtrinsic(v2x_sn, "v2x")
                v2x_extrinsic["sn"] = v2x_sn
                v2x_extrinsic["model"] = model
                v2x_extrinsic["type"] = type
                v2x_extrinsic = merge_dict(v2x_extrinsic_tmp, v2x_extrinsic)
                v2x_extrinsic_list.append(v2x_extrinsic)
            index += 1

    extrinsics_list = [camera_extrinsic_list,
                       gnss_imu_extrinsic_list,
                       lidar_extrinsic_list,
                       obc_extrinsic_list,
                       omc_extrinsic_list,
                       radar_extrinsic_list,
                       v2x_extrinsic_list,
                       vehicle_intrinsic_list]

    csv_file = os.getcwd() + "/" + car_number + ".csv"
    with open(csv_file, 'w', newline='', encoding='UTF-8-SIG') as f:
        print("write csv to : {}".format(csv_file))
        fw = csv.writer(f)
        for extrinsics in extrinsics_list:
            write_dict_to_csv(extrinsics, fw)


def list_to_dict(header, row, dict_tmp, index):
    result = {}
    # print(row, header)
    while index < len(row):
        # print(index, len(header))
        key = header[index]
        if row[index] == "{":
            value, index = list_to_dict(header, row, dict_tmp, index + 1)
        elif row[index] == "}":
            return result, index
        else:
            value = row[index]

        if key in result:
            if type(result[key]) != list:
                value_exit = result[key]
                result[key] = []
                result[key].append(value_exit)
            result[key].append(value)
        else:
            result[key] = value

        index += 1
    return result, index


def list_is_empty(input):
    for i in input:
        if i != "":
            return False
    return True


def csv_to_dict(csv_file):
    camera_extrinsic_list = []
    radar_extrinsic_list = []
    lidar_extrinsic_list = []
    gnss_imu_extrinsic_list = []
    obc_extrinsic_list = []
    v2x_extrinsic_list = []
    omc_extrinsic_list = []
    vehicle_intrinsic_list = []

    extrinsics_list = [camera_extrinsic_list,
                       gnss_imu_extrinsic_list,
                       lidar_extrinsic_list,
                       obc_extrinsic_list,
                       omc_extrinsic_list,
                       radar_extrinsic_list,
                       v2x_extrinsic_list,
                       vehicle_intrinsic_list]
    extrinsics_tmp_list = [camera_extrinsic_tmp,
                           gnss_extrinsic_tmp,
                           lidar_extrinsic_tmp,
                           obc_extrinsic_tmp,
                           omc_extrinsic_tmp,
                           radar_extrinsic_tmp,
                           v2x_extrinsic_tmp,
                           vehicle_intrinsic_tmp]

    print("open : {}".format(os.getcwd() + "/" + csv_file))
    with open(csv_file, 'r', newline='', encoding='UTF-8-SIG') as f:
        fr = csv.reader(f)
        index = -1
        for row in fr:
            if list_is_empty(row):
                continue
            elif row[0] == "sn":
                header = row
                index += 1
            else:
                extrinsics_list[index].append(list_to_dict(header, row, extrinsics_tmp_list[index], 0)[0])
                print("row : ", row, )
    return extrinsics_list


def dict_is_empty(input_dict):
    result = True
    for key in input_dict:
        if input_dict[key] != "":
            result = False

    return result


def dict_to_pb_txt(input_dict, i, f):
    # print(input_dict, i)
    for key in input_dict:
        # print(key,type(input_dict[key]))
        if key == "sn" or key == "model" or (key == "type" and i == 0):
            continue
        elif type(input_dict[key]) == dict:
            if dict_is_empty(input_dict[key]):
                dict_to_pb_txt(input_dict[key], i + 1, f)
            else:
                line = " " * i * 2 + key + " {" + "\n"
                f.write(line)
                dict_to_pb_txt(input_dict[key], i + 1, f)
                line = " " * i * 2 + "}" + "\n"
                f.write(line)
        elif type(input_dict[key]) == list:
            for iter in input_dict[key]:
                if type(iter) == dict:
                    if dict_is_empty(iter):
                        dict_to_pb_txt(iter, i + 1, f)
                    else:
                        line = " " * i * 2 + key + " {" + "\n"
                        f.write(line)
                        dict_to_pb_txt(iter, i + 1, f)
                        line = " " * i * 2 + "}" + "\n"
                        f.write(line)
                else:
                    if iter != "":
                        line = " " * i * 2 + key + ": " + iter + "\n"
                        f.write(line)
        else:
            if input_dict[key] != "":
                line = " " * i * 2 + key + ": " + input_dict[key] + "\n"
                f.write(line)
    return True


def write_extrinsic(car_sn, extrinsics_list):
    device_lsit = ["camera", "gnss", "lidar", "obc", "omc", "radar", "v2x", "vehicle"]
    car_file = param_path + car_sn + ".pb.txt"
    with open(car_file, "w") as car_f:
        print("write : {}".format(car_file))
        car_f.write("car_id: \"{}\"\n".format(car_sn))

        for i in range(len(extrinsics_list)):
            extrinsics_sub_list = extrinsics_list[i]
            device = device_lsit[i]
            for extrinsics in extrinsics_sub_list:
                print(extrinsics)
                sn = extrinsics["sn"]
                extrinsic_file = "{}{}/installation/{}.pb.txt".format(param_path, device, sn)
                if device == "vehicle":
                    extrinsic_file = "{}{}/inherent/{}.pb.txt".format(param_path, device, sn)
                with open(extrinsic_file, "w") as f:
                    dict_to_pb_txt(extrinsics, 0, f)

                    car_f.write("hardwares {\n")
                    car_f.write("  model: \"{}\"\n".format(extrinsics["model"]))
                    car_f.write("  key: \"{}\"\n".format(extrinsics["sn"]))
                    car_f.write("  type: {}\n".format(extrinsics["type"]))
                    car_f.write("}\n")
                    print("write {}".format(extrinsic_file))


def writeCSV2TXT(car_sn):
    extrinsics_list = csv_to_dict(car_sn + ".csv")
    write_extrinsic(car_sn, extrinsics_list)


if __name__ == "__main__":

    if len(sys.argv) < 3:
        print("python3 write_extrinisc_txt_csv.py csv Qxxxx\n  python3 write_extrinisc_txt_csv.py txt Qxxxx")
        sys.exit()

    if sys.argv[1] == "csv":
        writeVechicleExtrinsic(sys.argv[2])
    elif sys.argv[1] == "txt":
        writeCSV2TXT(sys.argv[2])
    else:
        print("python3 write_extrinisc_txt_csv.py csv Qxxxx\n  python3 write_extrinisc_txt_csv.py txt Qxxxx")