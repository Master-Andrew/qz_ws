import os

param_path = "/home/qcraft/vehicles/v2/"
intrinsic_path = "/home/qcraft/vehicles/v2/camera/inherent/"
extrinsic_path = "/home/qcraft/vehicles/v2/camera/installation/"


def decode_lines(result, lines, index):
  while index < len(lines) and lines[index].split()[0] != "}":
    words = lines[index].split()
    # print(words)
    if words[-1] == "{":
      key = words[0]
      value = dict()
      value, index = decode_lines(value, lines, index + 1)

    else:
      key = words[0][:-1]
      value = words[1]

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

  # print(result)

  return result


def decodeIntrinsic(sn, intrinsic_path=intrinsic_path):
  intrinsic_file = intrinsic_path + sn + ".pb.txt"
  return decodePbTxt(intrinsic_file)


def decodeExtrinsic(sn, extrinsic_path=extrinsic_path):
  intrinsic_file = extrinsic_path + sn + ".pb.txt"
  return decodePbTxt(intrinsic_file)


def rad2degree(rad):
  return rad / 3.14159 * 180


def fill_intrinsic(intrinsic):
  if "k4" not in intrinsic["intrinsics"]["distort_coeffs"]:
    intrinsic["intrinsics"]["distort_coeffs"]["k4"] = 0
    intrinsic["intrinsics"]["distort_coeffs"]["k5"] = 0
    intrinsic["intrinsics"]["distort_coeffs"]["k6"] = 0
