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
  intrinsic_file = "/home/qcraft/vehicles/v2/{}/inherent/{}.pb.txt".format(device, sn)
  return decodePbTxt(intrinsic_file)


def decodeExtrinsic(sn, device):
  intrinsic_file = "/home/qcraft/vehicles/v2/{}/installation/{}.pb.txt".format(device, sn)
  return decodePbTxt(intrinsic_file)


def rad2degree(rad):
  return rad / 3.14159 * 180
  # return rad


def fill_intrinsic(intrinsic):
  if "k4" not in intrinsic["intrinsics"]["distort_coeffs"]:
    intrinsic["intrinsics"]["distort_coeffs"]["k4"] = 0
    intrinsic["intrinsics"]["distort_coeffs"]["k5"] = 0
    intrinsic["intrinsics"]["distort_coeffs"]["k6"] = 0


if __name__=="__main__":
  print(decodePbTxt("/home/qcraft/vehicles/v2/omc/installation/OMC_0DXGWREG.pb.txt"))
