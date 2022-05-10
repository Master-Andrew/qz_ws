import csv

a = "  calibration_time: \"2021-11-06 04:44:21\"\n"

b = "    calibration_time {\n"


def split_line(line):
  if "{" in line:
    return line.split()
  else:
    result = line.split(":", 1)
    for i in range(len(result)):
      result[i] = result[i].split()[0]
    return result


with open("/home/qcraft/vehicles/v2/camera/installation/H110SA-D07230514.pb.txt") as f:
  lines = f.readlines()
  for line in lines:
    print(split_line(line))


