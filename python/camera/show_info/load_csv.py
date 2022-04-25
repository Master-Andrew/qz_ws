import numpy as np

def load_extrinsic_file(name):
  tmp = np.loadtxt(name, dtype=np.str, delimiter=",")
  result = {}
  for data in tmp[1:]:
    if data[0] != "":
      id = data[0]
      result[id] = {}
      result[id]["x"] = float(data[1])
      result[id]["y"] = float(data[2])
      result[id]["z"] = float(data[3])
      result[id]["roll"] = float(data[4])
      result[id]["pitch"] = float(data[5])
      result[id]["yaw"] = float(data[6])
      result[id]["ref"] = data[7]

  return result  # 返回array类型的数据


def load_position_file(name):
  tmp = np.loadtxt(name, dtype=np.str, delimiter=",")
  lable = tmp[0, :]
  result = {}
  for data in tmp[1:]:
    result[data[0]] = {}
    for i in range(1, len(data)):
      result[data[0]][lable[i]] = data[i]

  return result

if __name__ == "__main__":
  # name = "Q80_jinlv.csv"
  # result = load_extrinsic_file(name)
  name = "camera_position.csv"
  result = load_position_file(name)
  print(result)
