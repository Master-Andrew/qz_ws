import os


def rename_1():
  PATH = "/home/qcraft/Downloads/camera_intrinsic/"
  list_dir = os.listdir(PATH)
  print(list_dir)
  list_dir_rename = []
  for i in range(len(list_dir)):
    list_dir_rename.append(list_dir[i][16:] + ".txt")
    os.rename(PATH + list_dir[i], PATH + list_dir_rename[i])
  print(list_dir_rename)


def rename_2(source, dist):
  intrinsic_path = "/home/qcraft/vehicles/v2/camera/inherent/"
  extrinsic_path = "/home/qcraft/vehicles/v2/camera/installation/"

  intrinsic_file_source = intrinsic_path + source + ".pb.txt"
  extrinsic_file_source = extrinsic_path + source + ".pb.txt"

  intrinsic_file_dist = intrinsic_path + dist + ".pb.txt"
  extrinsic_file_dist = extrinsic_path + dist + ".pb.txt"
  # print(intrinsic_file_source)
  if os.path.isfile(intrinsic_file_source):
    if os.path.isfile(intrinsic_file_dist):
      print(intrinsic_file_dist, " exit!!!!")
      return

    os.rename(intrinsic_file_source, intrinsic_file_dist)
    print(intrinsic_file_source, "->", intrinsic_file_dist)
  # print(extrinsic_file_source)
  if os.path.isfile(extrinsic_file_source):
    if os.path.isfile(extrinsic_file_dist):
      print(extrinsic_file_dist, " exit!!!!")
      return

    os.rename(extrinsic_file_source, extrinsic_file_dist)
    print(extrinsic_file_source, "->", extrinsic_file_dist)


# renamed: v2 / camera / inherent / CAMERA_CON4WCIC.pb.txt -> v2 / camera / inherent / H110SA - D11300042.pb.txt
# renamed: v2 / camera / inherent / CAMERA_EFXTF8YN.pb.txt -> v2 / camera / inherent / H110SA - D11300059.pb.txt
# renamed: v2 / camera / inherent / CAMERA_4RO7OE4J.pb.txt -> v2 / camera / inherent / H110SA - D11300106.pb.txt
# renamed: v2 / camera / inherent / CAMERA_GM8Q0NA1.pb.txt -> v2 / camera / inherent / H110SA - D11300118.pb.txt
# renamed: v2 / camera / inherent / CAMERA_J45RF72R.pb.txt -> v2 / camera / inherent / H110SA - D11300137.pb.txt
# renamed: v2 / camera / inherent / CAMERA_EVDR7NGP.pb.txt -> v2 / camera / inherent / H110SA - D11300336.pb.txt
# renamed: v2 / camera / inherent / CAMERA_XL9X06JY.pb.txt -> v2 / camera / inherent / H60S - D12160032.pb.txt
# renamed: v2 / camera / inherent / CAMERA_DF9VP8QJ.pb.txt -> v2 / camera / inherent / H60S - E02180027.pb.txt

"""
	modified:   v2/Q8001.pb.txt
	renamed:    v2/camera/inherent/CAMERA_CON4WCIC.pb.txt -> v2/camera/inherent/H110SA-D11300042.pb.txt
	renamed:    v2/camera/inherent/CAMERA_EFXTF8YN.pb.txt -> v2/camera/inherent/H110SA-D11300059.pb.txt
	renamed:    v2/camera/inherent/CAMERA_4RO7OE4J.pb.txt -> v2/camera/inherent/H110SA-D11300106.pb.txt
	renamed:    v2/camera/inherent/CAMERA_GM8Q0NA1.pb.txt -> v2/camera/inherent/H110SA-D11300118.pb.txt

	renamed:    v2/camera/inherent/CAMERA_J45RF72R.pb.txt -> v2/camera/inherent/H110SA-D11300137.pb.txt
	renamed:    v2/camera/inherent/CAMERA_EVDR7NGP.pb.txt -> v2/camera/inherent/H110SA-D11300336.pb.txt
	renamed:    v2/camera/inherent/CAMERA_XL9X06JY.pb.txt -> v2/camera/inherent/H60S-D12160032.pb.txt
	renamed:    v2/camera/inherent/CAMERA_DF9VP8QJ.pb.txt -> v2/camera/inherent/H60S-E02180027.pb.txt

"""
if __name__ == "__main__":
  source = "CAMERA_DF9VP8QJ"
  dist = "H60S-E02180027"
  rename_2(source, dist)
