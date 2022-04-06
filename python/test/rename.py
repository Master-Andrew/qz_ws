import os

PATH = "/home/qcraft/Downloads/camera_intrinsic/"
list_dir = os.listdir(PATH)
print(list_dir)
list_dir_rename = []
for i in range(len(list_dir)):
    list_dir_rename.append(list_dir[i][16:] + ".txt")
    os.rename(PATH + list_dir[i], PATH + list_dir_rename[i])
print(list_dir_rename)