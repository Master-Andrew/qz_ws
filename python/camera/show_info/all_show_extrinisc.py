import os
from show_extrinsic import *

car_list = [file[:-7] for file in os.listdir(param_path) if file[0] == "Q"]
for car in car_list:
  print(car)
  showVechicleExtrinsic(car)
