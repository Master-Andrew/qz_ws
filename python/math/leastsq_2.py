# 拟合曲线
import numpy as np
import matplotlib.pyplot as plt
import scipy as sp
from scipy.optimize import leastsq

# 样本数据
# 身高数据
Xi = np.array([162, 165, 159, 173, 157, 175, 161, 164, 172, 158])
# 体重数据
Yi = np.array([48, 64, 53, 66, 52, 68, 50, 52, 64, 49])


# 需要拟合的函数func（）指定函数的形状
def func(p, x):
  k, b = p
  return k * x + b


# 定义偏差函数，x，y为数组中对应Xi,Yi的值
def error(p, x, y):
  return func(p, x) - y


# 设置k，b的初始值，可以任意设定，经过实验，发现p0的值会影响cost的值：Para[1]
p0 = [1, 20]

# 把error函数中除了p0以外的参数打包到args中,leastsq()为最小二乘法函数
Para = leastsq(error, p0, args=(Xi, Yi))
# 读取结果
k, b = Para[0]
print('k=', k, 'b=', b)

# 画样本点
plt.figure(figsize=(8, 6))
plt.scatter(Xi, Yi, color='red', label='Sample data', linewidth=2)

# 画拟合直线
x = np.linspace(150, 180, 80)
y = k * x + b

# 绘制拟合曲线
plt.plot(x, y, color='blue', label='Fitting Curve', linewidth=2)
plt.legend()  # 绘制图例

plt.xlabel('Height:cm', fontproperties='simHei', fontsize=12)
plt.ylabel('Weight:Kg', fontproperties='simHei', fontsize=12)

plt.show()
