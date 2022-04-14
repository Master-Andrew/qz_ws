import numpy as np
import matplotlib.pyplot as plt

# Store the iteration value of each step
result = []
# Store downhill factors for each step
all_r = []


# Primitive function
def fx(xk):
  return -xk ** 0.5 + xk ** 3 + 2


def dfx(xk):
  return -0.5 * xk ** (-0.5) + 3 * xk ** 2


def ddfx(xk):
  return 0.25 * xk ** (-1.5) + 6 * xk


# Judge whether it is singular. True is singular and false is nonsingular
def strange(xk):
  return True if dfx(xk) == 0 else False


# Newton iterative formula
# xk is the iterative value and r is the downhill factor
def nd(xk, r):
  return xk - dfx(xk) / ddfx(xk) * r


# Newton downhill formula
# return True Downhill success  False Downhill failure
def nd_xs(xk, m):
  r = 1
  count = 1
  while True:
    if count > m:
      return False

    xk1 = nd(xk, r)
    if abs(fx(xk1)) < abs(fx(xk)):
      result.append(xk1)
      all_r.append(r)
      return True
    else:
      r *= 0.5

    count += 1


# Main function
def main():
  # # initial value
  # x = float(input("Please enter the initial value："))
  # result.append(x)
  # # Error limit
  # e = float(input("Please enter the error limit："))
  # # Maximum number of iterations
  # n = int(input("Please enter the maximum number of iterations："))
  # # Maximum number of downhill
  # m = int(input("Please enter the maximum number of times to go down the mountain："))

  # initial value
  x = 5
  result.append(x)
  # Error limit
  e = 0.00001
  # Maximum number of iterations
  n = 10000
  # Maximum number of downhill
  m = 1000

  # Number of iterations
  ite = 1
  while True:
    if ite > n:
      print("Number of iterations exceeded！")
      return

    if strange(result[-1]):
      print("Singular, denominator zero！")
      return

    if not nd_xs(result[-1], m):
      print("Downhill failure！")
      return

    if abs(result[-1] - result[-2]) < e:
      print("Downhill factor of each step：" + str(all_r))
      print("Iteration value of each step (including initial value)：" + str(result))
      print("Iteration value of each step (including initial value)：" + str([fx(x) for x in result]))
      print("Iteration value of each step (including initial value)：" + str([dfx(x) for x in result]))
      # return
      break

    ite += 1

  plt.plot(result, [fx(x) for x in result])
  plt.scatter(result, [fx(x) for x in result])
  x = list(np.linspace(-10, 10, num=2000, retstep=True)[0])
  y = [fx(xk) for xk in x]
  plt.plot(x, y)
  plt.show()


if __name__ == '__main__':
  main()
