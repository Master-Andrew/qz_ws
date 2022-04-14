#! /usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np


def calc_left_k_mat(k):
  """
  获得左侧k矩阵
  :param k:
  :return:
  """
  k_mat = []
  for i in range(k + 1):
    now_line = []
    for j in range(k + 1):
      now_line.append(j + i)
    k_mat.append(now_line)
  return k_mat


def calc_right_k_mat(k):
  """
  计算右侧矩阵
  :param k:
  :return:
  """
  k_mat = []
  for i in range(k + 1):
    k_mat.append([i, i + 1])
  return k_mat


def pow_k(x, k):
  """
  计算x列表中的k次方和
  :param x: 点集合的x坐标
  :param k: k值
  :return:
  """
  sum = 0
  for i in x:
    sum += i ** k
  return sum


def get_left_mat_with_x(k_mat, k):
  """
  将 左侧k矩阵运算得到左侧新的矩阵
  :param k_mat:
  :param k:
  :return:
  """
  left_mat = []
  for kl in k_mat:
    now_data = []
    for k in kl:
      now_data.append(pow_k(x, k))
    left_mat.append(now_data)
  return left_mat


def get_right_mat_with(right_k_mat):
  """
  将 右侧k矩阵运算得到右侧新的矩阵
  :param right_k_mat:
  :return:
  """
  right_mat = []
  for i in range(len(right_k_mat)):
    sum = 0
    for xL, yL in zip(x, y):
      a = (xL ** right_k_mat[i][0]) * (yL ** right_k_mat[i][1])
      sum += a
    right_mat.append(sum)
  return right_mat


def fuse_mat(left, right):
  """
  融合两个矩阵
  :param left:
  :param right:
  :return:
  """
  new_mat = []
  for i in range(len(left)):
    asd = np.append(left[i], right[i])
    new_mat.append(list(asd))
  return new_mat


if __name__ == '__main__':
  k = 3
  x = [1, 2, 3]
  y = [1, 2, 3]
  # 计算原始左侧K矩阵
  left_k_mat = calc_left_k_mat(k)
  print("原始左侧K矩阵")
  print(left_k_mat)
  # 计算原始右侧K矩阵
  right_k_mat = calc_right_k_mat(k)
  print("原始右侧k矩阵")
  print(right_k_mat)
  # 计算左侧 k 矩阵
  new_left_mat = get_left_mat_with_x(k_mat=left_k_mat, k=k)
  # 计算右侧 k 矩阵
  new_right_mat = get_right_mat_with(right_k_mat=right_k_mat)
  print("计算后左侧K矩阵")
  print(new_left_mat)
  print("计算后右侧侧K矩阵")
  print(new_right_mat)
  print("-----" * 10)
  # 融合两个矩阵 左侧 矩阵每一行增加 右侧矩阵的对应行
  new_all = fuse_mat(new_left_mat, new_right_mat)
  print("完整矩阵")
  print(new_all)
