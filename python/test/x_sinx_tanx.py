from math import sin, tan

print(tan(45 / 180 * 3.14159))

for i in range(10000):
    x = i / 1e4
    sinx = sin(x)
    if abs(abs(sinx - x)) > 1e-3:
        print(x, x / 3.14159 * 180, sinx, sinx - x)
        break

for i in range(10000):
    x = i / 1e4
    tanx = tan(x)
    if abs(abs(tanx - x)) > 1e-3:
        print(x, x / 3.14159 * 180, tanx, tanx - x)
        break
