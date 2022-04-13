# -*- coding: utf-8 -*-


import os
import sys
import math
# import numpy as np

import geo


class ProcessENU():
    def __init__(self):
        self.wgs84_a = 6378137.0000
        self.wgs84_b = 6356752.3142
        self.wgs84_f = (self.wgs84_a - self.wgs84_b) / self.wgs84_a
        # self.wgs84_f = 1.0 / 298.257223563
        self.pow_e_2 = self.wgs84_f * (2.0 - self.wgs84_f)
        # self.pow_e_2 = (self.wgs84_a ** 2 - self.wgs84_b ** 2) / self.wgs84_a ** 2

    def pos2ecef(self, lon0, lat0, alt0):
        # print('pos2ecef', lon, lat, alt)
        lon = math.radians(float(lon0))
        lat = math.radians(float(lat0))
        alt = float(alt0)

        cosLon = math.cos(lon)
        cosLat = math.cos(lat)
        sinLon = math.sin(lon)
        sinLat = math.sin(lat)

        N = self.wgs84_a / math.sqrt(1.0 - self.pow_e_2 * sinLat * sinLat)

        ecef_x = (N + alt) * cosLat * cosLon
        ecef_y = (N + alt) * cosLat * sinLon
        ecef_z = (N * (1.0 - self.pow_e_2) + alt) * sinLat

        print('pos2ecef', lon0, lat0, alt0, '->', ecef_x, ecef_y, ecef_z)

        return (ecef_x, ecef_y, ecef_z)

    def pos2enu(self, lon, lat, alt, ref_lon, ref_lat, ref_alt):
        # lon0, lat0, alt0 = p0_lon, p0_lat, p0_alt

        ecef_x1, ecef_y1, ecef_z1 = self.pos2ecef(ref_lon, ref_lat, ref_alt)
        ecef_x0, ecef_y0, ecef_z0 = self.pos2ecef(lon, lat, alt)

        # offset_x, offset_y, offset_z = ecef_x1-ecef_x0, ecef_y1-ecef_y0, ecef_z1-ecef_z0
        offset_x, offset_y, offset_z = ecef_x0 - ecef_x1, ecef_y0 - ecef_y1, ecef_z0 - ecef_z1
        # array_delta = np.array( [[offset_x], [offset_y], [offset_z]], dtype=np.float64)
        # print(array_delta.reshape(1,3))

        # lon0 = math.radians(lon)
        # lat0 = math.radians(lat)

        lon0 = math.radians(ref_lon)
        lat0 = math.radians(ref_lat)

        cosLon = math.cos(lon0)
        cosLat = math.cos(lat0)
        sinLon = math.sin(lon0)
        sinLat = math.sin(lat0)

        x = -1 * sinLon * offset_x + cosLon * offset_y
        y = -1 * sinLat * cosLon * offset_x - 1 * sinLat * sinLon * offset_y + cosLat * offset_z
        z = cosLat * cosLon * offset_x + cosLat * sinLon * offset_y + sinLat * offset_z
        return (x, y, z)

        # array_S1 = np.array([-1 * sinLon,           cosLon,                 0], dtype=np.float64)
        # array_S2 = np.array([-1 * sinLat * cosLon,  -1 * sinLat *sinLon,    cosLat], dtype=np.float64)
        # array_S3 = np.array([cosLat * cosLon,       cosLat * sinLon,        sinLat], dtype=np.float64)
        # array_S = np.array([array_S1, array_S2, array_S3], dtype=np.float64)

        # v = np.dot(array_S, array_delta)
        # # print(v.reshape(1,3))
        # # print(v[0][0], v[1][0], v[2][0])
        # return (v[0][0], v[1][0], v[2][0])


if __name__ == "__main__":
    p0_lon, p0_lat, p0_alt = 121.18928781247354, 31.28466766203434, 15.004388429224491
    p1_lon, p1_lat, p1_alt = 121.18928390608936, 31.284666663066726, 16.226316452026367

    enu1 = ProcessENU()
    v1 = enu1.pos2enu(p0_lon, p0_lat, p0_alt, p1_lon, p1_lat, p1_alt)
    print(v1)

    # v2 = enu1.pos2ecef(p0_lon, p0_lat, p0_alt)
    # print(v2)

    # v3 = geo.geodetic_to_enu(p0_lat, p0_lon, p0_alt, p1_lat, p1_lon, p1_alt)
    # print(v3)
