# -*- coding: utf-8 -*-

import math

from MutoLib.base import *

# from base import *

"""数学计算相关宏定义"""

SIN30 = 0.5
COS30 = 0.866
SIN45 = 0.7071
COS45 = 0.7071
SIN15 = 0.2588
COS15 = 0.9659


def rotate0(src: point3d):
    dest = point3d(src.x, src.y, src.z)
    return dest


def rotate45(src: point3d):
    dest = point3d()
    dest.x = src.x * COS45 - src.y * SIN45
    dest.y = src.x * SIN45 + src.y * COS45
    dest.z = src.z
    return dest


def rotate135(src: point3d):
    dest = point3d()
    dest.x = src.x * -COS45 - src.y * SIN45
    dest.y = src.x * SIN45 + src.y * -COS45
    dest.z = src.z
    return dest


def rotate180(src: point3d):
    dest = point3d()
    dest.x = -src.x
    dest.y = -src.y
    dest.z = src.z
    return dest


def rotate225(src: point3d):
    dest = point3d()
    dest.x = src.x * -COS45 - src.y * -SIN45
    dest.y = src.x * -SIN45 + src.y * -COS45
    dest.z = src.z
    return dest


def rotate315(src: point3d):
    dest = point3d()
    dest.x = src.x * COS45 - src.y * -SIN45
    dest.y = src.x * -SIN45 + src.y * COS45
    dest.z = src.z
    return dest

def rotate_vector(src, angle):
    dest_x = src[0]*math.cos(angle/180.0*math.pi) - src[1]*math.sin(angle/180.0*math.pi)
    dest_y = src[0]*math.sin(angle/180.0*math.pi) + src[1]*math.cos(angle/180.0*math.pi)
    dest = (dest_x, dest_y)
    return dest

if __name__ == '__main__':
    src = (5, 5)
    dest = rotate_vector(src, 45)

    print(dest)
