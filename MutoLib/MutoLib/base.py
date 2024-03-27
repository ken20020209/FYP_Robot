# -*- coding: utf-8 -*-

"""
定义六足机器人运动学计算需要用到的基础类及其运算方法（通过操作符重载方式）。
"""


class point3d(object):
    def __init__(self, x=.0, y=.0, z=.0):
        self.x = x
        self.y = y
        self.z = z

    @classmethod
    def from_tuple(cls, point_tuple):
        return cls(point_tuple[0], point_tuple[1], point_tuple[2])

    def __sub__(self, other):
        return point3d(self.x - other.x, self.y - other.y, self.z - other.z)

    def __add__(self, other):
        return point3d(self.x + other.x, self.y + other.y, self.z + other.z)

    def __mul__(self, rhs):
        return point3d(self.x * rhs, self.y * rhs, self.z * rhs)

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z


class locations(object):
    def __init__(self, fore_right=point3d(), right=point3d(), hind_right=point3d(), hind_left=point3d(), left=point3d(), fore_left=point3d()):
        self.__points = [fore_right, right, hind_right, hind_left, left, fore_left]

    @classmethod
    def from_list(cls, points_list):
        point3d_list = [point3d.from_tuple(point_tuple) for point_tuple in points_list]
        return cls(point3d_list[0], point3d_list[1], point3d_list[2], point3d_list[3], point3d_list[4], point3d_list[5])

    def get(self, index):
        return self.__points[index]

    def __sub__(self, other):
        return locations(self.__points[0] - other.__points[0],
                         self.__points[1] - other.__points[1],
                         self.__points[2] - other.__points[2],
                         self.__points[3] - other.__points[3],
                         self.__points[4] - other.__points[4],
                         self.__points[5] - other.__points[5])

    def __add__(self, other):
        return locations(self.__points[0] + other.__points[0],
                         self.__points[1] + other.__points[1],
                         self.__points[2] + other.__points[2],
                         self.__points[3] + other.__points[3],
                         self.__points[4] + other.__points[4],
                         self.__points[5] + other.__points[5])

    def __mul__(self, rhs):
        return locations(self.__points[0] * rhs,
                         self.__points[1] * rhs,
                         self.__points[2] * rhs,
                         self.__points[3] * rhs,
                         self.__points[4] * rhs,
                         self.__points[5] * rhs)

    def __str__(self):
        return str([(point.x, point.y, point.z) for point in self.__points])


if __name__ == '__main__':
    
    point_a = point3d(1, 2, 3)
    point_b = point3d(4, 5, 6)
    point_c = point_a - point_b
    # point_c += point_c
    # point_c = point_c * 3
    point_d = point3d(point_a.x, point_a.y, point_a.z)
    print(point_c.x, point_c.y, point_c.z)
    print(point_a == point_d)

