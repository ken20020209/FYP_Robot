# -*- coding: utf-8 -*-

import time
from math import sin, cos, pi, atan2, sqrt, acos

from MutoLib.config import *
from MutoLib.math_utils import *

# from config import *
# from math_utils import *
# from MutoLib import Muto


hpi = pi/2

class RealLeg(object):
    """
    定义六足机器人单腿类，包括基坐标转换和正逆运动学相关类函数。
    """
    def __init__(self, leg_index, leg_servo):
        self._leg_servo = leg_servo
        self._leg_index = leg_index
        """ 
        local_conv: 世界坐标系上的表达转换到本地坐标系上的表达
        world_conv: 本地坐标系上的表达转换到世界坐标系上的表达
        """
        if self._leg_index == 0:  # 45 or -315 degree:
            self._mount_position = point3d(leg_mount_other_x, leg_mount_other_y, 0)
            self._local_conv = rotate315  # local_P = R(local上world下，world在local上的描述) * world_P
            self._world_conv = rotate45  # world_P = R(world上local下，local在world上的描述) * local_P
            self._tip_pos_local = point3d(p1_x, p1_y, p1_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        elif self._leg_index == 1:  # 0 degree:
            self._mount_position = point3d(leg_mount_left_right_x, 0, 0)
            self._local_conv = rotate0
            self._world_conv = rotate0
            self._tip_pos_local = point3d(p2_x, p2_y, p2_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        elif self._leg_index == 2:  # -45 or 315 degree:
            self._mount_position = point3d(leg_mount_other_x, -leg_mount_other_y, 0)
            self._local_conv = rotate45
            self._world_conv = rotate315
            self._tip_pos_local = point3d(p3_x, p3_y, p3_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        elif self._leg_index == 3:  # -135 or 225 degree:
            self._mount_position = point3d(-leg_mount_other_x, -leg_mount_other_y, 0)
            self._local_conv = rotate135
            self._world_conv = rotate225
            self._tip_pos_local = point3d(p4_x, p4_y, p4_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        elif self._leg_index == 4:  # -180 or 180 degree:
            self._mount_position = point3d(-leg_mount_left_right_x, 0, 0)
            self._local_conv = rotate180
            self._world_conv = rotate180
            self._tip_pos_local = point3d(p5_x, p5_y, p5_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        elif self._leg_index == 5:  # -225 or 135 degree:
            self._mount_position = point3d(-leg_mount_other_x, leg_mount_other_y, 0)
            self._local_conv = rotate225
            self._world_conv = rotate135
            self._tip_pos_local = point3d(p6_x, p6_y, p6_z)
            self._tip_pos = self.translate2world(self._tip_pos_local)
        else:
            raise ValueError

    """coordinate system translation"""
    def translate2local(self, world_point: point3d):
        return self._local_conv(world_point - self._mount_position)

    def translate2world(self, local_point: point3d):
        return self._world_conv(local_point) + self._mount_position

    def inverse_kinematics(self, target_point: point3d):
        """坐标原点在根部舵机安装处"""
        angles = [0.0] * 3

        x = target_point.x - leg_root2joint1
        y = target_point.y
        angles[0] = atan2(y, x) * 180 / pi

        x = sqrt(x**2 + y**2) - leg_joint1_2joint2
        y = target_point.z
        ar = atan2(y, x)
        lr2 = x**2 + y**2
        lr = sqrt(lr2)
        a1 = acos((lr2 + leg_joint2_2joint3**2 - leg_joint3_2tip**2) / (2 * leg_joint2_2joint3 * lr))
        a2 = acos((lr2 - leg_joint2_2joint3**2 + leg_joint3_2tip**2) / (2 * leg_joint3_2tip * lr))
        angles[1] = (ar + a1) * 180 / pi
        angles[2] = 90 - ((a1 + a2) * 180 / pi)
        return angles

    def move_tip(self, target_point_world: point3d):
        """word coordiante system (default)"""
        if target_point_world == self._tip_pos:
            return
        dest_local = self.translate2local(target_point_world)
        # logging info
        self.__move(dest_local)
        self._tip_pos = target_point_world
        self._tip_pos_local = dest_local

    def __move(self, target_point_local: point3d):
        try:
            angles_f = self.inverse_kinematics(target_point_local)
            # print("ik ", target_point_local.x, target_point_local.y, target_point_local.z)
        except:
            print("ik Error:", target_point_local)
            return
        angles = [int(angles_f[0]), -int(angles_f[1]), int(angles_f[2])]
        for i in range(3):
            self._leg_servo.set_angle(self._leg_index, i, angles[i])

