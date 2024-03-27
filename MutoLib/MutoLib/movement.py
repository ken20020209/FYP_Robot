# -*- coding: utf-8 -*-

from enum import Enum

from MutoLib.movement_table import *
from MutoLib.pathPlanning import PathPlanning

# from movement_table import *
# from pathPlanning import PathPlanning


"""
定义六足机器人移动相关函数。
"""


class MovementMode(Enum):
    MOVEMENT_STANDBY = 0

    MOVEMENT_FORWARD = 1
    MOVEMENT_FORWARDFAST = 2
    MOVEMENT_BACKWARD = 3
    MOVEMENT_TURNLEFT = 4
    MOVEMENT_TURNRIGHT = 5
    MOVEMENT_SHIFTLEFT = 6
    MOVEMENT_SHIFTRIGHT = 7
    MOVEMENT_CLIMB = 8
    MOVEMENT_ROTATEX = 9
    MOVEMENT_ROTATEY = 10
    MOVEMENT_ROTATEZ = 11
    MOVEMENT_TWIST = 12

    MOVEMENT_MOVE = 13

    MOVEMENT_TOTAL = 14


k_table = [standby_table,
           forward_table,
           forward_fast_table,
           backward_table,
           turn_left_table,
           turn_right_table,
           shift_left_table,
           shift_right_table,
           climb_table,
           rotate_x_table,
           rotate_y_table,
           rotate_z_table,
           twist_table,
           move_table]


class Movement(object):
    def __init__(self, mode):
        self.__mode = mode
        self.__position = locations()  # k_standby
        self.__index = 0  # index in mode position table
        self.__path_tool = PathPlanning()


    def set_mode(self, new_mode):
        self.__mode = new_mode
        table = k_table[self.__mode]
        self.__index = table.entries[0]
        print("movement.mode", self.__mode)
        # print("table", table.original)

    def set_speed(self, dir, speed):
        self.set_mode(MovementMode.MOVEMENT_MOVE.value)
        if dir == 1:
            data, mode, duration, _  = self.__path_tool.gen_move_x(int(speed))
        elif dir == 2:
            data, mode, duration, _  = self.__path_tool.gen_move_y(int(speed))
        elif dir == 3:
            data, mode, duration, _  = self.__path_tool.gen_move_z(int(speed))
        else:
            return
        point = self.__path_tool.to_points_array(data)
        # print("speed table:", speed, point)
        k_table[self.__mode].update_table(point)


    def set_move_speed(self, x, y, z):
        self.set_mode(MovementMode.MOVEMENT_MOVE.value)
        data, mode, duration, _  = self.__path_tool.gen_move2(x, y, z)
        point = self.__path_tool.to_points_array(data)
        print("move speed update table")
        k_table[self.__mode].update_table(point)

    def next(self):
        table = k_table[self.__mode]
        finish = False
        # index会循环取[0, table_length)的值
        self.__index = (self.__index + 1) % table.length
        if self.__index == 0:
            finish = True
        self.__position += (table.table[self.__index] - self.__position)
        return finish, self.__position
