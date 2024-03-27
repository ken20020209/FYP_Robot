from collections import deque
import numpy as np
import math

from MutoLib.config import *
from MutoLib.math_utils import *
from MutoLib.kinematicLib import Kinematic

# from config import *
# from math_utils import *
# from kinematicLib import Kinematic


class PathPlanning(object):
    def __init__(self):
        self.__steps = 20
        self.__ik = Kinematic()


    def gen_move_x(self, radius=25):
        self.__steps = 20
        step_duration = 20
        mode = "shift"
        reverse = False
        if radius < 0:
            reverse = True
        half_steps = int(self.__steps / 2)
        # path = self.__ik.semicircle_generator(abs(radius), self.__steps, reverse)
        path = self.__ik.semicircle2_generator(self.__steps, abs(radius), 25, 0, reverse)
        mir_path = deque(path)
        mir_path.rotate(half_steps)  # 相对的腿错开半个self.__steps的运动规律
        return [path, mir_path, path, mir_path, path, mir_path], mode, step_duration, (0, half_steps)

    def gen_move_y(self, radius=25):
        self.__steps = 20
        step_duration = 20
        mode = "shift"
        reverse = False
        rotate_angle = 90
        if radius < 0:
            rotate_angle = 270
        half_steps = int(self.__steps / 2)
        # path = self.__ik.semicircle_generator(abs(radius), self.__steps, reverse)
        path = self.__ik.semicircle2_generator(self.__steps, abs(radius), 25, 0, reverse)
        path = self.__ik.path_rotate_z(path, rotate_angle)
        mir_path = deque(path)
        mir_path.rotate(half_steps)  # 相对的腿错开半个self.__steps的运动规律
        return [path, mir_path, path, mir_path, path, mir_path], mode, step_duration, (0, half_steps)

    def gen_move_z(self, radius=25):
        self.__steps = 20
        step_duration = 20
        mode = "shift"
        reverse = False
        rotate_angle = 0
        if radius < 0:
            rotate_angle = 180
        half_steps = int(self.__steps / 2)
        # path = self.__ik.semicircle_generator(abs(radius), self.__steps, reverse)
        path = self.__ik.semicircle2_generator(self.__steps, abs(radius), 25, 0, reverse)
        mir_path = deque(path)
        mir_path.rotate(half_steps)  # 相对的腿错开半个self.__steps的运动规律
        path_1 = self.__ik.path_rotate_z(path, 45 + rotate_angle)
        path_2 = self.__ik.path_rotate_z(mir_path, 0 + rotate_angle)
        path_3 = self.__ik.path_rotate_z(path, 315 + rotate_angle)
        path_4 = self.__ik.path_rotate_z(mir_path, 255 + rotate_angle)
        path_5 = self.__ik.path_rotate_z(path, 180 + rotate_angle)
        path_6 = self.__ik.path_rotate_z(mir_path, 135 + rotate_angle)
        return [path_1, path_2, path_3, path_4, path_5, path_6], mode, step_duration, (0, half_steps)


    def gen_move2(self, x, y, z):
        mode = "shift"
        step_duration = 20
        half_steps = int(self.__steps / 2)
        reverse = False
        if x < 0:
            reverse = True
        # v = math.sqrt(x*x + y*y)
        v = x
        angle = z
        w = round(angle/180*math.pi, 2)
        V_offset = round(v * math.sin(w), 2)
        rotate_angle = 0
        radius_left = abs(int(v-V_offset))
        radius_right = abs(int(v+V_offset))
        print("radius:", radius_left, radius_right)
        path_left = self.__ik.semicircle2_generator(self.__steps, radius_left, 25, 0, reverse)
        path_right = self.__ik.semicircle2_generator(self.__steps, radius_right, 25, 0, reverse)

        mir_path_left = deque(path_left)
        mir_path_left.rotate(half_steps)  # 相对的腿错开半个self.__steps的运动规律
        mir_path_right = deque(path_right)
        mir_path_right.rotate(half_steps)  # 相对的腿错开半个self.__steps的运动规律

        path_1 = path_right
        path_2 = mir_path_right
        path_3 = path_right
        path_4 = mir_path_left
        path_5 = path_left
        path_6 = mir_path_left
        return [path_1, path_2, path_3, path_4, path_5, path_6], mode, step_duration, (0, half_steps)

    def verify_path(self, data, mode):
        return self.__ik.verify_path(data, mode)

    def to_points_array(self, data):
        return self.__ik.to_points_array(data)

if __name__ == "__main__":
    import time
    path = PathPlanning()
    start = time.time()
    # data, mode, duration, _ = path.gen_move_z(20)
    data, mode, duration, _ = path.gen_move2(20, 0, 10)
    # path.verify_path(data, mode)
    # print("data:", data)
    point_array = path.to_points_array(data)
    # print("point_array:", point_array)
    interval = time.time()-start
    print("interval:", interval)
