from collections import deque
import math
import numpy as np
import copy

from MutoLib.config import *

# from config import *


class Kinematic(object):

    def __init__(self):
        pass

    """坐标原点在根部舵机安装处"""
    def ik(self, to):
        angles = []
        x = to[0] - leg_root2joint1
        y = to[1]

        angles.append(math.atan2(y, x) * 180 / math.pi)

        x = math.sqrt(x*x + y*y) - leg_joint1_2joint2
        y = to[2]
        ar = math.atan2(y, x)
        lr2 = x*x + y*y
        lr = math.sqrt(lr2)
        a1 = math.acos((lr2 + leg_joint2_2joint3**2 - leg_joint3_2tip**2)/(2*leg_joint2_2joint3*lr))
        a2 = math.acos((lr2 - leg_joint2_2joint3**2 + leg_joint3_2tip**2)/(2*leg_joint3_2tip*lr))

        angles.append((ar + a1) * 180 / math.pi)
        angles.append(90 - ((a1 + a2) * 180 / math.pi))
        return angles

    def ik2Angle(self,ox,oy,oz,l0=27.5,l1=50.6,l2=72.6,l3=134.5):
        x = ox - l0
        y = oy
        z = oz
        a = math.atan2(y, x)
        l23 = math.sqrt(x*x+y*y)-l1
        ar = math.atan2(-z, l23)
        lr2 = l23*l23+z*z
        lr = math.sqrt(lr2)
        a1 = math.acos((lr2+l2*l2-l3*l3)/(2*l2*lr))
        a2 = math.acos((lr2-l2*l2+l3*l3)/(2*l3*lr))
        b = a1-ar
        c = math.math.pi/2-(a1+a2)
        angle_1 = round(math.degrees(a))
        angle_2 = round(math.degrees(b))
        angle_3 = round(math.degrees(c))
        return angle_1, angle_2, angle_3

    # 步态是简单的半圆弧曲线
    def semicircle_generator(self, radius, steps, reverse=False):
        assert (steps % 4) == 0  # steps把一个圆分成多少步
        half_steps = int(steps/2)
        step_angle = math.pi / half_steps
        step_stride = 2*radius / half_steps
        result = []
        # 前半部分，后移（只有y轴改变——腿接触地面）
        for i in range(half_steps):
            result.append((0, round(radius - i*step_stride, 2), 0))  # 直线 [radius, 0]
        # print("part1: ", result)
        # 第二（后半）部分，以半圆形状前移（y，z 改变）
        for i in range(half_steps):
            angle = math.pi - step_angle*i  # 范围：[pi, 0]
            y = radius * math.cos(angle)  # math.pi的范围决定y从负数开始 [-radius, radius]
            z = radius * math.sin(angle)  # y和z组合的半圆
            result.append((0, round(y,2), round(z,2)))
        # print("part2: ", result)
        result = deque(result)
        # print("part3: ", result)
        result.rotate(int(steps/4))  # y从3/4处开始轮转，而不是0处
        # print("part4: ", result)
        if reverse:
            result = deque(reversed(result))
            result.rotate(1)
        return result

    def semicircle2_generator(self, steps, y_radius, z_radius, x_radius, reverse=False):
        assert (steps % 4) == 0
        half_steps = int(steps / 2)
        step_angle = math.pi / half_steps
        step_y_stride = 2 * y_radius / half_steps
        result = []
        # 前半部分，后移（只有y轴改变——腿接触地面）
        for i in range(half_steps):
            result.append((0, y_radius - i * step_y_stride, 0))  # 直线 [radius, 0]
        # 第二（后半）部分，以半圆形状前移（y，z 改变）
        for i in range(half_steps):
            angle = math.pi - step_angle * i
            y = y_radius * math.cos(angle)
            z = z_radius * math.sin(angle)
            x = x_radius * math.sin(angle)
            result.append((x, y, z))
        result = deque(result)
        result.rotate(int(steps / 4))
        if reverse:
            result = deque(reversed(result))
            result.rotate(1)
        return result


    


    def get_rotate_x_matrix(self, angle):
        angle = angle * math.pi / 180
        return np.array([[1, 0, 0, 0],
                        [0, round(math.cos(angle), 2), round(-math.sin(angle), 2), 0],
                        [0, round(math.sin(angle), 2), round(math.cos(angle), 2), 0],
                        [0, 0, 0, 1]])


    def get_rotate_y_matrix(self, angle):
        angle = angle * math.pi / 180
        return np.array([[round(math.cos(angle), 2), 0, round(math.sin(angle), 2), 0],
                        [0, 1, 0, 0],
                        [round(-math.sin(angle), 2), 0, round(math.cos(angle), 2), 0],
                        [0, 0, 0, 1]])


    def get_rotate_z_matrix(self, angle):
        angle = angle * math.pi / 180
        return np.array([[round(math.cos(angle), 2), round(-math.sin(angle), 2), 0, 0],
                        [round(math.sin(angle), 2), round(math.cos(angle), 2), 0, 0],
                        [0, 0, 1, 0],
                        [0, 0, 0, 1]])


    def matrix_mul(self, tr_matrix, pt):
        ptx = list(pt) + [1]
        return list(np.matmul(tr_matrix, np.array(ptx).T))[:-1]


    def point_rotate_x(self, pt, angle):
        tr_matrix = self.get_rotate_x_matrix(angle)
        return self.matrix_mul(tr_matrix, pt)


    def point_rotate_y(self, pt, angle):
        tr_matrix = self.get_rotate_y_matrix(angle)
        return self.matrix_mul(tr_matrix, pt)


    def point_rotate_z(self, pt, angle):
        tr_matrix = self.get_rotate_z_matrix(angle)
        return self.matrix_mul(tr_matrix, pt)


    def path_rotate_x(self, path, angle):
        return [self.point_rotate_x(p, angle) for p in path]


    def path_rotate_y(self, path, angle):
        return [self.point_rotate_y(p, angle) for p in path]


    def path_rotate_z(self, path, angle):
        return [self.point_rotate_z(p, angle) for p in path]


    def verify_points(self, pt):
        '''
        检查坐标点是否符合要求
        '''
        angles = self.ik(pt)
        ok = True
        failed = []
        for i, angle in enumerate(angles):
            if angle < angleLimitation[i][0] or angle > angleLimitation[i][1]:
                ok = False
                failed.append((i, angle))
        return ok, failed

    def verify_path(self, data, mode):
        all_ok = True
        if mode == "shift":
            # data: float[6][N][3]
            assert (len(data) == 6)
            result = []
            for i in range(len(data[0])):
                for j in range(6):
                    pt = [default_position[j][k] - mount_position[j][k] + data[j][i][k] for k in range(3)]
                    pt = self.point_rotate_z(pt, default_angle[j])
                    ok, failed = self.verify_points(pt)
                    if not ok:
                        print("{}, {} failed: {}".format(i, j, failed))
                        all_ok = False
        else:
            all_ok = False
        return all_ok

    def to_points_array(self, path_array):
        # path_array: float[6][N][3]
        count = len(path_array[0])
        # result[6][count=20][3]  k_standby
        standy = copy.deepcopy(k_standby)
        result = [[(0, 0, 0) for j in range(6)] for i in range(count)]
        for i in range(0,count,1):
            for j in range(0,6,1):
                t_x = round(standy[j][0] + path_array[j][i][0], 2)
                t_y = round(standy[j][1] + path_array[j][i][1], 2)
                t_z = round(standy[j][2] + path_array[j][i][2], 2)
                temp = (t_x, t_y, t_z)
                result[i][j] = temp
        return result



if __name__ == "__main__":
    import time
    ik = Kinematic()
    start = time.time()
    result = ik.semicircle_generator(25, 20)
    print("result: ", result)
    interval = time.time()-start
    print("interval:", interval)

