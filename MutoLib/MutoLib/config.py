# -*- coding: utf-8 -*-
from MutoLib.math_utils import *
# from math_utils import *


# mounting position
leg_mount_left_right_x = 53.4 # position in x direction of the middle legs
leg_mount_other_x = 31.12     # position in x direction of the fore and hind legs
leg_mount_other_y = 83.07     # position in y direction of the fore and hind legs
mount_position = ((leg_mount_other_x, leg_mount_other_y, 0),
                  (leg_mount_left_right_x, 0, 0),
                  (leg_mount_other_x, -leg_mount_other_y, 0),
                  (-leg_mount_other_x, -leg_mount_other_y, 0),
                  (-leg_mount_left_right_x, 0, 0),
                  (-leg_mount_other_x, leg_mount_other_y, 0))

# link length
leg_root2joint1 = 27.5
leg_joint1_2joint2 = 50.59
leg_joint2_2joint3 = 72.60
leg_joint3_2tip = 134.5


# default mounting angle
default_angle = (-45, 0, 45, 135, 180, 225)

# angle limitation
angleLimitation = ((-45, 45), (-45, 75), (-60, 60))

# movement constance
standby_z = leg_joint3_2tip * COS15 - leg_joint2_2joint3 * SIN30
left_right_x = leg_mount_left_right_x + leg_root2joint1 + leg_joint1_2joint2 + leg_joint2_2joint3 * COS30 + leg_joint3_2tip * SIN15
other_x = leg_mount_other_x + (
            leg_root2joint1 + leg_joint1_2joint2 + leg_joint2_2joint3 * COS30 + leg_joint3_2tip * SIN15) * COS45
other_y = leg_mount_other_y + (
            leg_root2joint1 + leg_joint1_2joint2 + leg_joint2_2joint3 * COS30 + leg_joint3_2tip * SIN15) * SIN45

# paths way points' locations description
p1_x = other_x
p1_y = other_y
p1_z = -standby_z

p2_x = left_right_x
p2_y = 0
p2_z = -standby_z

p3_x = other_x
p3_y = -other_y
p3_z = -standby_z

p4_x = -other_x
p4_y = -other_y
p4_z = -standby_z

p5_x = -left_right_x
p5_y = 0
p5_z = -standby_z

p6_x = -other_x
p6_y = other_y
p6_z = -standby_z

# for paths generation
default_position = ((other_x, other_y, -standby_z),
                    (left_right_x, 0, -standby_z),
                    (other_x, -other_y, -standby_z),
                    (-other_x, -other_y, -standby_z),
                    (-left_right_x, 0, -standby_z),
                    (-other_x, other_y, -standby_z))

# for paths locations calculation of real robot
k_standby = [(p1_x, p1_y, p1_z),
             (p2_x, p2_y, p2_z),
             (p3_x, p3_y, p3_z),
             (p4_x, p4_y, p4_z),
             (p5_x, p5_y, p5_z),
             (p6_x, p6_y, p6_z)]
standby_entries = (0, )
standby_entries_count = 1  # Number of elements of variable standby_entries


START_PX = leg_mount_other_x + leg_root2joint1 * COS45
START_PY = leg_mount_other_y + leg_root2joint1 * SIN45

START_PX_M = leg_mount_left_right_x + leg_root2joint1
START_PY_M = 0

PO1 = (START_PX, START_PY)
PO2 = (START_PX_M, START_PY_M)
PO3 = (START_PX, -START_PY)
PO4 = (-START_PX, -START_PY)
PO5 = (-START_PX_M, START_PY_M)
PO6 = (-START_PX, START_PY)

# print(PO1)
# print(PO2)
# (50.56525, 102.51525)
# (80.9, 0)

move_paths = [[(p1_x + 0.00, p1_y + 0.00, p1_z + 25.00), (p2_x + 0.00, p2_y + 0.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 0.00, p3_z + 25.00), (p4_x + 0.00, p4_y + 0.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 0.00, p5_z + 25.00), (p6_x + 0.00, p6_y + 0.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 7.73, p1_z + 23.78), (p2_x + 0.00, p2_y + -5.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 7.73, p3_z + 23.78), (p4_x + 0.00, p4_y + -5.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 7.73, p5_z + 23.78), (p6_x + 0.00, p6_y + -5.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 14.69, p1_z + 20.23), (p2_x + 0.00, p2_y + -10.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 14.69, p3_z + 20.23), (p4_x + 0.00, p4_y + -10.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 14.69, p5_z + 20.23), (p6_x + 0.00, p6_y + -10.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 20.23, p1_z + 14.69), (p2_x + 0.00, p2_y + -15.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 20.23, p3_z + 14.69), (p4_x + 0.00, p4_y + -15.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 20.23, p5_z + 14.69), (p6_x + 0.00, p6_y + -15.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 23.78, p1_z + 7.73), (p2_x + 0.00, p2_y + -20.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 23.78, p3_z + 7.73), (p4_x + 0.00, p4_y + -20.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 23.78, p5_z + 7.73), (p6_x + 0.00, p6_y + -20.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 25.00, p1_z + 0.00), (p2_x + 0.00, p2_y + -25.00, p2_z + 0.00), (p3_x + 0.00, p3_y + 25.00, p3_z + 0.00), (p4_x + 0.00, p4_y + -25.00, p4_z + 0.00), (p5_x + 0.00, p5_y + 25.00, p5_z + 0.00), (p6_x + 0.00, p6_y + -25.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + 20.00, p1_z + 0.00), (p2_x + 0.00, p2_y + -23.78, p2_z + 7.73), (p3_x + 0.00, p3_y + 20.00, p3_z + 0.00), (p4_x + 0.00, p4_y + -23.78, p4_z + 7.73), (p5_x + 0.00, p5_y + 20.00, p5_z + 0.00), (p6_x + 0.00, p6_y + -23.78, p6_z + 7.73)],
                 [(p1_x + 0.00, p1_y + 15.00, p1_z + 0.00), (p2_x + 0.00, p2_y + -20.23, p2_z + 14.69), (p3_x + 0.00, p3_y + 15.00, p3_z + 0.00), (p4_x + 0.00, p4_y + -20.23, p4_z + 14.69), (p5_x + 0.00, p5_y + 15.00, p5_z + 0.00), (p6_x + 0.00, p6_y + -20.23, p6_z + 14.69)],
                 [(p1_x + 0.00, p1_y + 10.00, p1_z + 0.00), (p2_x + 0.00, p2_y + -14.69, p2_z + 20.23), (p3_x + 0.00, p3_y + 10.00, p3_z + 0.00), (p4_x + 0.00, p4_y + -14.69, p4_z + 20.23), (p5_x + 0.00, p5_y + 10.00, p5_z + 0.00), (p6_x + 0.00, p6_y + -14.69, p6_z + 20.23)],
                 [(p1_x + 0.00, p1_y + 5.00, p1_z + 0.00), (p2_x + 0.00, p2_y + -7.73, p2_z + 23.78), (p3_x + 0.00, p3_y + 5.00, p3_z + 0.00), (p4_x + 0.00, p4_y + -7.73, p4_z + 23.78), (p5_x + 0.00, p5_y + 5.00, p5_z + 0.00), (p6_x + 0.00, p6_y + -7.73, p6_z + 23.78)],
                 [(p1_x + 0.00, p1_y + 0.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 0.00, p2_z + 25.00), (p3_x + 0.00, p3_y + 0.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 0.00, p4_z + 25.00), (p5_x + 0.00, p5_y + 0.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 0.00, p6_z + 25.00)],
                 [(p1_x + 0.00, p1_y + -5.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 7.73, p2_z + 23.78), (p3_x + 0.00, p3_y + -5.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 7.73, p4_z + 23.78), (p5_x + 0.00, p5_y + -5.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 7.73, p6_z + 23.78)],
                 [(p1_x + 0.00, p1_y + -10.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 14.69, p2_z + 20.23), (p3_x + 0.00, p3_y + -10.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 14.69, p4_z + 20.23), (p5_x + 0.00, p5_y + -10.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 14.69, p6_z + 20.23)],
                 [(p1_x + 0.00, p1_y + -15.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 20.23, p2_z + 14.69), (p3_x + 0.00, p3_y + -15.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 20.23, p4_z + 14.69), (p5_x + 0.00, p5_y + -15.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 20.23, p6_z + 14.69)],
                 [(p1_x + 0.00, p1_y + -20.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 23.78, p2_z + 7.73), (p3_x + 0.00, p3_y + -20.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 23.78, p4_z + 7.73), (p5_x + 0.00, p5_y + -20.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 23.78, p6_z + 7.73)],
                 [(p1_x + 0.00, p1_y + -25.00, p1_z + 0.00), (p2_x + 0.00, p2_y + 25.00, p2_z + 0.00), (p3_x + 0.00, p3_y + -25.00, p3_z + 0.00), (p4_x + 0.00, p4_y + 25.00, p4_z + 0.00), (p5_x + 0.00, p5_y + -25.00, p5_z + 0.00), (p6_x + 0.00, p6_y + 25.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + -23.78, p1_z + 7.73), (p2_x + 0.00, p2_y + 20.00, p2_z + 0.00), (p3_x + 0.00, p3_y + -23.78, p3_z + 7.73), (p4_x + 0.00, p4_y + 20.00, p4_z + 0.00), (p5_x + 0.00, p5_y + -23.78, p5_z + 7.73), (p6_x + 0.00, p6_y + 20.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + -20.23, p1_z + 14.69), (p2_x + 0.00, p2_y + 15.00, p2_z + 0.00), (p3_x + 0.00, p3_y + -20.23, p3_z + 14.69), (p4_x + 0.00, p4_y + 15.00, p4_z + 0.00), (p5_x + 0.00, p5_y + -20.23, p5_z + 14.69), (p6_x + 0.00, p6_y + 15.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + -14.69, p1_z + 20.23), (p2_x + 0.00, p2_y + 10.00, p2_z + 0.00), (p3_x + 0.00, p3_y + -14.69, p3_z + 20.23), (p4_x + 0.00, p4_y + 10.00, p4_z + 0.00), (p5_x + 0.00, p5_y + -14.69, p5_z + 20.23), (p6_x + 0.00, p6_y + 10.00, p6_z + 0.00)],
                 [(p1_x + 0.00, p1_y + -7.73, p1_z + 23.78), (p2_x + 0.00, p2_y + 5.00, p2_z + 0.00), (p3_x + 0.00, p3_y + -7.73, p3_z + 23.78), (p4_x + 0.00, p4_y + 5.00, p4_z + 0.00), (p5_x + 0.00, p5_y + -7.73, p5_z + 23.78), (p6_x + 0.00, p6_y + 5.00, p6_z + 0.00)]]

