# -*- coding: utf-8 -*-

import time

from MutoLib.movement import Movement, MovementMode
from MutoLib.leg import RealLeg
from MutoLib.config import *
from MutoLib.base import point3d
from MutoLib.servo import Servo


# from movement import Movement, MovementMode
# from leg import RealLeg
# from config import *
# from base import point3d
# from servo import Servo


class Hexapod(object):
    def __init__(self, ser=None):
        self._movement = Movement(MovementMode.MOVEMENT_STANDBY.value)
        self._mode = MovementMode.MOVEMENT_STANDBY.value

        self.__leg_servo = Servo(ser)
        self.__legs = [RealLeg(i, self.__leg_servo) for i in range(6)]
        
        self._dir = 0
        self._speed = 0
        self._x = 0
        self._y = 0
        self._z = 0

    def init(self, setting=False):
        if not setting:
            self.process_movement(MovementMode.MOVEMENT_STANDBY.value)
        print("PiHexa init done.")

    def process_movement(self, mode):  # 重要函数，与步态执行有关
        # if self._mode != mode:
        #     self._mode = mode
        #     self._movement.set_mode(self._mode)
        finish = False
        while not finish:
            finish, location = self._movement.next()
            # start = time.time()
            for i in range(6):
                self.__legs[i].move_tip(location.get(i))
            # print("move tip interval:", time.time()-start, finish)
            # if finish:
                # print("-----------FINISH------------------:", finish)
        

    def robot_standy(self):
        self.process_movement(MovementMode.MOVEMENT_STANDBY.value)
        self._dir = 0
        self._speed = 0
        self._x = 0
        self._y = 0
        self._z = 0

    def move(self, x, y, z):
        if x == 0 and y == 0 and z == 0:
            print("move(0, 0, 0)")
            self._dir = 0
            self._speed = 0
            self._x = 0
            self._y = 0
            self._z = 0
            self._movement.set_mode(MovementMode.MOVEMENT_STANDBY.value)
            self.process_movement(MovementMode.MOVEMENT_STANDBY.value)
            return
        if z == 0:
            self._x = x
            self._y = y
            self._z = z
            if x != 0:
                if self._speed != int(x) or self._dir != 1:
                    self._dir = 1
                    self._speed = int(x)
                    print("set_speed", self._dir, self._speed)
                    self._movement.set_speed(self._dir, self._speed)
                self.process_movement(13)
            elif y != 0:
                if self._speed != int(y) or self._dir != 2:
                    self._dir = 2
                    self._speed = int(y)
                    print("set_speed", self._dir, self._speed)
                    self._movement.set_speed(self._dir, self._speed)
                self.process_movement(13)
            
        else:
            if (x == 0 and y == 0):
                if self._speed != int(z) or self._dir != 3:
                    self._dir = 3
                    self._speed = int(z)
                    print("set_speed", self._dir, self._speed)
                    self._movement.set_speed(self._dir, self._speed)
                self.process_movement(13)
                self._x = x
                self._y = y
                self._z = z
            else: # x z 不等于0，或者y z不等于0,或者x y z都不等于0
                print("move:", x, y, z)
                if self._x != x or self._y != y or self._z != z:
                    self._speed = 0
                    self._x = x
                    self._y = y
                    self._z = z
                    self._movement.set_move_speed(x, y, z)
                self.process_movement(13)

