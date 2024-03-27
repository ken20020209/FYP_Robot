# -*- coding: utf-8 -*-
import time
import struct


class Servo(object):
    def __init__(self, ser):
        self.__ser = ser

    def motor(self, servo_id, angle, runtime=100):
        mode = 0x01
        order = 0x40
        value_s = bytearray(struct.pack('h', int(runtime)))
        value = [servo_id, angle&0xFF, value_s[1], value_s[0]]
        value_sum = 0
        for i in range(0, 4):
            value_sum = value_sum + value[i]
        sum_data = ((4 + 0x08) + mode + order + value_sum) % 256
        sum_data = 255 - sum_data
        tx = [0x55, 0x00, (4 + 0x08), mode, order]
        tx.extend(value)
        tx.extend([sum_data, 0x00, 0xAA])
        self.__ser.write(tx)
        # print("motor:", tx)
        time.sleep(0.001)



    # 注意运动学定义的0°对应舵机90°位置
    def set_angle(self, leg_index, part_index, km_angle):
        # if part_index == 1:
        #     km_angle = -km_angle
        # print("set angle:", leg_index+1, part_index+1, km_angle)
        id = int(leg_index*3+part_index+1)
        self.motor(id, int(km_angle), 0)


if __name__ == '__main__':
    import serial
    port = "/dev/myserial"
    ser = serial.Serial(port, 115200, timeout=0.05)
    servo = Servo(ser)
    servo.motor(1, 20)

