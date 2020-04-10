#!/usr/bin/python3
from vilib import Vilib
from ezblock import delay
from ezblock import constrain
from picarmini import set_camera_servo1_angle
from picarmini import set_dir_servo_angle
from picarmini import forward
from picarmini import stop
from ezblock import WiFi

x_axis = None
width = None
pan_angle = None

Vilib.detect_color_name('blue')
Vilib.color_detect_switch(True)
Vilib.camera_start(True)
pan_angle = 0
WiFi().write('CN', 'MakerStarsHall', 'sunfounder')


def forever():
  global x_axis, width, pan_angle
  x_axis = Vilib.color_detect_object('x')
  width = Vilib.color_detect_object('width')
  delay(5)
  if x_axis == -1:
    pan_angle = pan_angle - 1
    pan_angle = constrain(pan_angle, -90, 90)
    set_camera_servo1_angle(pan_angle)
    pan_angle = constrain(pan_angle, -45, 45)
    set_dir_servo_angle(pan_angle)
  elif x_axis == 1:
    pan_angle = pan_angle + 1
    pan_angle = constrain(pan_angle, -90, 90)
    set_camera_servo1_angle(pan_angle)
    pan_angle = constrain(pan_angle, -45, 45)
    set_dir_servo_angle(pan_angle)
  if width > 50:
    forward(50)
  else:
    stop()

if __name__ == "__main__":
    while True:
        forever()  