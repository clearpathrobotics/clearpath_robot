#!/usr/bin/env python3

import tkinter as tk
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from clearpath_platform_msgs.msg import Lights, RGB
import threading

frames: list[tk.Frame] = []


class LightsSub(Node):
    def __init__(self):
        super().__init__('lights_sub')
        self.publisher_ = self.create_subscription(Lights, 'platform/mcu/_cmd_lights', self.sub_cb, qos_profile_sensor_data)

    def sub_cb(self, msg: Lights):
        global frames
        for i, rgb in enumerate(msg.lights):
            rgb: RGB
            #print(rgb)
            red = hex(rgb.red)[2:].zfill(2)
            if rgb.red == 0:
              red = '00'
            green = f'{rgb.green:#02x}'[2:].zfill(2)
            if rgb.green == 0:
              green = '00'
            blue = f'{rgb.blue:#02x}'[2:].zfill(2)
            if rgb.blue == 0:
              blue = '00'
            bg = '#' + str(red) + str(green) + str(blue)
            #print(bg)
            frames[i].config(bg=bg)


window = tk.Tk()

for i in range(2):
    for j in range(2):
        frame = tk.Frame(master=window, width=150, height=150, bg="red", highlightbackground="black", highlightthickness=1)
        frame.grid(row=i, column=j)
        frames.append(frame)

rclpy.init()
sub = LightsSub()

thread = threading.Thread(target=rclpy.spin, args=(sub,), daemon=True)
thread.start()



window.mainloop()
