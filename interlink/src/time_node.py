#!/usr/bin/env python

import rospy
import pygame
import sys

from interlink.msg import TimeTopic
from datetime import datetime

class Time():
    def __init__(self, sim_speed, start_hour, start_min, start_sec):
        self.hour = start_hour
        self.min = start_min
        self.sec = start_sec

        self.clock = pygame.time.Clock()
        self.clock_sec = 0

        self.sim_speed = sim_speed

    def time_elapse(self):
        # one update of simulation timeclock every 1/Rate seconds
        tick = self.clock.tick()
        self.clock_sec += tick

        if self.clock_sec >= 1000 / self.sim_speed:
        # self.clock_sec real timeclock counting for simulated second
        # 1 second in simulation = 1s/sim_speed in real
            self.clock_sec = 0
            n_sec = int(self.sec)
            n_min = int(self.min)
            n_hour = int(self.hour)
            if n_sec == 59:
                n_sec = 0
                if n_min == 59:
                    n_min = 0
                    if n_hour == 23:
                        n_hour = 0
                    else:
                        n_hour += 1
                else:
                    n_min += 1
            else:
                n_sec += 1

            if n_sec < 10:
                self.sec = "0" + str(n_sec)
            else:
                self.sec = str(n_sec)
            if n_min < 10:
                self.min = "0" + str(n_min)
            else:
                self.min = str(n_min)
            if n_hour < 10:
                self.hour = "0" + str(n_hour)
            else:
                self.hour = str(n_hour)

def time_node(control_rate, sim_speed, start_hour, start_min, start_sec):
    time_instance = Time(sim_speed, start_hour, start_min, start_sec)

    rospy.init_node("time_node", log_level = rospy.WARN)
    time_pub = rospy.Publisher("Clock", TimeTopic, queue_size = 1)

    msg = TimeTopic()
    rate = rospy.Rate(control_rate)
    while not rospy.is_shutdown():
        msg.time_stamp = time_instance.hour + ':' + time_instance.min + ':' + time_instance.sec

        msg.clock = time_instance.clock_sec

        time_pub.publish(msg)
        time_instance.time_elapse()

        rate.sleep()

if __name__ == '__main__':
    time_now = datetime.now()
    time_node(20, 1, str(time_now.hour), str(time_now.minute), str(time_now.second))