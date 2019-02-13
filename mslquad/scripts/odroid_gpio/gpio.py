#!/usr/bin/env python

import rospy
import wiringpi as wpi

from std_msgs.msg import Bool

class GPIOSet:

    def __init__(self):
        rospy.init_node('gpio', anonymous=True)
        try:
            self.pin_num = rospy.get_param('~gpio_pin_number')
            self.topic_name = rospy.get_param('~gpio_target_topic')
        except KeyError:
            print("Required ROS param(s) not found in ROS parameter server. Did you specify correct mav name?")
        wpi.wiringPiSetup()
        wpi.pinMode(self.pin_num, 1)
        self.gpioSetSub = rospy.Subscriber(self.topic_name, Bool, self.gpio_set_cb)
        rospy.loginfo("Initiated GPIO Subscriber node.")

    def gpio_set_cb(self, command):
        if command == Bool(True):
            wpi.digitalWrite(self.pin_num, 1)
        else:
            wpi.digitalWrite(self.pin_num, 0)

if __name__ == '__main__':
    gpioset = GPIOSet()
    rospy.spin()



