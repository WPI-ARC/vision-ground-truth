#!/usr/bin/python

#   Calder Phillips-Grafflin

import rospy
from std_msgs.msg import *
from arduino_valve_driver import *

class ArduinoValve:

    def __init__(self, port):
        self.valve = ArduinoValveDriver(port)
        self.command_sub = rospy.Subscriber("fea/duty_cycle", Float64, self.command_cb)
        rate = rospy.Rate(rospy.get_param('~hz', 100))
        while not rospy.is_shutdown():
            rate.sleep()

    def command_cb(self, msg):
        rospy.loginfo("Commanding valve to " + str(msg.data) + " duty cycle")
        self.valve.SetDutyCycle(msg.data)

if __name__ == '__main__':
    rospy.init_node('valve_driver')
    port = rospy.get_param("~port", "/dev/ttyACM0")
    ArduinoValve(port)
