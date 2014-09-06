#!/usr/bin/python

#   Calder Phillips-Grafflin

import rospy
from std_msgs.msg import *

class Logger:

    def __init__(self, sensor_topic, control_topic, reference_topic, error_topic):
        rospy.on_shutdown(self.on_shutdown)
        self.position_sub = rospy.Subscriber(sensor_topic, Float64, self.position_cb)
        self.control_sub = rospy.Subscriber(control_topic, Float64, self.control_cb)
        self.reference_sub = rospy.Subscriber(reference_topic, Float64, self.reference_cb)
        self.error_sub = rospy.Subscriber(error_topic, Float64, self.error_cb)
        self.positions = []
        self.controls = []
        self.references = []
        self.errors = []
        sleep_rate = rospy.Rate(100.0)
        while not rospy.is_shutdown():
            sleep_rate.sleep()

    def position_cb(self, msg):
        cur_time = rospy.get_time()
        self.positions.append([cur_time, msg.data])

    def control_cb(self, msg):
        cur_time = rospy.get_time()
        self.controls.append([cur_time, msg.data])

    def reference_cb(self, msg):
        cur_time = rospy.get_time()
        self.references.append([cur_time, msg.data])

    def error_cb(self, msg):
        cur_time = rospy.get_time()
        self.errors.append([cur_time, msg.data])

    def on_shutdown(self):
        print "Logging stored values to disk..."
        position_file = open("positions.csv", "w")
        for [time, value] in self.positions:
            position_file.write(str(time) + "," + str(value) + "\n")
        position_file.close()
        control_file = open("controls.csv", "w")
        for [time, value] in self.controls:
            control_file.write(str(time) + "," + str(value) + "\n")
        control_file.close()
        reference_file = open("references.csv", "w")
        for [time, value] in self.references:
            reference_file.write(str(time) + "," + str(value) + "\n")
        reference_file.close()
        error_file = open("errors.csv", "w")
        for [time, value] in self.errors:
            error_file.write(str(time) + "," + str(value) + "\n")
        error_file.close()
        print "...done, exiting"

if __name__ == '__main__':
    rospy.init_node('logger')
    control_rate = rospy.get_param("~control_rate", 30.0)
    sensor_topic = rospy.get_param("~sensor_topic", "fea/position")
    control_topic = rospy.get_param("~control_topic", "fea/duty_cycle")
    reference_topic = rospy.get_param("~reference_topic", "fea/reference")
    error_topic = rospy.get_param("~error_topic", "fea/error")
    Logger(sensor_topic, control_topic, reference_topic, error_topic)
