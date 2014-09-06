#!/usr/bin/python

#   Calder Phillips-Grafflin

import rospy
import math
from std_msgs.msg import *
import numpy

class FEAController:

    def __init__(self, control_rate, sensor_topic, command_topic, control_topic, reference_topic, error_topic, kp, kd, ki):
        self.control_interval = 1.0 / control_rate
        self.control_rate = rospy.Rate(control_rate)
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.current_command = 0.0
        self.integral_error = 0.0
        self.last_position_error = 0.0
        self.current_position = 0.0
        self.current_position_sub = rospy.Subscriber(sensor_topic, Float64, self.current_position_cb)
        self.command_sub = rospy.Subscriber(command_topic, Float64, self.current_command_cb)
        self.control_pub = rospy.Publisher(control_topic, Float64)
        self.reference_pub = rospy.Publisher(reference_topic, Float64)
        self.error_pub = rospy.Publisher(error_topic, Float64)
        while not rospy.is_shutdown():
            self.compute_control()
            self.control_rate.sleep()

    def current_position_cb(self, msg):
        if math.isnan(msg.data):
            rospy.logwarn("NaN angle, ignoring")
        else:
            self.current_position = msg.data

    def current_command_cb(self, msg):
        self.current_command = msg.data

    def compute_control(self):
        # Update current position
        current_position = self.current_position
        # Get the current desired position
        desired_position = self.current_command
        # Compute the current position error
        position_error = desired_position - current_position
        # Update the last derivative error
        derivative_error = (position_error - self.last_position_error) / self.control_interval
        self.last_position_error = position_error
        # Update the integral error
        self.integral_error = self.integral_error + (position_error * self.control_interval)
        # Compute the PID values
        p_correction = self.kp * position_error
        d_correction = self.kd * derivative_error
        i_correction = self.ki * self.integral_error
        print "P correction:", p_correction
        print "D correction:", d_correction
        print "I correction:", i_correction
        # Combine the corrections together
        correction = p_correction + d_correction + i_correction
        print "Total correction:", correction
        # Compute a feedforward value
        feedforward = self.compute_feedforward(desired_position)
        print "Feedforward:", feedforward
        # Combine correction with feedforward
        output = feedforward + correction
        print "Command:", output
        # Command the actuator
        command = output
        if (command < 0.0):
            command = 0.0
        elif (command > 1.0):
            command = 1.0
        control_msg = Float64()
        control_msg.data = output
        self.control_pub.publish(control_msg)
        # Publish data
        reference_msg = Float64()
        reference_msg.data = desired_position
        self.reference_pub.publish(reference_msg)
        error_msg = Float64()
        error_msg.data = position_error
        self.error_pub.publish(error_msg)

    def compute_feedforward(self, desired_position):
        if (desired_position < 0.0):
            desired_position = 0.0
        elif (desired_position > 1.25):
            desired_positon = 1.25
        # Base offset
        base_offset = 50.0
        slope = 39.8406
        return (base_offset + (slope * desired_position)) * 0.01

if __name__ == '__main__':
    rospy.init_node('valve_controller')
    control_rate = rospy.get_param("~control_rate", 30.0)
    sensor_topic = rospy.get_param("~sensor_topic", "fea/position")
    command_topic = rospy.get_param("~command_topic", "fea/command")
    control_topic = rospy.get_param("~control_topic", "fea/duty_cycle")
    reference_topic = rospy.get_param("~reference_topic", "fea/reference")
    error_topic = rospy.get_param("~error_topic", "fea/error")
    kp = rospy.get_param("~kp", 2.0)
    kd = rospy.get_param("~kd", 1.0)
    ki = rospy.get_param("~ki", 1.0)
    FEAController(control_rate, sensor_topic, command_topic, control_topic, reference_topic, error_topic, kp, kd, ki)
