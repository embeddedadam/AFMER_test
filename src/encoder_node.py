#!/usr/bin/python3
import rospy
import roslib
import math 
import time
import RPi.GPIO as GPIO

# Messages
from std_msgs.msg import Float32

class LLC_encoder:
    def __init__(self, pin_a, pin_b):
        GPIO.setmode(GPIO.BCM)
        self.a = pin_a
        self.b = pin_b
        GPIO.setup(self.a, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.b, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        GPIO.add_event_detect(self.a, GPIO.BOTH, callback=self.count, bouncetime=0.1)
        GPIO.add_event_detect(self.b, GPIO.BOTH, callback=self.count, bouncetime=0.1)

        self.gear_ratio = 3.6
        self.enc_impulses_per_motor_rot = 20
        self.enc_impulses_per_wheel_rot = self.enc_impulses_per_motor_rot * self.gear_ratio
        self.aState = False
        self.bState = False
        self.aLastState = False
        self.bLastState = False
        self.counter = 0
        self.aLastState = GPIO.input(self.a)

    def read_rotations(self):
        return self.counter / self.enc_impulses_per_wheel_rot

    def reset(self):
        self.counter = 0

    def count(self, channel):
        self.aState = GPIO.input(self.a)
        self.bState = GPIO.input(self.b)
        if self.aLastState != self.aState:
            if GPIO.input(self.b) != self.aState:
                self.counter -= 1
            else:
                self.counter += 1

        if self.bLastState != self.bState:
            if GPIO.input(self.a) == self.bState:
                self.counter -= 1
            else:
                self.counter += 1
        self.bLastState = self.bState
        self.aLastState = self.aState

class WheelsEncodersPublishers:
    def __init__(self):
        rospy.init_node("encoders_node")
        self.enc1 = LLC_encoder(21, 26) #lustrzane odbicie/podmiana pinow
        self.enc2 = LLC_encoder(13, 16)
        self.enc3 = LLC_encoder(5, 25) #lustrzane odbicie/podmiana pinow
        self.enc4 = LLC_encoder(18, 27)

        self.wheel_1_vel_publisher = rospy.Publisher("wheel_1_vel", Float32, queue_size=10)
        self.wheel_2_vel_publisher = rospy.Publisher("wheel_2_vel", Float32, queue_size=10)
        self.wheel_3_vel_publisher = rospy.Publisher("wheel_3_vel", Float32, queue_size=10)
        self.wheel_4_vel_publisher = rospy.Publisher("wheel_4_vel", Float32, queue_size=10)

        self.rate = rospy.get_param('~rate', 20)
        self.R = rospy.get_param('~robot_wheel_radius', 0.09)
        self.time_prev_update = rospy.Time.now()

        self.enc1_data = self.enc1.read_rotations()
        self.enc2_data = self.enc2.read_rotations()
        self.enc3_data = self.enc3.read_rotations()
        self.enc4_data = self.enc4.read_rotations()
        self.buffer_4 = 0

    def enc_2_rads(self, enc_cms):
        prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
        rads =  prop_revolution * (2*math.pi)
        return rads

    def update(self):
        time_curr_update = rospy.Time.now()
        dt = (time_curr_update - self.time_prev_update).to_sec()

        enc1_delta = self.enc1.read_rotations() - self.enc1_data
        enc2_delta = self.enc2.read_rotations() - self.enc2_data
        enc3_delta = self.enc3.read_rotations() - self.enc3_data
        enc4_delta = self.enc4.read_rotations() - self.enc4_data

        wheel_1_angular_vel = enc1_delta * 2 * math.pi / dt
        wheel_2_angular_vel = enc2_delta * 2 * math.pi / dt
        wheel_3_angular_vel = enc3_delta * 2 * math.pi / dt
        wheel_4_angular_vel = enc4_delta * 2 * math.pi / dt

        self.wheel_4_vel_publisher.publish(wheel_4_angular_vel)
        self.wheel_1_vel_publisher.publish(wheel_1_angular_vel)
        self.wheel_2_vel_publisher.publish(wheel_2_angular_vel)
        self.wheel_3_vel_publisher.publish(wheel_3_angular_vel)

        self.enc1_data = self.enc1.read_rotations()
        self.enc2_data = self.enc2.read_rotations()
        self.enc3_data = self.enc3.read_rotations()
        self.enc4_data = self.enc4.read_rotations()

        self.time_prev_update = time_curr_update

    def spin(self):
        rospy.loginfo("Start reading from encoders")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop reading from encoders")
        # Stop message
        self.wheel_1_vel_publisher.publish(0)
        self.wheel_2_vel_publisher.publish(0)
        self.wheel_3_vel_publisher.publish(0)
        self.wheel_4_vel_publisher.publish(0)
        rospy.sleep(1)

def main():
  encoder_publisher = WheelsEncodersPublishers()
  encoder_publisher.spin()

if __name__ == '__main__':
  main(); 


