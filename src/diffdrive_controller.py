#!/usr/bin/python
import rospy
import roslib

# Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class CmdVelToDiffDriveMotors:
    def __init__(self):
        rospy.init_node('diffdrive_controller')
        self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)

        self.wheel1_tangent_vel_target_pub = rospy.Publisher('wheel1_tangent_vel_target', Float32, queue_size=10)
        self.wheel2_tangent_vel_target_pub = rospy.Publisher('wheel2_tangent_vel_target', Float32, queue_size=10)
        self.wheel3_tangent_vel_target_pub = rospy.Publisher('wheel3_tangent_vel_target', Float32, queue_size=10)
        self.wheel4_tangent_vel_target_pub = rospy.Publisher('wheel4_tangent_vel_target', Float32, queue_size=10)

        self.L = rospy.get_param('~robot_wheel_separation_distance', 0.4319)
        self.R = rospy.get_param('~robot_wheel_radius', 0.09)

        self.rate = rospy.get_param('~rate', 50)
        self.timeout_idle = rospy.get_param('~timeout_idle', 2)
        self.time_prev_update = rospy.Time.now()

        self.target_v = 0
        self.target_w = 0

    def twistCallback(self, msg):
        self.target_v = msg.linear.x
        self.target_w = msg.angular.z
        self.time_prev_update = rospy.Time.now()

    def update(self):
        # Suppose we have a target velocity v and angular velocity w
        # Suppose we have a robot with wheel radius R and distance between wheels L
        # Let vr and vl be angular wheel velocity for right and left wheels, respectively
        # Relate 2v = R (vr +vl) because the forward speed is the sum of the combined wheel velocities
        # Relate Lw = R (vr - vl) because rotation is a function of counter-clockwise wheel speeds
        # Compute vr = (2v + wL) / 2R
        # Compute vl = (2v - wL) / 2R
        vr = self.target_v + self.target_w * (self.L / 2)
        vl = self.target_v - self.target_w * (self.L / 2)

        self.wheel1_tangent_vel_target_pub.publish(vl)
        self.wheel2_tangent_vel_target_pub.publish(vr)
        self.wheel3_tangent_vel_target_pub.publish(vl)
        self.wheel4_tangent_vel_target_pub.publish(vr)

    def shutdown(self):
        self.wheel1_tangent_vel_target_pub.publish(0)
        self.wheel2_tangent_vel_target_pub.publish(0)
        self.wheel3_tangent_vel_target_pub.publish(0)
        self.wheel4_tangent_vel_target_pub.publish(0)

    def spin(self):
        rospy.loginfo("Start diffdrive_controller")
        rate = rospy.Rate(self.rate)
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            time_diff_update = (rospy.Time.now() - self.time_prev_update).to_sec()
            if time_diff_update < self.timeout_idle:  # Only move if command given recently
                self.update()
            rate.sleep()
        rospy.spin()


def main():
    cmdvel_to_motors = CmdVelToDiffDriveMotors()
    cmdvel_to_motors.spin()


if __name__ == '__main__':
    main()
