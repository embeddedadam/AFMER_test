#!/usr/bin/python3
import rospy
import roslib
import board
import busio
from gpiozero import DigitalOutputDevice
from digitalio import Direction
from adafruit_mcp230xx.mcp23017 import MCP23017
from gpiozero import MCP3208
import digitalio
import pigpio
from std_msgs.msg import Float32
import wavePWM

i2c = busio.I2C(board.SCL, board.SDA)
MCP18 = MCP23017(i2c)

'''
Low level motor control class.
Provide pinout numbering for each instance of LLC_motor class.
'''


class LLC_motor:
    def __init__(self, name):
        self.name = name
        """
                        DIR|PWM|SLP|FLT|CS|PWR
        """
        self.motors = {
            "motor1": (19, 20, 8, 7, 6, 9),
            "motor2": (12, 6, 10, 6, 5, 11),
            "motor3": (23, 22, 12, 5, 7, 13),
            "motor4": (17, 4, 14, 4, 4, 15)
        }

        self.pins = {
            "DIR": self.motors[name][0],
            "PWM": self.motors[name][1],
            "SLP": self.motors[name][2],
            "FLT": self.motors[name][3],
            "CS": self.motors[name][4],
            "PWR": self.motors[name][5]
        }

        self.set_pins()

    def set_pins(self):
        self.pins["DIR"] = DigitalOutputDevice(self.pins["DIR"], active_high=False)
        self.pins["SLP"] = MCP18.get_pin(self.pins["SLP"])
        self.pins["FLT"] = MCP18.get_pin(self.pins["FLT"])
        #self.pins["CS"] = MCP3208(self.pins["CS"])
        self.pins["PWR"] = MCP18.get_pin(self.pins["PWR"])

        self.pins["SLP"].pull = digitalio.Pull.UP
        self.pins["SLP"].switch_to_output(False)
        self.pins["PWR"].switch_to_input()

    def sleep(self):
        self.pins["SLP"].switch_to_output(False)

    def wake(self):
        self.pins["SLP"].switch_to_output(True)

    def is_active(self):
        return self.pins["PWR"].value

    def forward(self):
        self.pins["DIR"].on

    def backward(self):
        self.pins["DIR"].off

    def coast(self):
        self.sleep()

    def brake(self):
        self.pwm.set_pulse_length_in_fraction(self.pins["PWM"], 0)
        self.pwm.update()


class ControlMotors:
    def __init__(self):
        rospy.init_node("motors_controller")
        self.pi = pigpio.pi()
        self.pwm = wavePWM.PWM(self.pi)
        self.pwm.set_frequency(12500)
        self.pid_on = True

        self.motor1 = LLC_motor(name="motor1")
        self.motor2 = LLC_motor(name="motor2")
        self.motor3 = LLC_motor(name="motor3")
        self.motor4 = LLC_motor(name="motor4")

        self.rate = rospy.get_param("~rate", 10)
        self.Kp = rospy.get_param('~Kp', 1.0)
        self.Ki = rospy.get_param('~Ki', 1.0)
        self.Kd = rospy.get_param('~Kd', 1.0)
        self.R = rospy.get_param('~robot_wheel_radius', 0.09)
        self.motor_max_angular_vel = rospy.get_param('~motor_max_angular_vel', 87.0)
        self.motor_min_angular_vel = rospy.get_param('~motor_min_angular_vel', -87.0)

        # Read in encoders for PID control
        self.wheel1_angular_vel_enc_sub = rospy.Subscriber('wheel_1_vel', Float32, self.wheel1_angular_vel_enc_callback)
        self.wheel2_angular_vel_enc_sub = rospy.Subscriber('wheel_2_vel', Float32, self.wheel2_angular_vel_enc_callback)
        self.wheel3_angular_vel_enc_sub = rospy.Subscriber('wheel_3_vel', Float32, self.wheel3_angular_vel_enc_callback)
        self.wheel4_angular_vel_enc_sub = rospy.Subscriber('wheel_4_vel', Float32, self.wheel4_angular_vel_enc_callback)

        # Read in tangential velocity targets
        self.wheel1_tangent_vel_target_sub = rospy.Subscriber('wheel1_tangent_vel_target', Float32,
                                                              self.wheel1_tangent_vel_target_callback)
        self.wheel2_tangent_vel_target_sub = rospy.Subscriber('wheel2_tangent_vel_target', Float32,
                                                              self.wheel2_tangent_vel_target_callback)
        self.wheel3_tangent_vel_target_sub = rospy.Subscriber('wheel3_tangent_vel_target', Float32,
                                                              self.wheel3_tangent_vel_target_callback)
        self.wheel4_tangent_vel_target_sub = rospy.Subscriber('wheel4_tangent_vel_target', Float32,
                                                              self.wheel4_tangent_vel_target_callback)

        # Publish computed values
        self.wheel1_angular_vel_target_pub = rospy.Publisher("wheel1_calc_angular_vel", Float32, queue_size=10)
        self.wheel2_angular_vel_target_pub = rospy.Publisher("wheel2_calc_angular_vel", Float32, queue_size=10)
        self.wheel3_angular_vel_target_pub = rospy.Publisher("wheel3_calc_angular_vel", Float32, queue_size=10)
        self.wheel4_angular_vel_target_pub = rospy.Publisher("wheel4_calc_angular_vel", Float32, queue_size=10)
        # Tangential velocity target
        self.wheel1_tangent_vel_target = 0
        self.wheel2_tangent_vel_target = 0.2
        self.wheel3_tangent_vel_target = 0
        self.wheel4_tangent_vel_target = 0

        # Angular velocity target
        self.wheel1_angular_vel_target = 0
        self.wheel2_angular_vel_target = 0
        self.wheel3_angular_vel_target = 0
        self.wheel4_angular_vel_target = 0

        # Angular velocity encoder readings
        self.wheel1_angular_vel_enc = 0
        self.wheel2_angular_vel_enc = 0
        self.wheel3_angular_vel_enc = 0
        self.wheel4_angular_vel_enc = 0

        # PID control variables
        self.wheel1_pid = {}
        self.wheel2_pid = {}
        self.wheel3_pid = {}
        self.wheel4_pid = {}

        self.pwm1_old = 0
        self.pwm2_old = 0
        self.pwm3_old = 0
        self.pwm4_old = 0

    def wheel1_angular_vel_enc_callback(self, msg):
        self.wheel1_angular_vel_enc = msg.data

    def wheel2_angular_vel_enc_callback(self, msg):
        self.wheel2_angular_vel_enc = msg.data

    def wheel3_angular_vel_enc_callback(self, msg):
        self.wheel3_angular_vel_enc = msg.data

    def wheel4_angular_vel_enc_callback(self, msg):
        self.wheel4_angular_vel_enc = msg.data

    def wheel1_tangent_vel_target_callback(self, msg):
        self.wheel1_tangent_vel_target = msg.data

    def wheel2_tangent_vel_target_callback(self, msg):
        self.wheel2_tangent_vel_target = msg.data

    def wheel3_tangent_vel_target_callback(self, msg):
        self.wheel3_tangent_vel_target = msg.data

    def wheel4_tangent_vel_target_callback(self, msg):
        self.wheel4_tangent_vel_target = msg.data

    # Compute angular velocity target
    def tangentvel_2_angularvel(self, tangent_vel):
        # v = wr
        # v - tangential velocity (m/s)
        # w - angular velocity (rad/s)
        # r - radius of wheel (m)
        angular_vel = tangent_vel / self.R
        return angular_vel

    def set_speed(self, pwm_width1, pwm_width2, pwm_width3, pwm_width4):
        # motor1
        if pwm_width1 != self.pwm1_old:
            if not self.motor1.is_active():
                self.motor1.wake()
                if not self.motor1.is_active():
                    ValueError("Sterownik nie chce wstac")
            if (-1 < pwm_width1 < 1):
                if pwm_width1 >= 0:
                    self.motor1.forward()
                else:
                    pwm_width1 = -1.0 * pwm_width1
                    self.motor1.backward()
                self.pwm.set_pulse_length_in_fraction(self.motor1.pins["PWM"], pwm_width1)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")
        else:
            pass

        # motor2
        if pwm_width2 != self.pwm2_old:
            if not self.motor2.is_active():
                self.motor2.wake()
                if not self.motor2.is_active():
                    ValueError("Sterownik nie chce wstac")
            if (-1 < pwm_width2 < 1):
                if pwm_width2 >= 0:
                    self.motor2.forward()
                else:
                    pwm_width2 = -1.0 * pwm_width2
                    self.motor2.backward()
                self.pwm.set_pulse_length_in_fraction(self.motor2.pins["PWM"], pwm_width2)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")
        else:
            pass

        # motor3
        if pwm_width3 != self.pwm3_old:
            if not self.motor3.is_active():
                self.motor3.wake()
                if not self.motor3.is_active():
                    ValueError("Sterownik nie chce wstac")
            if (-1 < pwm_width3 < 1):
                if pwm_width3 >= 0:
                    self.motor3.forward()
                else:
                    pwm_width3 = -1.0 * pwm_width3
                    self.motor3.backward()
                self.pwm.set_pulse_length_in_fraction(self.motor3.pins["PWM"], pwm_width3)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")
        else:
            pass

        # motor4
        if pwm_width4 != self.pwm4_old:
            if not self.motor4.is_active():
                self.motor4.wake()
                if not self.motor4.is_active():
                    ValueError("Sterownik nie chce wstac")
            if (-1 < pwm_width4 < 1):
                if pwm_width4 >= 0:
                    self.motor4.forward()
                else:
                    pwm_width4 = -1.0 * pwm_width4
                    self.motor4.backward()
                self.pwm.set_pulse_length_in_fraction(self.motor4.pins["PWM"], pwm_width4)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")
        else:
            pass

        self.pwm.update()
        self.pwm1_old = pwm_width1
        self.pwm2_old = pwm_width2
        self.pwm3_old = pwm_width3
        self.pwm4_old = pwm_width4

    def pid_control(self, wheel_pid, target, state):
        if len(wheel_pid) == 0:
            wheel_pid.update({'time_prev': rospy.Time.now(), 'derivative': 0, 'integral': [0] * 10, 'error_prev': 0,
                              'error_curr': 0})

        wheel_pid['time_curr'] = rospy.Time.now()

        # PID control
        wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
        if wheel_pid['dt'] == 0: return 0

        wheel_pid['error_curr'] = target - state
        wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr'] * wheel_pid['dt'])]
        wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev']) / wheel_pid['dt']

        wheel_pid['error_prev'] = wheel_pid['error_curr']
        control_signal = (
                    self.Kp * wheel_pid['error_curr'] + self.Ki * sum(wheel_pid['integral']) + self.Kd * wheel_pid[
                'derivative'])
        target_new = target + control_signal

        # Ensure control_signal does not flip sign of target velocity
        if target > 0 and target_new < 0: target_new = target
        if target < 0 and target_new > 0: target_new = target

        if (target == 0):  # Not moving
            target_new = 0
            return target_new

        wheel_pid['time_prev'] = wheel_pid['time_curr']
        return target_new

    def wheels_update(self):
        self.wheel1_angular_vel_target = self.tangentvel_2_angularvel(self.wheel1_tangent_vel_target)
        self.wheel1_angular_vel_target_pub.publish(self.wheel1_angular_vel_target)

        self.wheel2_angular_vel_target = self.tangentvel_2_angularvel(self.wheel2_tangent_vel_target)
        self.wheel2_angular_vel_target_pub.publish(self.wheel2_angular_vel_target)

        self.wheel3_angular_vel_target = self.tangentvel_2_angularvel(self.wheel3_tangent_vel_target)
        self.wheel3_angular_vel_target_pub.publish(self.wheel3_angular_vel_target)

        self.wheel4_angular_vel_target = self.tangentvel_2_angularvel(self.wheel4_tangent_vel_target)
        self.wheel4_angular_vel_target_pub.publish(self.wheel4_angular_vel_target)

        if self.pid_on:
            self.wheel1_angular_vel_target = self.pid_control(self.wheel1_pid, self.wheel1_angular_vel_target,
                                                              self.wheel1_angular_vel_enc)
            self.wheel2_angular_vel_target = self.pid_control(self.wheel2_pid, self.wheel2_angular_vel_target,
                                                              self.wheel2_angular_vel_enc)
            self.wheel3_angular_vel_target = self.pid_control(self.wheel3_pid, self.wheel3_angular_vel_target,
                                                              self.wheel3_angular_vel_enc)
            self.wheel4_angular_vel_target = self.pid_control(self.wheel4_pid, self.wheel4_angular_vel_target,
                                                              self.wheel4_angular_vel_enc)

        # Compute motor command
        wheel1_motor_cmd = self.angularvel_2_motorcmd(self.wheel1_angular_vel_target)
        wheel2_motor_cmd = self.angularvel_2_motorcmd(self.wheel2_angular_vel_target)
        wheel3_motor_cmd = self.angularvel_2_motorcmd(self.wheel3_angular_vel_target)
        wheel4_motor_cmd = self.angularvel_2_motorcmd(self.wheel4_angular_vel_target)

        print("PWM1: {}, PWM2: {}, PWM3: {}, PWM4: {}".format(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd,
                                                              wheel4_motor_cmd))
        self.set_speed(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd, wheel4_motor_cmd)

    def angularvel_2_motorcmd(self, angular_vel_target):
        if angular_vel_target != 0:
            cmd = angular_vel_target/self.motor_max_angular_vel
            if -1 > cmd:
                return -1
            elif 1 < cmd:
                return 1
            else:
                return cmd
        else:
            return 0

    def spin(self):
        rospy.loginfo("Start motors_controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            self.wheels_update()
            rate.sleep()
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop motors_controller")
        # Stop message
        self.set_speed(0, 0, 0, 0)
        self.pi.stop()
        rospy.sleep(1)


def main():
    controls_to_motors = ControlMotors()
    controls_to_motors.spin()


if __name__ == '__main__':
    main();
