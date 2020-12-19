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
        self.wake()

    def set_pins(self):
        self.pins["DIR"] = DigitalOutputDevice(self.pins["DIR"], active_high=False)
        self.pins["SLP"] = MCP18.get_pin(self.pins["SLP"])
        self.pins["SLP"].pull = digitalio.Pull.UP
        self.pins["SLP"].switch_to_output(False)

    def sleep(self):
        self.pins["SLP"].switch_to_output(False)

    def wake(self):
        self.pins["SLP"].switch_to_output(True)

    def forward(self):
        self.pins["DIR"].on()

    def backward(self):
        self.pins["DIR"].off()

    def coast(self):
        self.sleep()


class ControlMotors:
    def __init__(self):
        rospy.init_node("motors_controller")
        self.pi = pigpio.pi(show_errors=True)
        self.pwm = wavePWM.PWM(self.pi)
        self.pwm.set_frequency(10000)
        self.pid_on = True
        self.turning_off = False

        self.motor1 = LLC_motor(name="motor1")
        self.motor2 = LLC_motor(name="motor2")
        self.motor3 = LLC_motor(name="motor3")
        self.motor4 = LLC_motor(name="motor4")

        self.rate = rospy.get_param("~rate", 40)
        self.Kp = rospy.get_param('~Kp', 0.8) # 0.8
        self.Ki = rospy.get_param('~Ki', 0.0)
        self.Kd = rospy.get_param('~Kd', 0.8) # 0.8
        self.R = rospy.get_param('~robot_wheel_radius', 0.09)

        self.last_control_signal = 0
        self.slew_rate = 0.001
        self.saturation = 0.7

        # Read in encoders for PID control
        self.wheel1_angular_vel_enc_sub = rospy.Subscriber('wheel_1_vel', Float32, self.wheel1_angular_vel_enc_callback)
        self.wheel2_angular_vel_enc_sub = rospy.Subscriber('wheel_2_vel', Float32, self.wheel2_angular_vel_enc_callback)
        self.wheel3_angular_vel_enc_sub = rospy.Subscriber('wheel_3_vel', Float32, self.wheel3_angular_vel_enc_callback)
        self.wheel4_angular_vel_enc_sub = rospy.Subscriber('wheel_4_vel', Float32, self.wheel4_angular_vel_enc_callback)

        # Read errors
        self.wheel1_error = rospy.Publisher('wheel1_error', Float32, queue_size=1)
        self.wheel2_error = rospy.Publisher('wheel2_error', Float32, queue_size=1)
        self.wheel3_error = rospy.Publisher('wheel3_error', Float32, queue_size=1)
        self.wheel4_error = rospy.Publisher('wheel4_error', Float32, queue_size=1)

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
        self.wheel1_angular_vel_target_pub = rospy.Publisher("wheel1_calc_angular_vel", Float32, queue_size=1)
        self.wheel2_angular_vel_target_pub = rospy.Publisher("wheel2_calc_angular_vel", Float32, queue_size=1)
        self.wheel3_angular_vel_target_pub = rospy.Publisher("wheel3_calc_angular_vel", Float32, queue_size=1)
        self.wheel4_angular_vel_target_pub = rospy.Publisher("wheel4_calc_angular_vel", Float32, queue_size=1)

        # Tangential velocity target
        self.wheel1_tangent_vel_target = 2.0
        self.wheel2_tangent_vel_target = 1.0
        self.wheel3_tangent_vel_target = 0.5
        self.wheel4_tangent_vel_target = 4.5

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
            self.motor1.wake()
            if -1 < pwm_width1 < 1:
                if pwm_width1 >= 0:
                    self.motor1.forward()
                else:
                    self.motor1.backward()
                    pwm_width1 = -1.0 * pwm_width1
                self.pwm.set_pulse_length_in_fraction(self.motor1.pins["PWM"], pwm_width1)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")

        # motor2
        if pwm_width2 != self.pwm2_old:
            self.motor2.wake()
            if -1 < pwm_width2 < 1:
                if pwm_width2 >= 0:
                    self.motor2.forward()
                else:
                    self.motor2.backward()
                    pwm_width2 = -1.0 * pwm_width2
                self.pwm.set_pulse_length_in_fraction(self.motor2.pins["PWM"], pwm_width2)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")

        # motor3
        if pwm_width3 != self.pwm3_old:
            self.motor3.wake()
            if -1 < pwm_width3 < 1:
                if pwm_width3 >= 0:
                    self.motor3.forward()
                else:
                    self.motor3.backward()
                    pwm_width3 = -1.0 * pwm_width3
                self.pwm.set_pulse_length_in_fraction(self.motor3.pins["PWM"], pwm_width3)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")

        # motor4
        if pwm_width4 != self.pwm4_old:
            self.motor4.wake()
            if -1 < pwm_width4 < 1:
                if pwm_width4 >= 0:
                    self.motor4.forward()
                    self.pwm4_old = pwm_width4
                else:
                    self.motor4.backward()
                    pwm_width4 = -1.0 * pwm_width4
                self.pwm.set_pulse_length_in_fraction(self.motor4.pins["PWM"], pwm_width4)
            else:
                ValueError("Wartosc sygnalu z poza dozwolonego zakresu! --> <-1, 1>")

        if (self.pwm1_old != pwm_width1) or (self.pwm2_old != pwm_width2) or (self.pwm3_old != pwm_width3) or (self.pwm4_old != pwm_width4):
            self.pwm.update()
        # self.pwm.update()
        self.pwm1_old = pwm_width1
        self.pwm2_old = pwm_width2
        self.pwm3_old = pwm_width3
        self.pwm4_old = pwm_width4

    def pid_control(self, wheel_pid, target, state):
        if len(wheel_pid) == 0:
            wheel_pid.update({'time_prev': rospy.Time.now(), 'derivative': 0, 'integral': 0, 'error_prev': 0,
                              'error_curr': 0, 'control_prev': 0})

        wheel_pid['time_curr'] = rospy.Time.now()

        # PID control
        wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
        if wheel_pid['dt'] == 0: return 0

        wheel_pid['error_curr'] = target - state
        wheel_pid['integral'] = wheel_pid['integral'] + (wheel_pid['error_curr'] * wheel_pid['dt'])
        wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev']) / wheel_pid['dt']

        wheel_pid['error_prev'] = wheel_pid['error_curr']
        control_signal = (
                self.Kp * wheel_pid['error_curr'] + self.Ki * wheel_pid['integral'] + self.Kd * wheel_pid[
            'derivative'])

        # if target == 0:  # Not moving
        #     control_signal = 0
        #     return control_signal

        if abs(control_signal - wheel_pid['control_prev']) > self.slew_rate:
            if control_signal > wheel_pid['control_prev']:
                control_signal = wheel_pid['control_prev'] + self.slew_rate
            elif control_signal < wheel_pid['control_prev']:
                control_signal = wheel_pid['control_prev'] - self.slew_rate

        if control_signal >= self.saturation:
            control_signal = self.saturation
        elif control_signal <= -self.saturation:
            control_signal = -self.saturation

        wheel_pid['time_prev'] = wheel_pid['time_curr']
        control_signal = round(control_signal, 3)
        wheel_pid['control_prev'] = control_signal
        return control_signal

    def wheels_update(self):
        self.wheel1_angular_vel_target = self.tangentvel_2_angularvel(self.wheel1_tangent_vel_target)
        self.wheel1_angular_vel_target_pub.publish(self.wheel1_angular_vel_target)

        self.wheel2_angular_vel_target = self.tangentvel_2_angularvel(self.wheel2_tangent_vel_target)
        self.wheel2_angular_vel_target_pub.publish(self.wheel2_angular_vel_target)

        self.wheel3_angular_vel_target = self.tangentvel_2_angularvel(self.wheel3_tangent_vel_target)
        self.wheel3_angular_vel_target_pub.publish(self.wheel3_angular_vel_target)
        # print("nie ustawiono")
        self.wheel4_angular_vel_target = self.tangentvel_2_angularvel(self.wheel4_tangent_vel_target)
        self.wheel4_angular_vel_target_pub.publish(self.wheel4_angular_vel_target)

        self.wheel1_error.publish(self.wheel1_angular_vel_target-self.wheel1_angular_vel_enc)
        self.wheel2_error.publish(self.wheel2_angular_vel_target-self.wheel2_angular_vel_enc)
        self.wheel3_error.publish(self.wheel3_angular_vel_target-self.wheel3_angular_vel_enc)
        self.wheel4_error.publish(self.wheel4_angular_vel_target-self.wheel4_angular_vel_enc)

        if self.pid_on:
            wheel1_motor_cmd = self.pid_control(self.wheel1_pid, self.wheel1_angular_vel_target,
                                                self.wheel1_angular_vel_enc)
            wheel2_motor_cmd = self.pid_control(self.wheel2_pid, self.wheel2_angular_vel_target,
                                                self.wheel2_angular_vel_enc)
            wheel3_motor_cmd = self.pid_control(self.wheel3_pid, self.wheel3_angular_vel_target,
                                                self.wheel3_angular_vel_enc)
            wheel4_motor_cmd = self.pid_control(self.wheel4_pid, self.wheel4_angular_vel_target,
                                                self.wheel4_angular_vel_enc)

        print("PWM1: {}, PWM2: {}, PWM3: {}, PWM4: {}".format(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd,
                                                              wheel4_motor_cmd))

        self.set_speed(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd, wheel4_motor_cmd)
        # print("ustawiono")

    def spin(self):
        rospy.loginfo("Start motors_controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown() and not self.turning_off:
            t1 = rospy.Time.now()
            self.wheels_update()
            t2 = rospy.Time.now()
            rate.sleep()
            t3 = rospy.Time.now()
            A = (1000+(t2.nsecs-t1.nsecs)/(10**6))%1000
            S = (1000+(t3.nsecs-t2.nsecs)/(10**6))%1000
            T = (1000+(t3.nsecs-t1.nsecs)/(10**6))%1000
            print("Act: %.3f, Slp: %.3f, Tot: %.3f" % (A, S, T))

        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop motors_controller")
        self.turning_off = True
        # Stop message
        print("setting 0 speed on shutdown")
        self.set_speed(0, 0, 0, 0)
        print("stopping pi object on shutdown")
        self.pi.stop()
        rospy.sleep(1)


def main():
    controls_to_motors = ControlMotors()
    controls_to_motors.spin()


if __name__ == '__main__':
    main()
