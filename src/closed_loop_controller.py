#!/usr/bin/python3
import rospy
import roslib
import math # tylko dla math.pi
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
import rotary_encoder

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
            "motor1": (19, 20, 12, 7, 6, 8),
            "motor2": (12, 6, 13, 6, 5, 9),
            "motor3": (23, 22, 14, 5, 7, 10),
            "motor4": (17, 4, 15, 4, 4, 11)
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


class closed_loop_controller:
    def __init__(self):
        rospy.init_node("closed_loop_controller")
        self.pi = pigpio.pi(show_errors=True)
        self.pwm = wavePWM.PWM(self.pi)
        self.pwm.set_frequency(20000)
        self.pid_on = True
        self.turning_off = False

        self.encoder1 = rotary_encoder.decoder(self.pi, 21, 26)
        self.encoder2 = rotary_encoder.decoder(self.pi, 13, 16)
        self.encoder3 = rotary_encoder.decoder(self.pi, 5, 25)
        self.encoder4 = rotary_encoder.decoder(self.pi, 18, 27)

        self.wheel_1_vel_publisher = rospy.Publisher("wheel_1_vel", Float32, queue_size=10)
        self.wheel_2_vel_publisher = rospy.Publisher("wheel_2_vel", Float32, queue_size=10)
        self.wheel_3_vel_publisher = rospy.Publisher("wheel_3_vel", Float32, queue_size=10)
        self.wheel_4_vel_publisher = rospy.Publisher("wheel_4_vel", Float32, queue_size=10)

        self.motor1 = LLC_motor(name="motor1")
        self.motor2 = LLC_motor(name="motor2")
        self.motor3 = LLC_motor(name="motor3")
        self.motor4 = LLC_motor(name="motor4")

        self.motor1.wake()
        self.motor2.wake()
        self.motor3.wake()
        self.motor4.wake()

        self.rate = rospy.get_param("~rate", 20)
        self.Kp = rospy.get_param('~Kp', 0.001) # 0.001
        self.Ki = rospy.get_param('~Ki', 0.01) # 0.02
        self.Kd = rospy.get_param('~Kd', 0.0005) # 0.001
        Ts = 1 / self.rate
        Ti = self.Kp * Ts / self.Ki
        Td = self.Kd * Ts / self.Kp
        Kb = 1 / math.sqrt(Ti*Td)
        self.Kb = rospy.get_param('~Kb', Kb)
        self.R = rospy.get_param('~robot_wheel_radius', 0.09)

        self.last_control_signal = 0
        self.slew_rate = 0.2/self.rate
        self.saturation = 0.9

        # Read errors
        self.wheel1_error = rospy.Publisher('wheel_1_error', Float32, queue_size=1)
        self.wheel2_error = rospy.Publisher('wheel_2_error', Float32, queue_size=1)
        self.wheel3_error = rospy.Publisher('wheel_3_error', Float32, queue_size=1)
        self.wheel4_error = rospy.Publisher('wheel_4_error', Float32, queue_size=1)

        # Read in tangential velocity targets
        self.wheel1_tangent_vel_target_sub = rospy.Subscriber('wheel_1_tangent_vel_target', Float32,
                                                              self.wheel1_tangent_vel_target_callback)
        self.wheel2_tangent_vel_target_sub = rospy.Subscriber('wheel_2_tangent_vel_target', Float32,
                                                              self.wheel2_tangent_vel_target_callback)
        self.wheel3_tangent_vel_target_sub = rospy.Subscriber('wheel_3_tangent_vel_target', Float32,
                                                              self.wheel3_tangent_vel_target_callback)
        self.wheel4_tangent_vel_target_sub = rospy.Subscriber('wheel_4_tangent_vel_target', Float32,
                                                              self.wheel4_tangent_vel_target_callback)

        # Publish computed values
        self.wheel1_angular_vel_target_pub = rospy.Publisher("wheel_1_calc_angular_vel", Float32, queue_size=1)
        self.wheel2_angular_vel_target_pub = rospy.Publisher("wheel_2_calc_angular_vel", Float32, queue_size=1)
        self.wheel3_angular_vel_target_pub = rospy.Publisher("wheel_3_calc_angular_vel", Float32, queue_size=1)
        self.wheel4_angular_vel_target_pub = rospy.Publisher("wheel_4_calc_angular_vel", Float32, queue_size=1)

        # Tangential velocity target
        self.wheel1_tangent_vel_target = 0
        self.wheel2_tangent_vel_target = 0
        self.wheel3_tangent_vel_target = 0
        self.wheel4_tangent_vel_target = 0

        # PID control variables
        self.wheel1_pid = {}
        self.wheel2_pid = {}
        self.wheel3_pid = {}
        self.wheel4_pid = {}

        self.enc1_prev_rotations = 0
        self.enc2_prev_rotations = 0
        self.enc3_prev_rotations = 0
        self.enc4_prev_rotations = 0

        self.pwm1_old = 0
        self.pwm2_old = 0
        self.pwm3_old = 0
        self.pwm4_old = 0

    def wheel1_tangent_vel_target_callback(self, msg):
        self.wheel1_tangent_vel_target = msg.data

    def wheel2_tangent_vel_target_callback(self, msg):
        self.wheel2_tangent_vel_target = msg.data

    def wheel3_tangent_vel_target_callback(self, msg):
        self.wheel3_tangent_vel_target = msg.data

    def wheel4_tangent_vel_target_callback(self, msg):
        self.wheel4_tangent_vel_target = msg.data

    def tangentvel_2_angularvel(self, tangent_vel):
        # v = wr
        # v - tangential velocity (m/s)
        # w - angular velocity (rad/s)
        # r - radius of wheel (m)
        angular_vel = tangent_vel / self.R
        # print("-----ang_vel: {}, tang_vel: {}, R: {}-----------".format(angular_vel, tangent_vel, self.R))
        return angular_vel

    def set_speed(self, pwm_width1, pwm_width2, pwm_width3, pwm_width4):
        # motor1
        if pwm_width1 != self.pwm1_old:
            # self.motor1.wake()
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
            # self.motor2.wake()
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
            # self.motor3.wake()
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
            # self.motor4.wake()
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

        # if (self.pwm1_old != pwm_width1) or (self.pwm2_old != pwm_width2) or (self.pwm3_old != pwm_width3) or (self.pwm4_old != pwm_width4):
        #     self.pwm.update()
        self.pwm.update()
        # print(pwm_width2)
        self.pwm1_old = pwm_width1
        self.pwm2_old = pwm_width2
        self.pwm3_old = pwm_width3
        self.pwm4_old = pwm_width4

    def pid_control(self, wheel_pid, SP, PV):
        if len(wheel_pid) == 0:
            wheel_pid.update({'time_last': rospy.Time.now(), 'integral': 0, 'e_last': 0, 'control_last': 0})

        time = rospy.Time.now()
        CV_last = wheel_pid['control_last']

        # PID control
        dt = (time - wheel_pid['time_last']).to_sec()
        # print(dt)
        if dt == 0:
            return 0

        e = SP - PV
        
        wheel_pid['integral'] = wheel_pid['integral'] + (e * dt)

        P = self.Kp * e
        I = self.Ki * wheel_pid['integral']
        D = self.Kd * (e - wheel_pid['e_last']) / dt
        
        wheel_pid['e_last'] = e
        
        CV = P + I + D

        if SP == 0: # and (-0.1<e<0.1):  # Not moving
            CV = 0
            wheel_pid['integral'] = 0
        else:
            if abs(CV - CV_last) > self.slew_rate:
                if CV > CV_last:
                    CV = CV_last + self.slew_rate
                elif CV < CV_last:
                    CV = CV_last - self.slew_rate

            CV = round(CV, 3)

            if CV >= self.saturation:
                CV_final = self.saturation
            elif CV <= -self.saturation:
                CV_final = -self.saturation

            AW = CV_final - CV
            wheel_pid['integral'] = wheel_pid['integral'] - AW * self.Kb

        wheel_pid['control_last'] = CV_final
        wheel_pid['time_last'] = time
        return CV

    def wheels_update(self):
        wheel_1_angular_vel = self.encoder1.read_vel()
        wheel_2_angular_vel = self.encoder2.read_vel()
        wheel_3_angular_vel = self.encoder3.read_vel()
        wheel_4_angular_vel = self.encoder4.read_vel()

        self.wheel_1_vel_publisher.publish(wheel_1_angular_vel)
        self.wheel_2_vel_publisher.publish(wheel_2_angular_vel)
        self.wheel_3_vel_publisher.publish(wheel_3_angular_vel)
        self.wheel_4_vel_publisher.publish(wheel_4_angular_vel)

        wheel_1_angular_vel_target = self.tangentvel_2_angularvel(self.wheel1_tangent_vel_target)
        self.wheel1_angular_vel_target_pub.publish(wheel_1_angular_vel_target)

        wheel_2_angular_vel_target = self.tangentvel_2_angularvel(self.wheel2_tangent_vel_target)
        self.wheel2_angular_vel_target_pub.publish(wheel_2_angular_vel_target)

        wheel_3_angular_vel_target = self.tangentvel_2_angularvel(self.wheel3_tangent_vel_target)
        self.wheel3_angular_vel_target_pub.publish(wheel_3_angular_vel_target)

        wheel_4_angular_vel_target = self.tangentvel_2_angularvel(self.wheel4_tangent_vel_target)
        self.wheel4_angular_vel_target_pub.publish(wheel_4_angular_vel_target)

        self.wheel1_error.publish(wheel_1_angular_vel_target-wheel_1_angular_vel)
        self.wheel2_error.publish(wheel_2_angular_vel_target-wheel_2_angular_vel)
        self.wheel3_error.publish(wheel_3_angular_vel_target-wheel_3_angular_vel)
        self.wheel4_error.publish(wheel_4_angular_vel_target-wheel_4_angular_vel)

        if self.pid_on:
            wheel1_motor_cmd = self.pid_control(self.wheel1_pid, wheel_1_angular_vel_target,
                                                wheel_1_angular_vel)
            wheel2_motor_cmd = self.pid_control(self.wheel2_pid, wheel_2_angular_vel_target,
                                                wheel_2_angular_vel)
            wheel3_motor_cmd = self.pid_control(self.wheel3_pid, wheel_3_angular_vel_target,
                                                wheel_3_angular_vel)
            wheel4_motor_cmd = self.pid_control(self.wheel4_pid, wheel_4_angular_vel_target,
                                                wheel_4_angular_vel)

        print("PWM1: {}, PWM2: {}, PWM3: {}, PWM4: {}".format(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd,
                                                              wheel4_motor_cmd))

        self.set_speed(wheel1_motor_cmd, wheel2_motor_cmd, wheel3_motor_cmd, wheel4_motor_cmd)

    def spin(self):
        rospy.loginfo("Start motors_controller")
        rate = rospy.Rate(self.rate)

        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown() and not self.turning_off:
            # t1 = rospy.Time.now()
            self.wheels_update()
            # t2 = rospy.Time.now()
            rate.sleep()
            # t3 = rospy.Time.now()
            # A = (1000+(t2.nsecs-t1.nsecs)/(10**6))%1000
            # S = (1000+(t3.nsecs-t2.nsecs)/(10**6))%1000
            # T = (1000+(t3.nsecs-t1.nsecs)/(10**6))%1000
            # print("Act: %.3f, Slp: %.3f, Tot: %.3f" % (A, S, T))

        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop motors_controller")
        self.turning_off = True
        rospy.sleep(2/self.rate)
        print("setting 0 speed on shutdown")
        self.set_speed(0, 0, 0, 0)
        # self.encoder1.cancel()
        # self.encoder2.cancel()
        # self.encoder3.cancel()
        # self.encoder4.cancel()
        print("stopping pi object on shutdown")
        self.pi.stop()
        rospy.sleep(1)


def main():
    controls_to_motors = closed_loop_controller()
    controls_to_motors.spin()


if __name__ == '__main__':
    main()
