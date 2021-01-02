#!/usr/bin/env python

import pigpio
import time
import math
import OneEuroFilter

class decoder:

# Class to decode mechanical rotary encoder pulses.

    def __init__(self, pi, gpioA, gpioB):

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.counter = 0
        self.gear_ratio = 3.6
        self.pulses_per_rotation = 20
        self.enc_poll_size = 5
        self.enc_poll_cutoff_high = 2
        self.enc_poll_cutoff_low = 1
        self.enc_poll = [0]*self.enc_poll_size
        self.previous_filtered_rotations = 0
        self.previous_time = time.time()

        self.levA = 0
        self.levB = 0

        self.lastGpio = None

        self.pi.set_mode(gpioA, pigpio.INPUT)
        self.pi.set_mode(gpioB, pigpio.INPUT)

        self.pi.set_pull_up_down(gpioA, pigpio.PUD_UP)
        self.pi.set_pull_up_down(gpioB, pigpio.PUD_UP)

        self.cbA = self.pi.callback(gpioA, pigpio.EITHER_EDGE, self._pulse)
        self.cbB = self.pi.callback(gpioB, pigpio.EITHER_EDGE, self._pulse)

    def _pulse(self, gpio, level, tick):
        if gpio == self.gpioA:
            self.levA = level
        else:
            self.levB = level

        self.lastGpio = gpio

        if gpio == self.gpioA:
            if self.levA == self.levB:
                self.counter += 1
            else:
                self.counter -= 1
        elif gpio == self.gpioB:
            if self.levA == self.levB:
                self.counter -= 1
            else:
                self.counter += 1

    def read_rotations(self):
        return (self.counter / self.gear_ratio / self.pulses_per_rotation)

    def rotate_enc_poll(self):
        # print(10*"%.3f, " % tuple(self.enc_poll))
        self.enc_poll.append(self.enc_poll.pop(0))
        # print(10*"%.3f, " % tuple(self.enc_poll))
        self.enc_poll[-1] = self.read_rotations()
        # print(10*"%.3f, " % tuple(self.enc_poll))
        # print('-----')

    def read_filtered_rotations(self):
        self.rotate_enc_poll()
        temp_poll = self.enc_poll.copy()
        temp_poll.sort()
        temp_poll_sum = sum(temp_poll[self.enc_poll_cutoff_low:-self.enc_poll_cutoff_high])
        temp_poll_len = len(temp_poll[self.enc_poll_cutoff_low:-self.enc_poll_cutoff_high])
        temp_poll_average = temp_poll_sum / temp_poll_len
        return temp_poll_average

    def read_vel(self):
        t = time.time()
        filtered_rotations = self.read_filtered_rotations()
        delta = filtered_rotations - self.previous_filtered_rotations
        self.previous_filtered_rotations = filtered_rotations
        dt = t - self.previous_time
        self.previous_time = t
        return delta * 2 * math.pi / dt

    def cancel(self):
        """
        Cancel the rotary encoder decoder.
        """
        self.cbA.cancel()
        self.cbB.cancel()
