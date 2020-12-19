#!/usr/bin/env python

import pigpio

class decoder:

# Class to decode mechanical rotary encoder pulses.

    def __init__(self, pi, gpioA, gpioB):

        self.pi = pi
        self.gpioA = gpioA
        self.gpioB = gpioB
        self.counter = 0
        self.gear_ratio = 3.6
        self.pulses_per_rotation = 20

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

    def cancel(self):

        """
        Cancel the rotary encoder decoder.
        """

        self.cbA.cancel()
        self.cbB.cancel()
