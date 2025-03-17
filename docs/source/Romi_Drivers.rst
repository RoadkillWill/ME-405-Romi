Romi_Drivers
=======================

This section provides an overview of the motor, encoder, sensor, and IMU classes used in the ROMI robot.

.. contents:: **Contents**
   :depth: 2
   :local:

Motor Class
------------
.. code-block:: python

    from pyb import Pin, Timer, ADC, I2C, delay
    from time import ticks_us, ticks_diff
    from math import pi, sqrt
    import struct

    class Motor:
        """
        A class to control a motor using PWM signals.
        """
        def __init__(self, motor_tim, PWM_Pin, DIR, nSLP):
            self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
            self.tim = Timer(motor_tim, freq=1000)
            self.PWM_pin = self.tim.channel(3, pin=PWM_Pin, mode=Timer.PWM)
            self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)

        def set_effort(self, effort):
            if effort > 0:
                self.DIR_pin.low()
            else:
                self.DIR_pin.high()
                effort = -effort
            self.PWM_pin.pulse_width_percent(effort)

        def enable(self):
            self.nSLP_pin.high()
            self.DIR_pin.low()

        def disable(self):
            self.nSLP_pin.low()

Encoder Class
-------------
.. code-block:: python

    class Encoder:
        """
        A class to handle encoder feedback.
        """
        def __init__(self, tim, chA_pin, chB_pin):
            self.position = 0
            self.prev_count = 0
            self.delta = 0
            self.dt = 1
            self.time = ticks_us()

            self.timer = Timer(tim, period=0xFFFF, prescaler=0)
            self.chA = Pin(chA_pin, mode=Pin.IN)
            self.chB = Pin(chB_pin, mode=Pin.IN)
            self.timer_channelA = self.timer.channel(1, mode=Timer.ENC_AB, pin=chA_pin)
            self.timer_channelB = self.timer.channel(2, mode=Timer.ENC_AB, pin=chB_pin)
            self.timer.counter(0)

        def update(self):
            self.dt = max(ticks_diff(ticks_us(), self.time), 1)
            self.time = ticks_us()
            self.count = self.timer.counter()
            self.AR = 65535
            self.delta = self.count - self.prev_count

            if self.delta > (self.AR + 1) / 2:
                self.delta -= (self.AR + 1)
            elif self.delta < -(self.AR + 1) / 2:
                self.delta += (self.AR + 1)

            self.prev_count = self.count
            self.position += self.delta

        def get_position(self):
            return abs(self.position)

        def get_velocity(self):
            return abs(self.delta / self.dt)

        def zero(self):
            self.prev_count = 0
            self.position = 0
            self.delta = 0
            self.dt = 1
            self.time = ticks_us()
            self.timer.counter(0)

Sensor Array Class
-------------------
.. code-block:: python

    class Sensor_Array:
        """
        A class to handle an array of sensors.
        """
        def __init__(self):
            self.sensor_pins = [
                Pin(Pin.cpu.C5, mode=Pin.OUT_PP, value=0), 
                Pin(Pin.cpu.C0, mode=Pin.OUT_PP, value=0),
                Pin(Pin.cpu.C1, mode=Pin.OUT_PP, value=0),
                Pin(Pin.cpu.A6, mode=Pin.OUT_PP, value=0)
            ]
            self.sensor_ADC = [ADC(pin) for pin in self.sensor_pins]

        def read_sensors(self):
            for i in range(len(self.sensor_ADC)):
                raw_value = self.sensor_ADC[i].read()
                print(f"Sensor {i}: {raw_value}")

IMU (BNO055) Class
-------------------
.. code-block:: python

    class BNO055:
        """
        A class to interact with the BNO055 sensor over I2C.
        """
        BNO055_ADDRESS = 0x28
        BNO055_EULER_H_LSB_ADDR = 0x1A

        def __init__(self, i2c):
            self.i2c = i2c

        def read_heading(self):
            data = self.i2c.mem_read(6, self.BNO055_ADDRESS, self.BNO055_EULER_H_LSB_ADDR)
            if data:
                heading, _, _ = struct.unpack('<hhh', data)
                return heading / 16.0
            return None

Bumper Class
-------------------
.. code-block:: python

    class Bumper:
        """
        A class to handle bumper sensors.
        """
        def __init__(self):
            self.left_bumper_pins = [
                Pin(Pin.cpu.C11, mode=Pin.IN, pull=Pin.PULL_UP),
                Pin(Pin.cpu.C10, mode=Pin.IN, pull=Pin.PULL_UP)
            ]
            self.right_bumper_pins = [
                Pin(Pin.cpu.B2, mode=Pin.IN, pull=Pin.PULL_UP),
                Pin(Pin.cpu.B3, mode=Pin.IN, pull=Pin.PULL_UP)
            ]

        def check_bump(self):
            left_values = [pin.value() for pin in self.left_bumper_pins]
            right_values = [pin.value() for pin in self.right_bumper_pins]
            return any(value == 0 for value in left_values + right_values)
