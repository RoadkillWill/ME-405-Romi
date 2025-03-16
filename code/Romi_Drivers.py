from pyb import Pin, Timer, ADC, I2C, delay
from time import ticks_us, ticks_diff
from math import pi, sqrt
import struct


class Motor:
    """
    A class to control a motor using PWM signals.

    Attributes:
        nSLP_pin (Pin): The sleep pin.
        tim (Timer): The timer object.
        PWM_pin (Timer.channel): The PWM pin.
        DIR_pin (Pin): The direction pin.
    """

    def __init__(self, motor_tim, PWM_Pin, DIR, nSLP):
        """
        Initializes the Motor class.

        Args:
            motor_tim (int): The timer number.
            PWM_Pin (int): The pin number for PWM.
            DIR (int): The direction pin number.
            nSLP (int): The sleep pin number.
        """
        self.nSLP_pin = Pin(nSLP, mode=Pin.OUT_PP, value=0)
        self.tim = Timer(motor_tim, freq=1000)
        self.PWM_pin = self.tim.channel(3, pin=PWM_Pin, mode=Timer.PWM)
        self.DIR_pin = Pin(DIR, mode=Pin.OUT_PP, value=0)

    def set_effort(self, effort):
        """
        Sets the effort (speed) of the motor.

        Args:
            effort (int): The effort level (-100 to 100).
        """
        if effort > 0:
            self.DIR_pin.low()
        else:
            self.DIR_pin.high()
            effort = -effort
        self.PWM_pin.pulse_width_percent(effort)

    def enable(self):
        """
        Enables the motor by setting the sleep pin high and direction pin low.
        """
        self.nSLP_pin.high()
        self.DIR_pin.low()

    def disable(self):
        """
        Disables the motor by setting the sleep pin low.
        """
        self.nSLP_pin.low()


class Encoder:
    """
    A class to handle encoder feedback.

    Attributes:
        position (int): The current position.
        prev_count (int): The previous counter value.
        delta (int): The change in position.
        dt (int): The time difference.
        time (int): The current time.
        timer (Timer): The timer object.
        chA (Pin): The channel A pin.
        chB (Pin): The channel B pin.
        timer_channelA (Timer.channel): The timer channel for A.
        timer_channelB (Timer.channel): The timer channel for B.
    """

    def __init__(self, tim, chA_pin, chB_pin):
        """
        Initializes the Encoder class.

        Args:
            tim (int): The timer number.
            chA_pin (int): The channel A pin number.
            chB_pin (int): The channel B pin number.
        """
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
        """
        Updates the encoder reading by calculating the difference in encoder counts
        and updating the position and time.
        """
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
        """
        Gets the current position.

        Returns:
            int: The current position.
        """
        return abs(self.position)

    def get_velocity(self):
        """
        Gets the current velocity.

        Returns:
            float: The current velocity.
        """
        return abs(self.delta / self.dt)

    def zero(self):
        """
        Resets the encoder values to zero.
        """
        self.prev_count = 0
        self.position = 0
        self.delta = 0
        self.dt = 1
        self.time = ticks_us()
        self.timer.counter(0)


class Collector:
    """
    A class to collect data from the encoder.

    Attributes:
        encoder (Encoder): The encoder object.
        data (list): The collected data.
        start_time (int): The start time of data collection.
    """

    def __init__(self, encoder):
        """
        Initializes the Collector class.

        Args:
            encoder (Encoder): The encoder object.
        """
        self.encoder = encoder
        self.data = []
        self.start_time = ticks_us()

    def collect_data(self):
        """
        Collects data from the encoder, including timestamp, position, and velocity.
        """
        self.encoder.update()
        timestamp = ticks_diff(ticks_us(), self.start_time) / 1_000_000
        position = self.encoder.get_position() * (2 * pi / 1440)
        velocity = self.encoder.get_velocity() * (2 * pi / 1440)
        self.data.append((timestamp, position, velocity))

    def reset_data(self):
        """
        Resets the collected data.
        """
        self.data = []
        self.start_time = ticks_us()

    def get_data(self):
        """
        Gets the collected data.

        Returns:
            list: The collected data.
        """
        return self.data

    def print_data(self):
        """
        Prints the collected data in a formatted manner.
        """
        print("Time (s), Position, Velocity")
        for timestamp, position, velocity in self.data:
            print(f"{timestamp:.6f}, {position}, {velocity:.6f}")


class Sensor_Array:
    """
    A class to handle an array of sensors.

    Attributes:
        sensor_pins (list): The sensor pins.
        sensor_ADC (list): The ADC objects for sensors.
        sample_time (int): The sample time in milliseconds.
        sensor_data (list): The sensor data.
        black_cal (list): The black calibration values.
        white_cal (list): The white calibration values.
        dot_cal (list): The dot calibration values.
        linear (list): The linear values from sensors.
        centroidact (float): The centroid of active sensors.
        raw_value (list): The raw sensor values.
    """

    def __init__(self):
        """
        Initializes the Sensor_Array class.
        """
        self.sensor_pins = [
            Pin(Pin.cpu.C5, mode=Pin.OUT_PP, value=0), Pin(Pin.cpu.C0, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.C1, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.A6, mode=Pin.OUT_PP, value=0), Pin(Pin.cpu.B0, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.B1, mode=Pin.OUT_PP, value=0), Pin(Pin.cpu.A4, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.C4, mode=Pin.OUT_PP, value=0), Pin(Pin.cpu.A1, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.C3, mode=Pin.OUT_PP, value=0), Pin(Pin.cpu.A0, mode=Pin.OUT_PP, value=0),
            Pin(Pin.cpu.C2, mode=Pin.OUT_PP, value=0)
        ]
        self.sensor_ADC = [ADC(pin) for pin in self.sensor_pins]
        self.sample_time = 1000
        self.sensor_data = [0] * 12
        self.black_cal = [0] * 12
        self.white_cal = [0] * 12
        self.dot_cal = [0] * 12
        self.linear = [0] * 12
        self.centroidact = 0
        self.raw_value = [0] * 12

    def calibrate_black(self):
        """
        Calibrates the sensors for black values by reading ADC values and storing them.
        """
        black_cal_sum = 0
        print("Calibrating black values...")
        for i in range(len(self.sensor_ADC)):
            self.black_cal[i] = self.sensor_ADC[i].read()
            black_cal_sum += self.black_cal[i]
        print("Black calibration complete:")
        print(black_cal_sum)

    def calibrate_white(self):
        """
        Calibrates the sensors for white values by reading ADC values and storing them.
        """
        white_cal_sum = 0
        print("Calibrating white values...")
        for i in range(len(self.sensor_ADC)):
            self.white_cal[i] = self.sensor_ADC[i].read()
            white_cal_sum += self.white_cal[i]
        print("White calibration complete:")
        print(self.white_cal)
        print(white_cal_sum)

    def calibrate_dot(self):
        """
        Calibrates the sensors for dot values by reading ADC values and storing them.
        """
        dot_cal_sum = 0
        print("Calibrating dot value...")
        for i in range(len(self.sensor_ADC)):
            self.dot_cal[i] = self.sensor_ADC[i].read()
            dot_cal_sum += self.dot_cal[i]
        print("Dot calibration complete:")
        print(self.dot_cal)
        print(dot_cal_sum)

    def read_sensors(self):
        """
        Reads the sensor values and normalizes them based on calibration values.
        """
        for i in range(len(self.sensor_ADC)):
            raw_value = self.sensor_ADC[i].read()
            if self.white_cal[i] == self.black_cal[i]:
                self.sensor_data[i] = 0.0
            else:
                self.linear[i] = (raw_value - self.white_cal[i]) / (self.black_cal[i] - self.white_cal[i])
                if self.linear[i] > 1:
                    self.linear[i] = 1
                elif self.linear[i] < 0:
                    self.linear[i] = 0
                self.sensor_data[i] = self.linear[i]

    def check_for_grid(self):
        """
        Checks if a grid is detected by analyzing sensor data intensity.

        Returns:
            bool: True if grid is detected, False otherwise.
        """
        total_intensity = 0
        white_cal_sum = 0
        dot_cal_sum = 0
        for i in range(len(self.sensor_ADC)):
            raw_value = self.sensor_ADC[i].read()
            if self.white_cal[i] == self.black_cal[i]:
                self.sensor_data[i] = 0.0
            else:
                self.linear[i] = (raw_value - self.white_cal[i]) / (self.black_cal[i] - self.white_cal[i])
                if self.linear[i] > 1:
                    self.linear[i] = 1
                elif self.linear[i] < 0:
                    self.linear[i] = 0
                self.sensor_data[i] = self.linear[i]
            total_intensity += raw_value
            white_cal_sum += self.white_cal[i]
            dot_cal_sum += self.dot_cal[i]
        self.grid_low_range = 6000
        self.grid_high_range = dot_cal_sum + 300
        self.grid_threshold = (white_cal_sum - 300)
        self.grid_detected = False
        print("Total Intensity:", total_intensity)
        if total_intensity > self.grid_low_range and not self.grid_detected:
            self.grid_detected = True
        return self.grid_detected

    def compute_centroid(self):
        """
        Computes the centroid of the active sensors based on their values.

        Returns:
            float: The centroid value.
        """
        numerator = 0
        denominator = 0
        for i, value in enumerate(self.sensor_data):
            numerator += value * (i + 1)
            denominator += value
        if denominator == 0:
            self.centroidact = 5.5
            return 5.5
        self.centroidact = float(numerator / denominator)
        return self.centroidact


class BNO055:
    """
    A class to interact with the BNO055 sensor over I2C.

    Attributes:
        i2c (I2C): The I2C interface.
    """

    # I2C Address and Register Definitions
    BNO055_ADDRESS = 0x28
    BNO055_OPR_MODE_ADDR = 0x3D
    BNO055_CALIB_STAT_ADDR = 0x35
    BNO055_CALIB_DATA_ADDR = 0x55
    BNO055_EULER_H_LSB_ADDR = 0x1A
    BNO055_GYRO_DATA_X_LSB_ADDR = 0x14

    # Operation Modes
    CONFIG_MODE = 0x00
    NDOF_MODE = 0x0C

    def __init__(self, i2c):
        """
        Initializes the BNO055 class.

        Args:
            i2c (I2C): The I2C interface.
        """
        self.i2c = i2c
        self.set_mode(self.CONFIG_MODE)

    def set_mode(self, mode):
        """
        Sets the operation mode of the sensor.

        Args:
            mode (int): The operation mode.
        """
        if mode != self.CONFIG_MODE:
            self.i2c.mem_write(bytes([self.CONFIG_MODE]), self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR)
            delay(30)
        self.i2c.mem_write(bytes([mode]), self.BNO055_ADDRESS, self.BNO055_OPR_MODE_ADDR)
        delay(30)

    def get_calibration_status(self):
        """
        Gets the calibration status of the sensor.

        Returns:
            int: The calibration status.
        """
        calib_stat = self.i2c.mem_read(1, self.BNO055_ADDRESS, self.BNO055_CALIB_STAT_ADDR)
        return calib_stat[0]

    def get_calibration_data(self):
        """
        Gets the calibration data from the sensor.

        Returns:
            bytes: The calibration data.
        """
        calib_data = self.i2c.mem_read(22, self.BNO055_ADDRESS, self.BNO055_CALIB_DATA_ADDR)
        return calib_data

    def set_calibration_data(self, calib_data):
        """
        Sets the calibration data of the sensor.

        Args:
            calib_data (bytes): The calibration data.

        Raises:
            ValueError: If the calibration data length is not 22 bytes.
        """
        if len(calib_data) != 22:
            raise ValueError("Invalid calibration data length")
        self.i2c.mem_write(calib_data, self.BNO055_ADDRESS, self.BNO055_CALIB_DATA_ADDR)

    def read_euler_angles(self):
        """
        Reads the Euler angles from the sensor.

        Returns:
            tuple: The heading, roll, and pitch angles.
        """
        data = self.i2c.mem_read(6, self.BNO055_ADDRESS, self.BNO055_EULER_H_LSB_ADDR)
        if data:
            heading, roll, pitch = struct.unpack('<hhh', data)
            return heading / 16.0, roll / 16.0, pitch / 16.0
        return None

    def read_heading(self):
        """
        Reads the heading angle from the sensor.

        Returns:
            float: The heading angle.
        """
        angles = self.read_euler_angles()
        return angles[0] if angles else None

    def read_angular_velocity(self):
        """
        Reads the angular velocity from the sensor.

        Returns:
            tuple: The angular velocities along the x, y, and z axes.
        """
        data = self.i2c.mem_read(6, self.BNO055_ADDRESS, self.BNO055_GYRO_DATA_X_LSB_ADDR)
        if data:
            x, y, z = struct.unpack('<hhh', data)
            return x / 16.0, y / 16.0, z / 16.0
        return None

    def read_yaw_rate(self):
        """
        Reads the yaw rate from the sensor.

        Returns:
            float: The yaw rate.
        """
        ang_vel = self.read_angular_velocity()
        return ang_vel[2] if ang_vel else None


class Bumper:
    """
    A class to handle bumper sensors.

    Attributes:
        left_bumper_pins (list): The left bumper pins.
        right_bumper_pins (list): The right bumper pins.
        bump_flag (int): The bump flag.
    """

    def __init__(self):
        """
        Initializes the Bumper class.
        """
        self.left_bumper_pins = [
            Pin(Pin.cpu.C11, mode=Pin.IN, pull=Pin.PULL_UP),
            Pin(Pin.cpu.C10, mode=Pin.IN, pull=Pin.PULL_UP)
        ]
        self.right_bumper_pins = [
            Pin(Pin.cpu.B2, mode=Pin.IN, pull=Pin.PULL_UP),
            Pin(Pin.cpu.B3, mode=Pin.IN, pull=Pin.PULL_UP),
            Pin(Pin.cpu.A10, mode=Pin.IN, pull=Pin.PULL_UP)
        ]
        self.bump_flag = 1

    def readings(self):
        """
        Gets the readings from the bumper sensors.

        Returns:
            bool: True if all bumpers are not triggered (no bump detected), False otherwise.
        """
        self.reading_left = [pin.value() for pin in self.left_bumper_pins]
        self.reading_right = [pin.value() for pin in self.right_bumper_pins]
        return all(self.reading_left) and all(self.reading_right)

    def check_bump(self):
        """
        Checks if any bumper is triggered and updates the bump flag accordingly.
        """
        left_values = [pin.value() for pin in self.left_bumper_pins]
        right_values = [pin.value() for pin in self.right_bumper_pins]
        left_bump = any(value == 0 for value in left_values)
        right_bump = any(value == 0 for value in right_values)
        if left_bump or right_bump:
            self.bump_flag = 0
        else:
            self.bump_flag = 1