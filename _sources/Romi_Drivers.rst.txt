Romi Drivers Documentation
===========================

This section provides an overview of the motor, encoder, sensor, and IMU classes used in the ROMI robot.

.. contents:: **Contents**
   :depth: 2
   :local:

Motor Class
------------
Controls the speed and direction of the motors using PWM.

.. code-block:: python

    class Motor:
        """
        A class to control a DC motor using PWM signals.

        Attributes:
            nSLP_pin (Pin): The motor sleep pin.
            tim (Timer): The timer object for PWM control.
            PWM_pin (Timer.channel): The PWM signal channel.
            DIR_pin (Pin): The direction control pin.
        """

        def __init__(self, motor_tim, PWM_Pin, DIR, nSLP):
            """
            Initializes the Motor class.

            Args:
                motor_tim (int): Timer number for PWM control.
                PWM_Pin (int): PWM signal output pin.
                DIR (int): Direction control pin.
                nSLP (int): Motor sleep control pin.
            """
        
        def set_effort(self, effort):
            """
            Sets the motor speed and direction.

            Args:
                effort (int): Speed (-100 to 100), where negative values reverse direction.
            """
        
        def enable(self):
            """
            Enables the motor by setting the sleep pin high.
            """
        
        def disable(self):
            """
            Disables the motor by setting the sleep pin low.
            """

Encoder Class
-------------
Tracks the rotation of the wheels to provide position and velocity feedback.

.. code-block:: python

    class Encoder:
        """
        A class for handling quadrature encoder feedback.

        Attributes:
            position (int): The current encoder position.
            prev_count (int): The last recorded encoder count.
            delta (int): Change in encoder count since last update.
            dt (int): Time interval between updates.
            timer (Timer): Timer object for tracking encoder pulses.
        """

        def __init__(self, tim, chA_pin, chB_pin):
            """
            Initializes the Encoder class.

            Args:
                tim (int): Timer number used for quadrature encoding.
                chA_pin (int): Channel A encoder pin.
                chB_pin (int): Channel B encoder pin.
            """

        def update(self):
            """
            Updates the encoder reading by calculating the difference in encoder counts.
            """

        def get_position(self):
            """
            Retrieves the current position.

            Returns:
                int: Absolute position based on encoder readings.
            """

        def get_velocity(self):
            """
            Computes the velocity using encoder counts.

            Returns:
                float: Estimated velocity in counts per second.
            """

        def zero(self):
            """
            Resets the encoder position to zero.
            """

Collector Class
---------------
Collects and stores encoder data for analysis.

.. code-block:: python

    class Collector:
        """
        A class for collecting encoder data over time.

        Attributes:
            encoder (Encoder): Encoder object for data collection.
            data (list): A list storing timestamped encoder readings.
            start_time (int): The start time of data collection.
        """

        def __init__(self, encoder):
            """
            Initializes the Collector class.

            Args:
                encoder (Encoder): The encoder object to track.
            """

        def collect_data(self):
            """
            Collects encoder data, including timestamp, position, and velocity.
            """

        def reset_data(self):
            """
            Clears the collected data.
            """

        def get_data(self):
            """
            Retrieves the collected encoder data.

            Returns:
                list: List of timestamped encoder readings.
            """

        def print_data(self):
            """
            Prints collected encoder data in a readable format.
            """

Sensor Array Class
------------------
Handles an array of infrared reflectance sensors used for line detection.

.. code-block:: python

    class Sensor_Array:
        """
        A class for managing an array of IR sensors.

        Attributes:
            sensor_pins (list): List of sensor input pins.
            sensor_ADC (list): ADC objects for sensor readings.
            black_cal (list): Stored black-level calibration data.
            white_cal (list): Stored white-level calibration data.
        """

        def __init__(self):
            """
            Initializes the Sensor_Array class.
            """

        def calibrate_black(self):
            """
            Calibrates sensors for black surface detection.
            """

        def calibrate_white(self):
            """
            Calibrates sensors for white surface detection.
            """

        def calibrate_dot(self):
            """
            Calibrates sensors for detecting grid markers.
            """

        def read_sensors(self):
            """
            Reads sensor values and normalizes them based on calibration data.
            """

        def check_for_grid(self):
            """
            Checks for a grid intersection using sensor readings.

            Returns:
                bool: True if a grid is detected, False otherwise.
            """

        def compute_centroid(self):
            """
            Computes the weighted centroid of the detected line.

            Returns:
                float: Centroid position relative to sensor array.
            """

IMU (BNO055) Class
-------------------
Interacts with the BNO055 IMU for heading and motion tracking.

.. code-block:: python

    class BNO055:
        """
        A class to interact with the BNO055 IMU.

        Attributes:
            i2c (I2C): I2C communication interface.
        """

        def __init__(self, i2c):
            """
            Initializes the BNO055 IMU.

            Args:
                i2c (I2C): I2C interface for communication.
            """

        def set_mode(self, mode):
            """
            Sets the operation mode of the IMU.

            Args:
                mode (int): Desired IMU mode.
            """

        def get_calibration_status(self):
            """
            Retrieves the IMU calibration status.

            Returns:
                int: Calibration status of the IMU.
            """

        def read_euler_angles(self):
            """
            Reads the IMU’s Euler angles.

            Returns:
                tuple: (heading, roll, pitch).
            """

        def read_heading(self):
            """
            Retrieves the current heading angle.

            Returns:
                float: Heading angle in degrees.
            """

        def read_angular_velocity(self):
            """
            Reads angular velocity from the IMU.

            Returns:
                tuple: (x, y, z) angular velocities.
            """

Bumper Class
------------
Handles the bumper sensors for collision detection.

.. code-block:: python

    class Bumper:
        """
        A class for detecting bumps using mechanical bumper sensors.

        Attributes:
            left_bumper_pins (list): Pins for left bumper sensors.
            right_bumper_pins (list): Pins for right bumper sensors.
            bump_flag (int): Flag indicating if a bump is detected.
        """

        def __init__(self):
            """
            Initializes the bumper sensors.
            """

        def readings(self):
            """
            Reads bumper sensor states.

            Returns:
                bool: True if no bump detected, False otherwise.
            """

        def check_bump(self):
            """
            Checks for bumper activation and updates the bump flag.
            """

---