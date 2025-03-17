Main Task Code
=======================

This section provides an overview of the main tasks and scheduler logic used in the ROMI robot.

.. contents:: **Contents**
   :depth: 2
   :local:

Main Code
------------
.. code-block:: python

    from pyb import Pin, USB_VCP, I2C, delay
    from Romi_Drivers import Motor, Encoder, Collector, Sensor_Array, BNO055, Bumper
    from time import ticks_us, ticks_diff, ticks_add
    from task_share2 import Share, Queue
    from cotask import Task, task_list
    from nb_input import NB_Input

    """ --------------------- Initialization --------------------- """

    """ Initialize UART communication """
    ser = pyb.UART(5, 115200, timeout=1000)
    pyb.repl_uart(ser)
    nb_in = NB_Input(ser, echo=True)

    """ Initialize Romi Constants """
    wheel_circumference_in = 8.658
    encoder_ticks_per_rotation = 1440
    grid_distance_in = 28.5
    grid_distance_in_half = grid_distance_in / 2
    grid_escape_distance_in = 6
    deadzone1 = 1

    """ Define shared variables """
    left_motor_effort = Share('h', thread_protect=True, name="Left Motor Effort")
    right_motor_effort = Share('h', thread_protect=True, name="Right Motor Effort")
    left_position = Share('h', thread_protect=True, name="Left Position")
    right_position = Share('h', thread_protect=True, name="Right Position")

    """ Create driver objects """
    left_motor = Motor(motor_tim=8, PWM_Pin=Pin.cpu.C8, DIR=Pin.cpu.C6, nSLP=Pin.cpu.C7)
    right_motor = Motor(motor_tim=4, PWM_Pin=Pin.cpu.B8, DIR=Pin.cpu.B7, nSLP=Pin.cpu.B6)
    left_encoder = Encoder(tim=1, chA_pin=Pin.cpu.A8, chB_pin=Pin.cpu.A9)
    right_encoder = Encoder(tim=3, chA_pin=Pin.cpu.B4, chB_pin=Pin.cpu.B5)
    Sensor = Sensor_Array()
    bumper = Bumper()

    """ Enable motor drivers """
    left_motor.enable()
    right_motor.enable()

    """ Initialize I2C for the IMU (using bus 2). """
    imu_i2c = I2C(2, I2C.CONTROLLER, baudrate=400000)
    imu = BNO055(imu_i2c)

    """ Set the IMU to NDOF mode. """
    imu.set_mode(BNO055.NDOF_MODE)

Task: User Command
-------------------
.. code-block:: python

    def task_user_command():
        """
        Task to handle user commands received via UART.
        """
        while True:
            nb_in.check()
            if nb_in.any():
                user_input_str = nb_in.get().strip()
                print("Received input:", user_input_str)
                try:
                    effort = int(user_input_str)
                    if -100 <= effort <= 100:
                        print("Storing effort", effort, "in queue...")
                        user_input.put(effort)
                except ValueError:
                    cmd = user_input_str.lower()
                    if cmd == "calibrate":
                        calibrate_sensor.put(True)
                    elif cmd == "cl":
                        first_movement.put(True)
                        motor_drive.put(True)
                        motor_control.put(True)
                        line_follow.put(True)
                    elif cmd == "i2c":
                        calibrate_i2c.put(True)
                    elif cmd == "done":
                        cali_sensor.put(True)
                    else:
                        print("[ERROR] Invalid input.")

Task: Position Control
-----------------------
.. code-block:: python

    def task_position_control():
        """
        Task to control the position of the robot.
        """
        while True:
            if motor_drive.get():
                correction_factor = actuator_input.get() * 4
                effort_correction = int(correction_factor / 2)
                if correction_factor > 0:
                    left_motor_effort.put(base_effort - abs(effort_correction))
                    right_motor_effort.put(base_effort + abs(effort_correction))
                elif correction_factor < 0:
                    left_motor_effort.put(base_effort + abs(effort_correction))
                    right_motor_effort.put(base_effort - abs(effort_correction))
                else:
                    left_motor_effort.put(base_effort)
                    right_motor_effort.put(base_effort)
                left_motor.set_effort(left_motor_effort.get())
                right_motor.set_effort(right_motor_effort.get())

Task: Encoder Read
-------------------
.. code-block:: python

    def task_encoder_read():
        """
        Task to read encoder values and update shared variables.
        """
        while True:
            left_encoder.update()
            right_encoder.update()
            left_position.put(int(left_encoder.get_position()))
            right_position.put(int(right_encoder.get_position()))
            left_velocity.put(int(left_encoder.get_velocity()))
            right_velocity.put(int(right_encoder.get_velocity()))

Task: Closed-Loop Control
--------------------------
.. code-block:: python

    def task_closed_loop_control():
        """
        Task to handle closed-loop control for line following and grid navigation.
        """
        kp = 3
        while True:
            if motor_control.get():
                Sensor.read_sensors()
                sensed_line_position = float(Sensor.compute_centroid())
                error = setpoint - sensed_line_position
                p_error = kp * error
                control_signal = max(min(p_error, 100), -100)
                actuator_input.put(control_signal)

Task: IMU Calibration
----------------------
.. code-block:: python

    def task_calibrate_imu_once():
        """
        Task to calibrate the IMU sensor once.
        """
        calibrated = False
        while True:
            if calibrate_i2c.get() and not calibrated:
                print("Starting IMU calibration.")
                calib_status = imu.get_calibration_status()
                if (((calib_status >> 6) & 0x03) == 3 and 
                    ((calib_status >> 4) & 0x03) == 3 and 
                    ((calib_status >> 2) & 0x03) == 3 and 
                    (calib_status & 0x03) == 3):
                    print("IMU fully calibrated!")
                    calibrate_i2c.put(False)
                    calibrated = True

Task: Right Pivot
------------------
.. code-block:: python

    def task_right_pivot():
        """
        Task to perform a right pivot turn.
        """
        while True:
            if right_pivot.get():
                desired_heading = pre_turn_heading.get() + 80 
                current_heading = imu.read_heading()
                error = (desired_heading - current_heading + 180) % 360 - 180
                if abs(error) < 6.0:
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    right_pivot.put(False)
                    done_right_pivot.put(True)
                else:
                    effort = max(min(int(3 * error), 100), -100)
                    left_motor.set_effort(effort+6)
                    right_motor.set_effort(-effort-6)

Task: Left Pivot
-----------------
.. code-block:: python

    def task_left_pivot():
        """
        Task to perform a left pivot turn.
        """
        while True:
            if left_pivot.get():
                desired_heading = pre_turn_heading.get() - 90
                current_heading = imu.read_heading()
                error = (desired_heading - current_heading + 180) % 360 - 180
                if abs(error) < 6.5:
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    left_pivot.put(False)
                    done_left_pivot.put(True)
                else:
                    effort = max(min(int(3 * error), 100), -100)
                    left_motor.set_effort(effort+6)
                    right_motor.set_effort(-effort-6)

Task: Bump Check
-----------------
.. code-block:: python

    def task_bump_check():
        """
        Task to check for bumps.
        """
        while True:
            bumper.check_bump()
            if bumper.bump_flag == 0: 
                print("bump_flag set")
                back_up.put(True)

Scheduler
----------
.. code-block:: python

    task_list.append(Task(task_encoder_read, name='Encoder Read', priority=8, period=10))
    task_list.append(Task(task_user_command, name='User Command', priority=2, period=20))
    task_list.append(Task(task_closed_loop_control, name='Closed Loop Control', priority=7, period=20))
    
    print("Starting Scheduler")
    while True:
        try:
            task_list.pri_sched()
        except KeyboardInterrupt:
            left_motor.disable()
            right_motor.disable()
            break
