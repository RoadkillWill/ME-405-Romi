Main Task Code
=======================

This section provides an overview of the main tasks and scheduler logic used in the ROMI robot.

.. contents:: **Contents**
   :depth: 2
   :local:

Main Code
------------
This file contains the main task loop and all tasks involved in controlling the ROMI robot.

.. code-block:: python

    from pyb import Pin, USB_VCP, I2C, delay
    from Romi_Drivers import Motor, Encoder, Collector, Sensor_Array, BNO055, Bumper
    from time import ticks_us, ticks_diff, ticks_add
    from task_share2 import Share, Queue
    from cotask import Task, task_list
    from nb_input import NB_Input

Task: User Command
-------------------
Handles user input via UART and updates shared variables.

.. code-block:: python

    def task_user_command():
        """
        Reads user input from UART, processes commands, and updates shared variables accordingly.
        
        Commands include:
        - Integers (-100 to 100) to control motor effort.
        - 'calibrate' to initiate sensor calibration.
        - 'cl' to activate closed-loop control.
        - 'i2c' to start IMU calibration.
        - 'done' to signal completion of calibration steps.

        The function continuously monitors the UART input and processes commands in real time.
        """
        while True:
            nb_in.check()
            if nb_in.any():
                user_input_str = nb_in.get().strip()
                print("Received input:", user_input_str)
                try:
                    effort = int(user_input_str)
                    if -100 <= effort <= 100:
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
Controls motor effort based on user input, sensor readings, and grid navigation.

.. code-block:: python

    def task_position_control():
        """
        Adjusts motor efforts based on user input, bumper sensor readings, and grid navigation state.

        - If backup is needed, executes a predefined backup sequence to maneuver the robot.
        - If the robot is navigating the grid, adjusts motor control to follow grid paths.
        - Uses sensor feedback and IMU correction to ensure smooth and accurate movement.

        The function continuously checks for changes in navigation states and applies the appropriate control logic.
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
Reads encoder values and updates shared variables.

.. code-block:: python

    def task_encoder_read():
        """
        Reads the encoder values and updates shared variables for position and velocity.

        - Retrieves the latest encoder counts.
        - Computes the position and velocity.
        - Stores the values in shared variables for other tasks to use.

        This function ensures accurate movement tracking by continuously updating encoder values.
        """
        while True:
            left_encoder.update()
            right_encoder.update()
            left_position.put(int(left_encoder.get_position()))
            right_position.put(int(right_encoder.get_velocity()))
            yield 10

Task: Closed-Loop Control
--------------------------
Uses a PID controller to follow a line or navigate the grid.

.. code-block:: python

    def task_closed_loop_control():
        """
        Implements closed-loop control using a PID controller.

        - Computes an error based on the difference between the desired and actual line position.
        - Adjusts motor effort dynamically using proportional, integral, and derivative terms.
        - Handles transitions between line following and grid navigation.

        The function continuously monitors the sensor feedback and applies corrections to stay on track.
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
Guides the user through IMU calibration.

.. code-block:: python

    def task_calibrate_imu_once():
        """
        Guides the user through IMU calibration and stores calibration coefficients.

        - Prompts the user to move the sensor in all directions.
        - Checks if all sensor axes are fully calibrated.
        - Stores calibration coefficients once the process is complete.

        This function runs once until successful calibration is achieved.
        """
        calibrated = False
        while True:
            if calibrate_i2c.get() and not calibrated:
                calib_status = imu.get_calibration_status()
                if (((calib_status >> 6) & 0x03) == 3 and 
                    ((calib_status >> 4) & 0x03) == 3 and 
                    ((calib_status >> 2) & 0x03) == 3 and 
                    (calib_status & 0x03) == 3):
                    print("IMU fully calibrated!")
                    calibrate_i2c.put(False)
                    calibrated = True

Task: Pivot Turns
------------------
Handles left and right pivot turns using IMU feedback.

.. code-block:: python

    def task_right_pivot():
        """
        Executes a right pivot turn based on IMU feedback.

        - Reads the current heading from the IMU.
        - Adjusts motor effort until the desired turn angle is achieved.
        - Stops the motors once the turn is complete.

        The function ensures smooth and precise 90-degree turns using real-time heading adjustments.
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

Task: Bump Check
-----------------
Continuously checks for bump sensor activation.

.. code-block:: python

    def task_bump_check():
        """
        Monitors the bumper sensors for collisions.

        - If a collision is detected, stops the robot and triggers a backup sequence.
        - Ensures that the robot does not continue moving after an unexpected bump.

        This function continuously checks for collisions and applies the necessary safety response.
        """
        while True:
            bumper.check_bump()
            if bumper.bump_flag == 0:
                back_up.put(True)

Scheduler
----------
Manages all tasks and executes them based on priority.

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
