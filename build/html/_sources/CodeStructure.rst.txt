Code Structure
==============

This section describes the software architecture of the ME405 ROMI robot project. The code uses FSM (Finite State Machine) Logic to progress between distinct cases called "states." The code follows a cooperative multitasking structure using the cotask library, organizing functionality into multiple tasks that yield control periodically. It utilizes shared variables (Share and Queue) for inter-task communication, managing motor control, sensor calibration, and navigation states. Hardware interfaces, including motor controllers, encoders, and sensors, are abstracted through Romi_Drivers. The FSM governs grid navigation and backup sequences, while a PID controller ensures closed-loop control for line following. The scheduler prioritizes tasks for efficient execution using pri_sched(), making the system well-structured for embedded robotics applications.

More information on this structure can be found at: https://spluttflob.github.io/ME405-Support/index.html#ss_modules

.. contents:: **Contents**
   :depth: 2
   :local:

Task Diagram
------------

Here is an overview of our task structure. Although not all the shares that are in the code are listed here, this is due to them being used as variables only in one given task. They are more variables than shares.

.. image:: _static/Romi_task_diagram.png
   :width: 600px
   :align: center
   :alt: Task Diagram

State Diagrams for Each Task
----------------------------

To give some more detail about each task that is used in the code, here are our state transition diagrams that show in more detail what each task does.

Left Pivot

.. image:: _static/TD_left_pivot.png
   :width: 600px
   :align: center
   :alt: Left Pivot Task

Right Pivot

.. image:: _static/TD_right_pivot.png
   :width: 600px
   :align: center
   :alt: Right Pivot Task

Bump Check

.. image:: _static/TD_bump_check.png
   :width: 600px
   :align: center
   :alt: Bump Check Task

Calibrate IMU

.. image:: _static/TD_calibrate_imu.png
   :width: 600px
   :align: center
   :alt: Calibrate IMU Task

Calibrate Sensor

.. image:: _static/TD_calibrate_sensor.png
   :width: 600px
   :align: center
   :alt: Calibrate Sensor Task

Closed Loop Control

.. image:: _static/TD_cl_control.png
   :width: 600px
   :align: center
   :alt: Closed Loop Control Task

Encoder Read

.. image:: _static/TD_encoder.png
   :width: 600px
   :align: center
   :alt: Encoder Read Task

Position Control

.. image:: _static/TD_position_control.png
   :width: 600px
   :align: center
   :alt: Position Control Task

User Command

.. image:: _static/TD_user_commands.png
   :width: 600px
   :align: center
   :alt: User Command Task


