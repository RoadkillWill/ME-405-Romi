Code Structure
==============

This section describes the software architecture of the ME405 ROMI robot project. The code uses FSM (Finite State Machine) Logic to progress between distinct cases called "states." More information on this structure can be found at: https://spluttflob.github.io/ME405-Support/index.html#ss_modules

Task Diagram
============

Our code is broken up into distinct tasks using a scheduler to help assign priorities and ensure cooperative handling. These tasks are explained in more detail below to show function and structure.

.. image:: _static/Romi_task_diagram.png
   :width: 600px
   :align: center
   :alt: Task Diagram

State Diagrams for Each Task
============================

Left Pivot
----------

.. image:: _static/TD_left_pivot.png
   :width: 600px
   :align: center
   :alt: Left Pivot Task

Right Pivot
-----------

.. image:: _static/TD_right_pivot.png
   :width: 600px
   :align: center
   :alt: Right Pivot Task

Bump Check
----------

.. image:: _static/TD_bump_check.png
   :width: 600px
   :align: center
   :alt: Bump Check Task

Calibrate IMU
-------------

.. image:: _static/TD_calibrate_imu.png
   :width: 600px
   :align: center
   :alt: Calibrate IMU Task

Calibrate Sensor
----------------

.. image:: _static/TD_calibrate_sensor.png
   :width: 600px
   :align: center
   :alt: Calibrate Sensor Task

Closed Loop Control
-------------------

.. image:: _static/TD_cl_control.png
   :width: 600px
   :align: center
   :alt: Closed Loop Control Task

Encoder Read
------------

.. image:: _static/TD_encoder.png
   :width: 600px
   :align: center
   :alt: Encoder Read Task

Position Control
----------------

.. image:: _static/TD_position_control.png
   :width: 600px
   :align: center
   :alt: Position Control Task

User Command
------------

.. image:: _static/TD_user_commands.png
   :width: 600px
   :align: center
   :alt: User Command Task


