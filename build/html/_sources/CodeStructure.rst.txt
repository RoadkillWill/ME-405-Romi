Code Structure
==============

This section describes the software architecture of the ME405 ROMI robot project. The code uses FSM (Finite State Machine) Logic to progress between distinct cases called "states." More information on this structure can be found at: https://spluttflob.github.io/ME405-Support/index.html#ss_modules

Task Diagram
============

Our code is broken up into distinct tasks using a scheduler to help assign priorities and ensure cooperative handling. These tasks are explained in more detail below to show function and structure.

State Diagrams for Each Task
============================

Left Pivot
----------

Right Pivot
-----------

Bump Check
----------

Calibrate IMU
-------------

Calibrate Sensor
----------------

Closed Loop Control
-------------------

Encoder
-------

Position Control
----------------

User Command
------------


