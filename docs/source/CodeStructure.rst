Code Structure
==============

This section describes the software architecture of the ME405 ROMI robot project.

Main Components
---------------
The project is divided into several key modules, each handling a specific aspect of the system:

- **Main Execution (`main.py`)**  
  The entry point of the program, responsible for initializing all hardware components and managing task execution.

- **Motor Driver (`Romi_Drivers.py`)**  
  This module interfaces with the motor drivers, allowing speed control through Pulse Width Modulation (PWM).

- **IMU & Sensor Processing (`imu.py`)**  
  Reads IMU sensor data and performs filtering to ensure accurate motion estimation.

- **Encoders (`encoder.py`)**  
  Handles quadrature encoder readings for precise motor positioning.

- **Task Scheduling (`task.py`)**  
  Implements a cooperative multitasking system to handle multiple real-time operations.

File Structure Overview
-----------------------
Below is the hierarchical structure of the project files:

.. code-block:: bash

   ME-405-Romi/
   ├── code/
   │   ├── main.py              # Main entry script
   │   ├── Romi_Drivers.py      # Motor driver module
   │   ├── imu.py               # IMU processing
   │   ├── encoder.py           # Quadrature encoder interface
   │   ├── task.py              # Task scheduling
   ├── docs/
   │   ├── source/
   │   │   ├── index.rst
   │   │   ├── hardware.rst
   │   │   ├── wiring.rst
   │   │   ├── video_demo.rst
   │   │   ├── code_structure.rst  # This page
   ├── README.md
   ├── Makefile
   ├── conf.py

Module Descriptions
-------------------
Each module plays a critical role in system operation.