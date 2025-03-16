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
desired_left_speed = Share('h', thread_protect=True, name="Desired Left Speed")
desired_right_speed = Share('h', thread_protect=True, name="Desired Right Speed")
left_motor_effort = Share('h', thread_protect=True, name="Left Motor Effort")
right_motor_effort = Share('h', thread_protect=True, name="Right Motor Effort")
left_position = Share('h', thread_protect=True, name="Left Position")
right_position = Share('h', thread_protect=True, name="Right Position")
left_velocity = Share('h', thread_protect=True, name="Left Velocity")
right_velocity = Share('h', thread_protect=True, name="Right Velocity")
calibrate_sensor = Share('b', thread_protect=True, name="Calibrate Sensor")
motor_drive = Share('b', thread_protect=True, name="Motor Drive")
motor_control = Share('b', thread_protect=True, name="Motor Control")
calibrate_i2c = Share('b', thread_protect=True, name="I2C Calibrate")
back_up = Share('b', thread_protect=True, name="Back Up")
line_follow = Share('b', thread_protect=True, name="Line Follow")
grid_section = Share('b', thread_protect=True, name="Grid Section")
distance_ticks = Share('h', thread_protect=True, name="Grid Distance Ticks")
cali_sensor = Share('b', thread_protect=True, name="Calibrate Black/White Sensor")
grid_section_state = Share('h', thread_protect=True, name="Grid Section State")
left_pivot = Share('b', thread_protect=True, name="Left Pivot")
right_pivot = Share('b', thread_protect=True, name="Right Pivot")
done_left_pivot = Share('b', thread_protect=True, name="Done Left Pivot")
done_right_pivot = Share('b', thread_protect=True, name="Done Right Pivot")
pre_turn_heading = Share('f', thread_protect=True, name="Pre-Turn Heading")
first_movement = Share('b', thread_protect=True, name="First Movement")
starting_heading = Share('f', thread_protect=True, name="Starting Heading")
on_grid = Share('b', thread_protect=True, name="On Grid")
bumper_on = Share('b', thread_protect=True, name="Bumper")
heading_recorded = Share('b', thread_protect=True, name="Heading Recorded")
heading_obtained = Share('b', thread_protect=True, name="Heading Obtained")
turn_state = Share('h', thread_protect=True, name="Turn State")
user_input = Queue('h', 10, thread_protect=True, name="User Input")
actuator_input = Share('f', thread_protect=True, name="Actuator Input")

""" Create driver objects """
left_motor = Motor(motor_tim=8, PWM_Pin=Pin.cpu.C8, DIR=Pin.cpu.C6, nSLP=Pin.cpu.C7)
right_motor = Motor(motor_tim=4, PWM_Pin=Pin.cpu.B8, DIR=Pin.cpu.B7, nSLP=Pin.cpu.B6)
left_encoder = Encoder(tim=1, chA_pin=Pin.cpu.A8, chB_pin=Pin.cpu.A9)
right_encoder = Encoder(tim=3, chA_pin=Pin.cpu.B4, chB_pin=Pin.cpu.B5)
left_collector = Collector(left_encoder)
right_collector = Collector(right_encoder)
Sensor = Sensor_Array()
bumper = Bumper()

""" Enable motor drivers """
left_motor.enable()
right_motor.enable()

"""  Initialize I2C for the IMU (using bus 2). """
imu_i2c = I2C(2, I2C.CONTROLLER, baudrate=400000)
imu = BNO055(imu_i2c)

""" Set the IMU to NDOF mode. """
imu.set_mode(BNO055.NDOF_MODE)

""" --------------------- Task Definitions --------------------- """

def task_user_command():
    """
    Task to handle user commands received via UART.

    This task reads user input from the UART, processes the input,
    and updates shared variables accordingly.

    Valid commands:
    - Integers between -100 and 100: Sets the motor effort.
    - 'calibrate': Initiates sensor calibration.
    - 'cl': Activates closed-loop control.
    - 'i2c': Initiates IMU calibration.
    - 'done': Signals the completion of a calibration step.
    """

    while True:
        # Check for new input from UART
        nb_in.check()
        if nb_in.any():  # If there is any input
            user_input_str = nb_in.get().strip()  # Get and strip the input
            print("Received input:", user_input_str)
            try:
                effort = int(user_input_str)  # Try to convert input to integer
                if -100 <= effort <= 100:
                    print("Storing effort", effort, "in queue...")
                    user_input.put(effort)  # Put the effort into the queue
            except ValueError:  # If input is not an integer, handle as command
                cmd = user_input_str.lower()
                if cmd == "calibrate":
                    print("Initiating sensor calibration")
                    calibrate_sensor.put(True)
                elif cmd == "cl":
                    print("Activating closed-loop control")
                    first_movement.put(True)
                    motor_drive.put(True)
                    motor_control.put(True)
                    line_follow.put(True)
                elif cmd == "i2c":
                    print("Initiating IMU calibration")
                    calibrate_i2c.put(True)
                elif cmd == "done":
                    cali_sensor.put(True)
                else:
                    print("[ERROR] Invalid input. Allowed inputs: integers (-100 to 100), "
                          "'calibrate', 'cl', 'i2c'.")
        # Yield control for 20 ticks before the next check
        yield 20

def task_position_control():
    """
    Task to control the position of the robot.

    This task manages the motor efforts based on user input,
    bumper states, and grid navigation. It also handles backup
    sequences and grid section transitions.
    """

    base_effort = 0
    backup_sequence = None
    backup_timer = None
    while True:
        # Check if backup is needed
        if back_up.get():
            line_follow.put(False)
            motor_drive.put(False)
            motor_control.put(False)

            # Initialize backup sequence if not already done
            if backup_sequence is None:
                backup_sequence = [
                    ('reverse', 10),
                    ('left_pivot', None),
                    ('reverse', 100),
                    ('right_pivot', None),
                    ('forward', 70),
                    ('forward2', 90)
                ]
                backup_timer = backup_sequence.pop(0)  # Get the first action from the sequence

            if backup_timer:
                action, timer_val = backup_timer
                if action == 'reverse' and timer_val > 0:
                    left_motor.set_effort(-20)
                    right_motor.set_effort(-20)
                    backup_timer = (action, timer_val - 1)
                elif action == 'left_pivot':
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    if not heading_recorded.get():
                        pre_turn_heading.put(imu.read_heading())
                        heading_recorded.put(True)
                    left_pivot.put(True)
                    if done_left_pivot.get():
                        left_pivot.put(False)
                        done_left_pivot.put(False)
                        heading_recorded.put(False)
                        backup_timer = backup_sequence.pop(0)
                elif action == 'forward' and timer_val > 0:
                    left_motor.set_effort(30)
                    right_motor.set_effort(52)
                    backup_timer = (action, timer_val - 1)
                elif action == 'forward2' and timer_val > 0:
                    left_motor.set_effort(15)
                    right_motor.set_effort(20)
                    backup_timer = (action, timer_val - 1)
                elif action == 'right_pivot':
                    done_right_pivot.put(False)
                    right_pivot.put(True)
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    if not heading_recorded.get():
                        pre_turn_heading.put(imu.read_heading())
                        heading_recorded.put(True)
                        backup_timer = backup_sequence.pop(0)
                    if done_right_pivot.get():
                        right_pivot.put(False)
                        done_right_pivot.put(False)
                        heading_recorded.put(False)
                        backup_timer = backup_sequence.pop(0)
                else:
                    if backup_sequence:
                        backup_timer = backup_sequence.pop(0)
                    else:
                        back_up.put(False)
                        left_motor.set_effort(0)
                        right_motor.set_effort(0)
                        backup_sequence = None
                        backup_timer = None
                        print("done")
                        left_motor.disable()
                        right_motor.disable()
        else:
            # Get user input for base effort
            if not user_input.empty():
                base_effort = user_input.get()
                print("Updated base effort:", base_effort)
            # If motor drive is active
            if motor_drive.get():
                if first_movement.get() == True:
                    starting_heading.put(imu.read_heading())
                    print("Starting heading:", starting_heading.get())
                    first_movement.put(False)
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
        # Yield control for 20 ticks before the next check
        yield 20

def task_closed_loop_control():
    """
    Task to handle closed-loop control for line following and grid navigation.

    This task uses a PID controller to follow a line and navigate grid sections.
    It also manages transitions between line following and grid navigation states.
    """

    kp = 3
    ki = 0
    kd = 0
    setpoint = 5.5
    integral = 0
    old_timestamp = ticks_us()
    previous_error = 0
    encoder_start_left = 0
    encoder_start_right = 0
    using_encoder_drive = False
    while True:
        # If motor control is active
        if motor_control.get():
            Sensor.read_sensors()  # Read sensor data
            sensed_line_position = float(Sensor.compute_centroid())  # Compute line centroid
            if right_position.get() >= 21000 and on_grid.get() == False:
                on_grid.put(Sensor.check_for_grid())  # Check if on grid
                if on_grid.get() == True and not turn_state.get() == 1:
                    line_follow.put(False)
                    motor_drive.put(False)
                    grid_section.put(False)
                    print("Main sees on grid!")
                    left_motor.set_effort(0)
                    right_motor.set_effort(0)
                    heading_obtained.put(False)
                    grid_section.put(True)
                    print("turn State:", turn_state.get())

            # If in grid section
            if grid_section.get():
                print("now definitely in grid section")
                print(grid_section_state.get())
                line_follow.put(False)
                if grid_section_state.get() == 0:
                    distance_ticks.put(int(grid_distance_in_half * (encoder_ticks_per_rotation / wheel_circumference_in)))
                    if not using_encoder_drive:
                        encoder_start_left = left_position.get()
                        encoder_start_right = right_position.get()
                        using_encoder_drive = True
                        print("using encoder drive")
                        print("left start1:", encoder_start_left)
                        print("right start1:", encoder_start_right)
                    left_progress = abs(left_position.get() - encoder_start_left)
                    print("left progress1:", left_progress)
                    right_progress = abs(right_position.get() - encoder_start_right)
                    correction = imu.read_heading() - (starting_heading.get() - 180)
                    if correction >= 180:
                        correction -= 360
                    elif correction <= -180:
                        correction += 360
                    left_motor_effort.put(int(25 - correction * 0.5))
                    right_motor_effort.put(int(25 + correction * 0.5))
                    motor_drive.put(False)
                    if left_progress >= distance_ticks.get() or right_progress >= distance_ticks.get():
                        grid_section_state.put(1)
                        using_encoder_drive = False
                if grid_section_state.get() == 1:
                    distance_ticks.put(int(grid_distance_in_half * (encoder_ticks_per_rotation / wheel_circumference_in)))
                    if not using_encoder_drive:
                        encoder_start_left = left_position.get()
                        encoder_start_right = right_position.get()
                        using_encoder_drive = True
                        print("using encoder drive")
                        print("left start2:", encoder_start_left)
                        print("right start2:", encoder_start_right)
                    left_progress = abs(left_position.get() - encoder_start_left)
                    print("left progress2:", left_progress)
                    right_progress = abs(right_position.get() - encoder_start_right)
                    correction = imu.read_heading() - (starting_heading.get() - 180)
                    if correction >= 180:
                        correction -= 360
                    elif correction <= -180:
                        correction += 360
                    left_motor_effort.put(int(25 - correction * 0.5))
                    right_motor_effort.put(int(25 + correction * 0.5))
                    motor_drive.put(False)
                    if left_progress >= distance_ticks.get() or right_progress >= distance_ticks.get():
                        grid_section_state.put(2)
                if grid_section_state.get() == 2:
                    using_encoder_drive = False
                    left_motor_effort.put(0)
                    right_motor_effort.put(0)
                    print("First Drive Straight done")
                    grid_section_state.put(3)
                if grid_section_state.get() == 3:
                    print("Turning Now, then drive straight")
                    right_pivot.put(True)
                    pre_turn_heading.put(imu.read_heading())
                    grid_section_state.put(4)
                if grid_section_state.get() == 4 and done_right_pivot.get() == True:
                    distance_ticks.put(int(grid_escape_distance_in * (encoder_ticks_per_rotation / wheel_circumference_in)))
                    if not using_encoder_drive:
                        encoder_start_left = left_position.get()
                        encoder_start_right = right_position.get()
                        using_encoder_drive = True
                        print("using encoder drive")
                        print("left start3:", encoder_start_left)
                        print("right start3:", encoder_start_right)
                    left_progress = abs(left_position.get() - encoder_start_left)
                    print(left_progress)
                    right_progress = abs(right_position.get() - encoder_start_right)
                    left_motor_effort.put(15)
                    right_motor_effort.put(15 + deadzone1)
                    motor_drive.put(False)
                    if left_progress >= distance_ticks.get() or right_progress >= distance_ticks.get():
                        grid_section_state.put(5)
                        print("Second Drive Straight Done")
                if grid_section_state.get() == 5:
                    using_encoder_drive = False
                    grid_section.put(False)
                    line_follow.put(True)
                    motor_drive.put(True)
                    starting_heading.put(imu.read_heading())
                    print(starting_heading.get())
                    print("Exited grid, resuming line following.")

            # Line following logic
            if line_follow.get():
                sample_timestamp = ticks_us()
                dt = max((sample_timestamp - old_timestamp) / 1_000_000, 1e-6)
                error = setpoint - sensed_line_position
                integral += dt * error
                if abs(error) < 0.1:
                    integral *= 0.9
                integral = max(min(integral, 100), -100)
                p_error = kp * error
                i_error = ki * integral
                d_error = kd * (error - previous_error) / dt if dt > 0 else 0
                control_signal = p_error + i_error + d_error
                control_signal = max(min(control_signal, 100), -100)
                actuator_input.put(control_signal)
                old_timestamp = sample_timestamp
                previous_error = error

        # Yield control for 20 ticks before the next check
        yield 20

def task_encoder_read():
    """
    Task to read encoder values and update shared variables.

    This task updates the encoder positions and velocities,
    and stores the values in shared variables.
    """
    while True:
        # Update the encoder values for both left and right encoders
        left_encoder.update()
        right_encoder.update()
        
        # Store the updated positions in shared variables
        left_position.put(int(left_encoder.get_position()))
        right_position.put(int(right_encoder.get_position()))
        
        # Store the updated velocities in shared variables
        left_velocity.put(int(left_encoder.get_velocity()))
        right_velocity.put(int(right_encoder.get_velocity()))
        
        # Yield control for 10 ticks before the next update
        yield 10

def task_calibrate_sensor():
    """
    Task to handle sensor calibration.

    This task guides the user through the process of calibrating
    the sensor array for black, white, and dot values.
    """
    state = 0
    while True:
        # Check if calibration process is initiated
        if calibrate_sensor.get():
            if state == 0:
                # Prompt user to place sensors over a black surface and wait for 'done' input
                print("Place the sensors over a black surface and type 'done'.")
                state = 1
            elif state == 1:
                if cali_sensor.get():
                    # Calibrate for black surface
                    Sensor.calibrate_black()
                    print("Place the sensors over a white surface and type 'done'.")
                    cali_sensor.put(False)
                    state = 2
            elif state == 2:
                if cali_sensor.get():
                    # Calibrate for white surface
                    Sensor.calibrate_white()
                    print("Place the sensors over a dot and type 'done'.")
                    cali_sensor.put(False)
                    state = 3
            elif state == 3:
                if cali_sensor.get():
                    # Calibrate for dot
                    Sensor.calibrate_dot()
                    calibrate_sensor.put(False)
                    state = 0
        # Yield control for 20 ticks before the next check
        yield 20

def task_calibrate_imu_once():
    """
    Task to calibrate the IMU sensor once.

    This task continuously checks if the calibration flag is set. If calibration is required, 
    it prompts the user to move the sensor in all directions until the IMU is fully calibrated.
    Once calibrated, it prints the calibration coefficients and disables the calibration flag.
    """

    calibrated = False
    while True:
        # Check if calibration is needed and not already calibrated
        if calibrate_i2c.get() and not calibrated:
            print("Starting IMU calibration. Please move the sensor in all directions.")
            
            # Get the current calibration status
            calib_status = imu.get_calibration_status()
            print("Calibration status - System: {}, Gyro: {}, Accel: {}, Mag: {}"
                  .format((calib_status >> 6) & 0x03,
                          (calib_status >> 4) & 0x03,
                          (calib_status >> 2) & 0x03,
                          calib_status & 0x03))
            
            # Check if the IMU is fully calibrated
            if (((calib_status >> 6) & 0x03) == 3 and 
                ((calib_status >> 4) & 0x03) == 3 and 
                ((calib_status >> 2) & 0x03) == 3 and 
                (calib_status & 0x03) == 3):
                print("IMU fully calibrated!")
                print("Calibration Coefficients:", imu.get_calibration_data())
                calibrate_i2c.put(False)
                calibrated = True
        # Yield control for 20 ticks before the next check
        yield 20

def task_right_pivot():
    """
    Task to perform a right pivot turn.

    This task checks if a right pivot turn is needed. If so, it calculates the desired heading,
    reads the current heading from the IMU, and adjusts the motor efforts to achieve the desired heading.
    Once the desired heading is reached within a defined threshold, it stops the motors and marks the
    right pivot as done.
    """

    threshold = 6.0
    Kp = 3
    while True:
        # Check if right pivot is needed
        if right_pivot.get():
            # Calculate the desired heading based on pre-turn heading
            desired_heading = pre_turn_heading.get() + 80 
            current_heading = imu.read_heading()
            
            # Calculate the error between desired and current heading
            error = (desired_heading - current_heading + 180) % 360 - 180
            
            # Check if the error is within the threshold
            if abs(error) < threshold:
                left_motor.set_effort(0)
                right_motor.set_effort(0)
                right_pivot.put(False)
                done_right_pivot.put(True)
            else:
                # Adjust motor efforts based on the error
                effort = int(Kp * error)
                effort = max(min(effort, 100), -100)
                left_motor.set_effort(effort+6)
                right_motor.set_effort(-effort-6)
        # Yield control for 20 ticks before the next check
        yield 20 

def task_left_pivot():
    """
    Task to perform a left pivot turn.

    This task checks if a left pivot turn is needed. If so, it calculates the desired heading,
    reads the current heading from the IMU, and adjusts the motor efforts to achieve the desired heading.
    Once the desired heading is reached within a defined threshold, it stops the motors and marks the
    left pivot as done.
    """

    threshold = 6.5
    Kp = 3
    while True:
        # Check if left pivot is needed
        if left_pivot.get():
            # Calculate the desired heading based on pre-turn heading
            desired_heading = pre_turn_heading.get() - 90
            current_heading = imu.read_heading()
            print(desired_heading)
            print(current_heading)
            
            # Calculate the error between desired and current heading
            error = (desired_heading - current_heading + 180) % 360 - 180
            
            # Check if the error is within the threshold
            if abs(error) < threshold:
                left_motor.set_effort(0)
                right_motor.set_effort(0)
                left_pivot.put(False)
                done_left_pivot.put(True)
            else:
                # Adjust motor efforts based on the error
                effort = int(Kp * error)
                effort = max(min(effort, 100), -100)
                left_motor.set_effort(effort+6)
                right_motor.set_effort(-effort-6)
        # Yield control for 20 ticks before the next check
        yield 20 

def task_bump_check():
    """
    Task to check for bumps.

    This task continuously checks the bumper sensor for any bumps. If a bump is detected,
    it sets the bump flag and triggers the back-up action.
    """

    while True:
        # Check the bumper sensor for any bumps
        bumper.check_bump()  
        
        # If a bump is detected, set the bump flag and trigger back-up action
        if bumper.bump_flag == 0: 
            print("bump_flag set")
            back_up.put(True)
        # Yield control for 20 ticks before the next check
        yield 20


""" --------------------- Scheduler --------------------- """

task1 = Task(task_encoder_read, name='Encoder Read', priority=8, period=10, profile=True, trace=False)
task2 = Task(task_user_command, name='User Command', priority=2, period=20, profile=True, trace=False)
task3 = Task(task_closed_loop_control, name='Closed Loop Control', priority=7, period=20, profile=True, trace=False)
task4 = Task(task_position_control, name='Position Control', priority=6, period=20, profile=True, trace=False)
task5 = Task(task_calibrate_sensor, name='Calibrate Sensor', priority=0, period=20, profile=True, trace=False)
task6 = Task(task_calibrate_imu_once, name='IMU Calibration', priority=1, period=20, profile=True, trace=False)
task7 = Task(task_bump_check, name='Bump Check', priority=5, period=20, profile=True, trace=False)
task8 = Task(task_right_pivot,name="Right Pivot", priority=4, period=20, profile=True, trace=False)
task9 = Task(task_left_pivot,name="Left Pivot", priority=3, period=20, profile=True, trace=False)

task_list.append(task1)
task_list.append(task2)
task_list.append(task3)
task_list.append(task4)
task_list.append(task5)
task_list.append(task6)
task_list.append(task7)
task_list.append(task8)
task_list.append(task9)

print("Starting Scheduler")
while True:
    try:
        task_list.pri_sched()
    except KeyboardInterrupt:
        left_motor.disable()
        right_motor.disable()
        break
    except Exception as e:
        left_motor.disable()
        right_motor.disable()
        raise e