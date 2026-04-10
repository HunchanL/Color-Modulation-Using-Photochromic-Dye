"""
Library for controlling the motors of the laser steering robot
Using Tic 36v4 stepper motor controller
"""
import subprocess
import yaml
import time

'''
Motor control command wrapping function
'''
def ticcmd(*args):
    """Runs the ticcmd command to control the stepper motor."""
    return subprocess.check_output(['ticcmd'] + list(args))

##########################################################################################
'''
Conversion Factors
'''
def volume_to_steps(v: float, stepmode: int = 1):
    """Converts a given volume in mL to the corresponding number of steps for the stepper motor."""
    # Assuming 1 mL corresponds to 1000 steps at full step mode (1x)
    if v < 0: 
        dir = -1
    else:
        dir = 1
    rev = 0.827 * abs(v) + 0.2557 # Linear regression based on empirical data
    steps = -rev * 200 * stepmode * dir # 200 steps per revolution, adjusted for step mode
    return int(steps)
'''
Functions to set up motor
'''
def energize_motor(motor_num):
    """Energizes the stepper motor to enable movement."""
    ticcmd('-d', str(motor_num), '--energize')
    print("Motor energized.")

def deenergize_motor(motor_num):
    """De-energizes the stepper motor to disable movement."""
    ticcmd('-d', str(motor_num), '--deenergize')
    print("Motor de-energized.")

def clear_driver_errors(motor_num):
    """Clears any errors in the stepper motor driver."""
    ticcmd('-d', str(motor_num), '--clear-driver-error')
    print("Driver errors cleared.")

def reset_motor(motor_num):
    """Resets the motor to its default settings."""
    ticcmd('-d', str(motor_num), '--reset')
    print("Motor reset to default settings.")

def reset_motor_position(motor_num):
    """Resets the current position of the stepper motor to 0 without moving it."""
    ticcmd('-d', str(motor_num), '--halt-and-set-position', '0')  # Force current position to be 0
    print("Motor position reset to 0.")

def set_current_limit(motor_num, current_limit):
    """Sets the current limit for the stepper motor."""
    ticcmd('-d', str(motor_num), '--current', str(current_limit))
    print(f"Current limit set to {current_limit}.")

def set_step_mode(motor_num, STEP_MODE, MAX_SPEED, MAX_ACCELERATION):
    """Sets the step mode and dynamically adjusts max speed."""
    ticcmd('-d', str(motor_num), '--step-mode', str(STEP_MODE))
    # BASE_STEP_SPEED = 2000000
    # MAX_SPEED = 50000000#BASE_STEP_SPEED * STEP_MODE
    # MAX_ACCELERATION = 640000 
    
    ticcmd('-d', str(motor_num), '--step-mode', str(STEP_MODE))  # Set the step mode
    ticcmd('-d', str(motor_num), '--max-speed', str(MAX_SPEED))  # Update the controller’s max speed limit
    ticcmd('-d', str(motor_num), '--max-accel', str(MAX_ACCELERATION))
    ticcmd('-d', str(motor_num), '--max-decel', str(MAX_ACCELERATION))
    print(f"Step mode set to {STEP_MODE}, Max speed updated to {MAX_SPEED}.")

def set_exit_safe_start(motor_num):
    """Sets the exit safe start limit for the stepper motor."""
    ticcmd('-d', str(motor_num), '--exit-safe-start')

def set_halt_and_set_position(motor_num):
    """Sets the target position for the stepper motor."""
    ticcmd('-d', str(motor_num), '--halt-and-set-position', str(0))
    print(f"Target position set to {0}.")

##########################################################################################
'''
Functions to get status of the motor
'''
def get_controller_id():
    """Fetches the ID of the controller."""
    serial_numbers = yaml.safe_load(ticcmd('--list'))
    return serial_numbers

def get_step_mode(motor_num):
    """Fetches the current step mode from the stepper motor and maps it to an integer."""
    status = yaml.safe_load(ticcmd('-d', str(motor_num), '-s', '--full'))
    step_mode_str = status['Step mode']

    # Step mode mapping
    step_mode_map = {
        "Full step": 1,
        "1/2 step": 2,
        "1/4 step": 4,
        "1/8 step": 8,
        "1/16 step": 16,
        "1/32 step": 32
    }
    return step_mode_map.get(step_mode_str, 1)  # Default to full step (1) if unknown

def get_motor_info(motor_num):
    """Fetches the motor information from the stepper motor."""
    status = yaml.safe_load(ticcmd('-d', str(motor_num), '-s', '--full'))
    position = int(status['Current position'])
    velocity = int(status['Current velocity'])
    return position, velocity

# Function to get the current position of the stepper motor
def get_motor_position(motor_num):
    """Fetches the current stepper motor position."""
    status = yaml.safe_load(ticcmd('-d', str(motor_num), '-s', '--full'))
    return int(status['Current position'])

# Function to get the actual velocity from the motor
def get_actual_velocity(motor_num):
    """Fetches the actual velocity the stepper motor is running at."""
    status = yaml.safe_load(ticcmd('-d', str(motor_num), '-s', '--full'))
    return int(status['Current velocity'])  # Get the real applied speed

##########################################################################################
'''
Functions to run the motor
'''
def init_motor_setup(motor_nums, STEP_MODE, CURRENT_LIMIT, MAX_SPEED, MAX_ACCELERATION):
    """Initialize the motor controller and motors."""
    for n in motor_nums:
        reset_motor(n)
        set_step_mode(n, STEP_MODE, MAX_SPEED, MAX_ACCELERATION)
        set_current_limit(n, CURRENT_LIMIT)
        energize_motor(n)
        clear_driver_errors(n)
        set_halt_and_set_position(n)
        set_exit_safe_start(n)
    print('Initialized motor controller and motors setup.')

def run_target_position(motor_nums, target_positions):
    """Command the motor to run upto the target position."""
    ticcmd('-d', str(motor_nums), '--exit-safe-start', '--position', str(target_positions))
    #print(f"Motor {motor_nums} Target Position: {target_positions}.")

def run_target_velocity(motor_nums, target_velocities):
    """Command the motor to run at a specific velocity."""
    ticcmd('-d', str(motor_nums), '--exit-safe-start', '--velocity', str(target_velocities))
    print(f"Motor {motor_nums} Target velocity: {target_velocities}.")

def run_home_position(motor_nums):
    """Command the motor to run to the home position.""" 
    motor_pos_current = [0, 0, 0]      
    motor_vel_current = [0, 0, 0]      
    for _ in range(12):
        for i, serial_num in enumerate(motor_nums):
            run_target_position(serial_num, 0)
        time.sleep(1)
        
    for i, serial_num in enumerate(motor_nums):
        motor_pos_current[i], motor_vel_current[i] = get_motor_info(serial_num)
        deenergize_motor(serial_num)
    print('Motor Position:', motor_pos_current)
