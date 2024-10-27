"""
This is the main controller program for our robot developed by TEACH Robotics 2024
This code was written by Paul Estrada and Toby Zipper (Programming)

This code has two control modes: 'Tank Mode' and 'Arcade Mode'. The Start
button on the gamepad switches the robot between these two modes. The robot will start in 'Tank Mode'.

Here are the controls for Tank Mode:
Left Joystick Up/Down    - Motor 1 Fwd/Rev
Right Joystick Up/Down   - Motor 2 Fwd/Rev

Here are the controls for Arcade Mode:
Left Joystick Up/Down    - Robot Fwd/Rev
Left Joystick Left/Right - Robot Turn Left/Right

These controls work in both modes:
Right Trigger            - Motor 3 Forward
Right Shoulder Button    - Motor 4 Reverse
Left Trigger             - Servo 1 to 0 degrees
Left Shoulder Button     - Servo 1 to 90 degrees

"""

import board  # Import board module for pin definitions
import time  # Import time module for delays
import pwmio  # Import PWM I/O module for PWM control
import digitalio  # Import digital I/O module for digital pin control
from adafruit_motor import servo  # Import servo module from Adafruit motor library
from adafruit_simplemath import map_range, constrain  # Import map_range and constrain functions from Adafruit simplemath library
from circuitpython_gizmo import Gizmo  # Import Gizmo class from circuitpython_gizmo library

gizmo = Gizmo()  # Initialize Gizmo object

pwm_freq = 50  # Set PWM frequency to 50 Hertz
min_pulse = 1000  # Set minimum pulse width to 1000 milliseconds
max_pulse = 2000  # Set maximum pulse width to 2200 milliseconds
servo_range = 360  # Set servo range to 180 degrees

# Motor and Servo Setup
# Left Motor
motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
# Right Motor
motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Motor arm setup
# Raise arm motor
motor_task_raise_arm = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
# Extend/retract arm motor
motor_task_arm_extension_retraction = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Servos setup
# Habitat modules dropper servo
servo_task_habitat_modules_dropper = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

# Claw open and close servo
servo_task_claw_open_and_close = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_2, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)
# 
# Bumper switches setup
# Bumper switch inside the claw
bumper_in_claw = digitalio.DigitalInOut(gizmo.GPIO_1)

bumper_in_claw.switch_to_input()

# Bumper switch for wall detection
bumper_for_wall = digitalio.DigitalInOut(gizmo.GPIO_2)
bumper_for_wall.switch_to_input()

# Bumper switch for arm in position
bumper_arm_in = digitalio.DigitalInOut(gizmo.GPIO_3)
bumper_arm_in.switch_to_input()

# Bumper switch for arm out position
bumper_arm_out = digitalio.DigitalInOut(gizmo.GPIO_4)
bumper_arm_out.switch_to_input()

# Bumper switch for arm down position
bumper_arm_down = digitalio.DigitalInOut(gizmo.GPIO_5)
bumper_arm_down.switch_to_input()

# Configure the built-in LED pin as an output
builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

# Modes setup
TANK_MODE = 0  # Define constant for Tank Mode
ARCADE_MODE = 1  # Define constant for Arcade Mode
SPLIT_ARCADE = 2  # Define constant for Split Arcade Mode

mode = TANK_MODE  # Set default starting mode to Tank Mode
autonomus_runing = False  # Initialize autonomous running flag
arm_sensors = False  # Initialize arm sensors flag
prev_mode_button = False  # Initialize previous mode button state
prev_back_button = False  # Initialize previous back button state
prev_b_button = False

precice = False
precision_precent = 1

modules_dropped = False
modules_close = 0
modules_open = 180

claw_is_closed = False  # Initialize claw state
claw_close_angle = 0  # Define claw close angle
claw_open_angle = 360  # Define claw open angle

print("Program started")
while True:
    builtin_led.value = not builtin_led.value  # Toggle the built-in LED state

    gizmo.refresh()  # Refresh the Gizmo object to get the latest inputs

    if gizmo.buttons.b and not prev_b_button:
        if precice == True:
            precice = False
            precision_precent = 1
        else:
            precice = True
            precision_precent = .5
    prev_b_button = gizmo.buttons.b 

    # Check if the back button is pressed to switch modes
    if gizmo.buttons.back and not prev_mode_button:
        mode = ARCADE_MODE if mode == TANK_MODE else TANK_MODE  # Toggle between Tank Mode and Arcade Mode
        print(f"Mode changed to {'ARCADE_MODE' if mode == ARCADE_MODE else 'TANK_MODE'}")
    prev_mode_button = gizmo.buttons.back  # Update previous mode button state

    # Control motors based on the current mode
    if mode == TANK_MODE:
        motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, 1.0, -1.0)  *precision_precent # Map left joystick Y-axis to left motor throttle
        motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0) *precision_precent # Map right joystick Y-axis to right motor throttle



    elif mode == ARCADE_MODE:
        speed = map_range(gizmo.axes.right_x, 0, 255, 1.0, -1.0)  # Map left joystick Y-axis to speed
        steering = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)  # Map left joystick X-axis to steering
        motor_left.throttle = constrain(speed - steering, 1.0, -1.0) *precision_precent # Calculate left motor throttle based on speed and steering
        motor_right.throttle = constrain(speed + steering, 1.0, -1.0)*precision_precent  # Calculate right motor throttle based on speed and steering
    
    elif mode == SPLIT_ARCADE:
        speed = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
        steering = map_range(gizmo.axes.left_x, 0, 255, -1.0, 1.0)
        motor_left.throttle = constrain(speed - steering, -1.0, 1.0)
        motor_right.throttle = constrain(speed + steering, -1.0, 1.0)
   
    # Raise and lower arm motor controlarm
    if gizmo.buttons.left_shoulder:
        print("Left shoulder pressed")
        motor_task_raise_arm.throttle = 1.0  # Raise arm
    elif gizmo.buttons.left_trigger:
        print("Left trigger pressed")
        motor_task_raise_arm.throttle = -1.0  # Lower arm
    else:
        motor_task_raise_arm.throttle = 0.0  # or 0.0, keep the arm raised (hopefully)

    # Arm extension and retraction motor control
    if gizmo.buttons.right_shoulder:
        motor_task_arm_extension_retraction.throttle = 1.0  # Extend arm
        print("right shoulder pressed")
    elif gizmo.buttons.right_trigger:
        motor_task_arm_extension_retraction.throttle = -1.0  # Retract arm
        print("right trigger pressed")
    else:
        motor_task_arm_extension_retraction.throttle = 0.0

    if gizmo.buttons.x and not prev_x_button:
        if claw_is_closed:
            servo_task_claw_open_and_close.angle = claw_open_angle  # Open claw
            claw_is_closed = False
            print("Button x pressed - Claw opened")
        else:
            servo_task_claw_open_and_close.angle = claw_close_angle  # Close claw
            claw_is_closed = True
            print("Button x pressed - Claw closed")
    prev_x_button = gizmo.buttons.x  # Update previous X button state
    print("Button x pressed - Claw opened")
    prev_x_button = gizmo.buttons.x  # Update previous X button state

    if gizmo.buttons.y and not prev_y_button:
        if modules_dropped:
            servo_task_habitat_modules_dropper.angle = modules_open # Reset habitat modules dropper
            modules_dropped = False
            print("Button y pressed - Habitat modules dropper reset")
        else:
            servo_task_habitat_modules_dropper.angle = modules_close  # Drop habitat modules
            modules_dropped = True
            print("Button y pressed - Habitat modules dropped")
    prev_y_button = gizmo.buttons.y  # Update previous Y button state
    print("Button y pressed - Modules dropped")
    prev_y_button = gizmo.buttons.y

    # Check if the start button is pressed to enable autonomous running
    if gizmo.buttons.start and not prev_start_button:
        autonomus_runing = True  # Enable autonomous running
        print("Autonomous running enabled")
    prev_start_button = gizmo.buttons.start  # Update previous start button state

    time.sleep(0.01)  # Tiny delay to prevent crashing the Gizmo board and overheating it
