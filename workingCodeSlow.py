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

import board
import time
import pwmio
import digitalio
from adafruit_motor import servo
from adafruit_simplemath import map_range, constrain
from circuitpython_gizmo import Gizmo

gizmo = Gizmo()

pwm_freq = 50
min_pulse = 1000
max_pulse = 2000
servo_range = 360

motor_left = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_1, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

motor_right = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_4, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

motor_task_raise_arm = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_3, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

motor_task_arm_extension_retraction = servo.ContinuousServo(
    pwmio.PWMOut(gizmo.MOTOR_2, frequency=pwm_freq),
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

servo_task_habitat_modules_dropper = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_1, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

servo_task_claw_open_and_close = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_2, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

servo_SeatBelt = servo.Servo(
    pwmio.PWMOut(gizmo.SERVO_3, frequency=pwm_freq),
    actuation_range=servo_range,
    min_pulse=min_pulse,
    max_pulse=max_pulse
)

builtin_led = digitalio.DigitalInOut(board.GP25)
builtin_led.direction = digitalio.Direction.OUTPUT

TANK_MODE = 0
ARCADE_MODE = 1

seatbeltDown = False

mode = TANK_MODE
arm_sensors = False
prev_mode_button = False
prev_back_button = False
prev_b_button = False


modules_dropped = False
modules_close = 360
modules_open = 0

Jiggle_Count = 0

claw_is_closed = False
claw_close_angle = 0
claw_open_angle = 360

print("Program started")
while True:
    builtin_led.value = not builtin_led.value

    gizmo.refresh()

    if gizmo.buttons.back and not prev_mode_button:
        mode = ARCADE_MODE if mode == TANK_MODE else TANK_MODE
        print(f"Mode changed to {'ARCADE_MODE' if mode == ARCADE_MODE else 'TANK_MODE'}")
    prev_mode_button = gizmo.buttons.back

    if mode == TANK_MODE:
        motor_left.throttle = map_range(gizmo.axes.left_y, 0, 255, 1.0, -1.0)
        motor_right.throttle = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)

    elif mode == ARCADE_MODE:
        speed = map_range(gizmo.axes.right_x, 0, 255, 1.0, -1.0)
        steering = map_range(gizmo.axes.right_y, 0, 255, -1.0, 1.0)
        motor_left.throttle = constrain(speed - steering, 1.0, -1.0)
        motor_right.throttle = constrain(speed + steering, 1.0, -1.0)
   
    # Raise and lower arm motor controlarm
    if gizmo.buttons.left_shoulder:
        print("Left shoulder pressed")
        motor_task_raise_arm.throttle = 1.0  # Raise arm
        servo_SeatBelt.angle = 180
        print("Seatbelt activated")
    elif gizmo.buttons.left_trigger:
        print("Left trigger pressed")
        motor_task_raise_arm.throttle = -1.0  # Lower arm
        servo_SeatBelt.angle = 180
        print("Seatbelt activated")
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

    if gizmo.buttons.start and not prev_start_button:
        if prev_start_button:
            servo_SeatBelt.angle = 180
            prev_start_button = False
            print("Seatbelt down")
        else:
            servo_SeatBelt.angle = 180
            prev_start_button = True
            print("Start button pressed - Seatbelt down")
    prev_start_button = gizmo.buttons.start
    print("Start button pressed - Seatbelt down")
    prev_start_button = gizmo.buttons.start
    
    if gizmo.buttons.a:
        motor_task_raise_arm.throttle = 1
        time.sleep(.2)
        motor_task_raise_arm.throttle = -0.5
        time.sleep(.2)

    time.sleep(0.01)
