import cv2
import numpy as np
import RPi.GPIO as GPIO
import time

# Setup GPIO pins
GPIO.setmode(GPIO.BCM)

# Motor pins (using the same configuration as the first challenge)
motor1_pin1 = 3
motor1_pin2 = 4
motor2_pin1 = 17
motor2_pin2 = 27

# Line sensor and ultrasonic sensor pins
line_sensor_left = 22
line_sensor_right = 10  # Adjusted to avoid conflict with motor pins
trigger_pin = 5
echo_pin = 6

# Set up GPIO pins as outputs for motors
GPIO.setup(motor1_pin1, GPIO.OUT)
GPIO.setup(motor1_pin2, GPIO.OUT)
GPIO.setup(motor2_pin1, GPIO.OUT)
GPIO.setup(motor2_pin2, GPIO.OUT)

# Set up GPIO pins as inputs for sensors
GPIO.setup(line_sensor_left, GPIO.IN)
GPIO.setup(line_sensor_right, GPIO.IN)
GPIO.setup(trigger_pin, GPIO.OUT)
GPIO.setup(echo_pin, GPIO.IN)

# Function to set motor direction and speed
def set_motor_speed(motor, speed):
    if speed > 0:
        GPIO.output(motor[0], GPIO.HIGH)
        GPIO.output(motor[1], GPIO.LOW)
    elif speed < 0:
        GPIO.output(motor[0], GPIO.LOW)
        GPIO.output(motor[1], GPIO.HIGH)
    else:
        GPIO.output(motor[0], GPIO.LOW)
        GPIO.output(motor[1], GPIO.LOW)

# Function to move the robot forward
def move_forward():
    set_motor_speed((motor1_pin1, motor1_pin2), 100)
    set_motor_speed((motor2_pin1, motor2_pin2), 100)

# Function to move the robot backward
def move_backward():
    set_motor_speed((motor1_pin1, motor1_pin2), -100)
    set_motor_speed((motor2_pin1, motor2_pin2), -100)

# Function to turn the robot left
def turn_left():
    set_motor_speed((motor1_pin1, motor1_pin2), -100)
    set_motor_speed((motor2_pin1, motor2_pin2), 100)

# Function to turn the robot right
def turn_right():
    set_motor_speed((motor1_pin1, motor1_pin2), 100)
    set_motor_speed((motor2_pin1, motor2_pin2), -100)

# Function to stop the robot
def stop_robot():
    set_motor_speed((motor1_pin1, motor1_pin2), 0)
    set_motor_speed((motor2_pin1, motor2_pin2), 0)

# Measure distance using the ultrasonic sensor
def measure_distance():
    GPIO.output(trigger_pin, True)
    time.sleep(0.00001)
    GPIO.output(trigger_pin, False)

    start_time = time.time()
    stop_time = time.time()

    while GPIO.input(echo_pin) == 0:
        start_time = time.time()

    while GPIO.input(echo_pin) == 1:
        stop_time = time.time()

    time_elapsed = stop_time - start_time
    distance = (time_elapsed * 34300) / 2
    return distance

# Follow the line using IR sensors
def follow_line():
    left_value = GPIO.input(line_sensor_left)
    right_value = GPIO.input(line_sensor_right)

    if left_value == GPIO.LOW and right_value == GPIO.HIGH:
        turn_right()
    elif left_value == GPIO.HIGH and right_value == GPIO.LOW:
        turn_left()
    else:
        move_forward()

# Detect green color using the camera
def detect_green():
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_green = np.array([40, 40, 40])
        upper_green = np.array([80, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        distance = measure_distance()
        if distance < 20:
            stop_robot()
            print("Obstacle detected, stopping")
            while measure_distance() < 20:
                time.sleep(0.1)

        if cv2.countNonZero(mask) > 0:
            follow_line()
            print("Green detected, moving")
        else:
            stop_robot()
            print("No green detected, stopping")

        cv2.imshow('frame', frame)
        cv2.imshow('mask', mask)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        detect_green()
    except KeyboardInterrupt:
        GPIO.cleanup()
