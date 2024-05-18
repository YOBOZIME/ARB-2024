import cv2
import numpy as np
from gpiozero import Motor
import time

# Define motors using GPIO pins
motor_left = Motor(forward=3, backward=4)
motor_right = Motor(forward=17, backward=27)

# Function to move the robot forward
def move_forward():
    motor_left.forward()
    motor_right.forward()

# Function to move the robot backward
def move_backward():
    motor_left.backward()
    motor_right.backward()

# Function to turn the robot left
def turn_left():
    motor_left.backward()
    motor_right.forward()

# Function to turn the robot right
def turn_right():
    motor_left.forward()
    motor_right.backward()

# Function to stop the robot
def stop_robot():
    motor_left.stop()
    motor_right.stop()

def detect_initial_color():
    cap = cv2.VideoCapture(0)
    detected_color = None

    try:
        while True:
            ret, frame = cap.read()
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            
            color_ranges = {
                'White': ([0, 0, 200], [179, 40, 255]),  # White color range
                'Black': ([0, 0, 0], [179, 255, 40]),     # Black color range
                'Yellow': ([20, 100, 100], [30, 255, 255]),  # Yellow color range
                'Cyan': ([85, 100, 100], [105, 255, 255]),   # Cyan color range
                'Magenta': ([140, 100, 100], [170, 255, 255]),  # Magenta color range
                'Red': ([0, 100, 100], [10, 255, 255]),    # Red color range
                'Blue': ([90, 50, 50], [130, 255, 255]),   # Blue color range
                'Green': ([36, 25, 25], [70, 255, 255])    # Green color range
            }
            
            for color, (lower, upper) in color_ranges.items():
                lower_color = np.array(lower)
                upper_color = np.array(upper)
                mask = cv2.inRange(hsv, lower_color, upper_color)
                
                contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in contours:
                    area = cv2.contourArea(contour)
                    if area > 100:  # Adjust the threshold as needed
                        x, y, w, h = cv2.boundingRect(contour)
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        detected_color = color
            
            cv2.imshow('Initial Color Detector', frame)
            
            if detected_color:
                print(f"Detected Initial Color: {detected_color}")
                return detected_color
            
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

def follow_line_and_detect_qr(processed_qr_codes):
    cap = cv2.VideoCapture(0)
    qr_detector = cv2.QRCodeDetector()
    try:
        while True:
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)  # Detect black line

            # Find contours of the black line
            contours, _ = cv2.findContours(binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Assuming the largest contour is the line
                c = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Calculate the center of the contour
                center_x = x + w // 2
                frame_center = frame.shape[1] // 2

                # Basic proportional control to follow the line
                if center_x < frame_center - 20:
                    print("Adjusting left")
                    turn_left()
                elif center_x > frame_center + 20:
                    print("Adjusting right")
                    turn_right()
                else:
                    print("Moving forward")
                    move_forward()
            else:
                stop_robot()
            
            # Detect QR code in the frame
            data, bbox, _ = qr_detector.detectAndDecode(frame)
            if bbox is not None:
                bbox = bbox.astype(int)
                cv2.polylines(frame, [bbox], True, (0, 255, 0), 3)
                if data and data not in processed_qr_codes:
                    print("QR Code Data:", data)
                    processed_qr_codes.add(data)
                    stop_robot()
                    return data  # Return the movement command detected from the QR code
            
            # Display the frame
            cv2.imshow('Line Following and QR Code Detection', frame)
            
            # Check for 'q' to quit
            key = cv2.waitKey(1)
            if key == ord('q'):
                break  # Break the loop if 'q' is pressed
    finally:
        cap.release()
        cv2.destroyAllWindows()

def execute_command(command):
    if command.lower() == 'forward':
        print("Moving forward...")
        move_forward()
        time.sleep(2)
    elif command.lower() == 'backward':
        print("Moving backward...")
        move_backward()
        time.sleep(2)
    elif command.lower() == 'left':
        print("Turning left...")
        turn_left()
        time.sleep(2)
    elif command.lower() == 'right':
        print("Turning right...")
        turn_right()
        time.sleep(2)
    else:
        print(f"Unknown command: {command}")
    stop_robot()

def main():
    print("Line Follower Robot - Color and Movement Detection")
    print("Press 'q' to quit")

    # Detect the initial color
    initial_color = detect_initial_color()
    
    if initial_color:
        print(f"Stored color: {initial_color}")

    processed_qr_codes = set()

    while True:
        # Follow the line and detect QR codes to get movement commands
        qr_command = follow_line_and_detect_qr(processed_qr_codes)
        if qr_command:
            print("QR Code Detected:", qr_command)
            execute_command(qr_command)
        else:
            print("QR Code not detected.")
        
        # Wait for 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
