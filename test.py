import cv2
import numpy as np
import time

# Placeholder functions for motor control
def move_forward():
    print("Moving forward")
    # Add motor control logic to move the robot forward

def move_backward():
    print("Moving backward")
    # Add motor control logic to move the robot backward

def turn_left():
    print("Turning left")
    # Add motor control logic to turn the robot left

def turn_right():
    print("Turning right")
    # Add motor control logic to turn the robot right

def stop():
    print("Stopping")
    # Add motor control logic to stop the robot

def detect_qr_code(cap):
    qr_detector = cv2.QRCodeDetector()
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        # Detect QR code in the frame
        data, bbox, _ = qr_detector.detectAndDecode(frame)
        
        if bbox is not None:
            bbox = bbox.astype(int)
            cv2.polylines(frame, [bbox], True, (0, 255, 0), 3)
            if data:
                print("QR Code Data:", data)
                return data  # Return the detected QR code data
            
        # Display the frame with QR code bounding box
        cv2.imshow('QR Code Scanner', frame)
        
        # Check for 'q' to quit
        if cv2.waitKey(1) == ord('q'):
            break

def detect_color(cap, target_color):
    color_ranges = {
        'Red': ([0, 100, 100], [10, 255, 255]),
        'Blue': ([90, 50, 50], [130, 255, 255]),
        'Green': ([36, 25, 25], [70, 255, 255])
    }
    
    lower_color, upper_color = color_ranges.get(target_color, ([0, 0, 0], [0, 0, 0]))
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(lower_color), np.array(upper_color))
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Adjust the threshold as needed
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                return True  # Detected the target color rectangle
        
        cv2.imshow('Color Detector', frame)
        
        if cv2.waitKey(1) == ord('q'):
            break
    
    return False

def main():
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    
    print("Line Follower Robot - Color Detection")
    print("Press 'q' to quit")
    
    # Detect starting color using QR code
    starting_color = detect_qr_code(cap)
    if starting_color:
        print("Starting color detected:", starting_color)
    else:
        print("QR Code not detected. Exiting.")
        return
    
    while True:
        # Move forward and detect QR codes
        move_forward()
        qr_data = detect_qr_code(cap)
        if qr_data:
            print("QR Code Instruction:", qr_data)
            if qr_data == "right":
                turn_right()
            elif qr_data == "left":
                turn_left()
            elif qr_data == "forward":
                move_forward()
            elif qr_data == "backward":
                move_backward()
        
        # Check for the target color to stop
        if detect_color(cap, starting_color):
            stop()
            print("Arrived at the target color. Stopping.")
            break

        time.sleep(0.1)

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
