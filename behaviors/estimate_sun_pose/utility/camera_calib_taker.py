import cv2
import os
import time

def capture_images():
    # 1. Setup the directory
    output_dir = 'calibration_images'
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # 2. Open the camera
    # Try 0 first. If you have multiple cameras, change this to 1 or 2.
    cap = cv2.VideoCapture(0)

    # Set resolution to camera's max capability for best calibration
    # (Common values: 1920x1080, 1280x720, 640x480)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    print("--- Camera Calibration Capture Tool ---")
    print("Press [SPACE] to save an image.")
    print("Press [Q] to quit.")

    count = 0
    
    # Check existing files to avoid overwriting
    existing_files = os.listdir(output_dir)
    count = len([f for f in existing_files if f.endswith('.jpg') or f.endswith('.png')])

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Make a copy for display so we can draw text on it without ruining the saved image
        display_frame = frame.copy()

        # Add info text overlay
        cv2.putText(display_frame, f"Images Captured: {count}", (30, 50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(display_frame, "Press SPACE to save, Q to quit", (30, 90), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        cv2.imshow('Calibration Capture', display_frame)

        key = cv2.waitKey(1) & 0xFF

        # Save on SPACE bar
        if key == 32: 
            img_name = os.path.join(output_dir, f"calib_{count:04d}.jpg")
            cv2.imwrite(img_name, frame)
            print(f"{img_name} saved!")
            count += 1
            
            # Visual flash effect
            cv2.rectangle(display_frame, (0,0), (frame.shape[1], frame.shape[0]), (255,255,255), cv2.FILLED)
            cv2.imshow('Calibration Capture', display_frame)
            cv2.waitKey(50) # Pause briefly to show the flash

        # Quit on 'q'
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("Capture finished.")

if __name__ == "__main__":
    capture_images()