import cv2


def capture_frames():
    # Start the webcam
    cap = cv2.VideoCapture(0)  # '0' is typically the default value for the first webcam.
    cap.set(cv2.CAP_PROP_EXPOSURE, -1)
    if not cap.isOpened():
        print("Error: Webcam could not be accessed.")
        return

    frame_counter = 0  # Counter to keep track of saved frames

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Failed to grab frame")
            break

        # Display the resulting frame
        cv2.imshow('Webcam Live', frame)

        # Wait for key press for 1ms, check if '1' was pressed
        key = cv2.waitKey(1) & 0xFF
        if key == ord('1'):
            # Save the frame
            frame_filename = f"frame_{frame_counter}.jpg"
            cv2.imwrite(frame_filename, frame)
            print(f"Saved {frame_filename}")
            frame_counter += 1

        # Check if 'q' was pressed to quit
        elif key == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_frames()
