#!/usr/bin/env python

'''
Welcome to the ArUco Marker Pose Estimator!

This program:
  - Estimates the pose of an ArUco Marker
'''

from __future__ import print_function  # Python 2/3 compatibility
import cv2  # Import the OpenCV library
import cv2.aruco as aruco
import numpy as np  # Import Numpy library
from scipy.spatial.transform import Rotation as R
import math  # Math library
## my own
from webcam_stream import WebcamStream
from recorder import fake_cam, FakeCamStream
from Distance import euler_from_quaternion,calculations, display_menu,verify_int


# Dictionary that was used to generate the ArUco marker
aruco_dictionary_name = "DICT_ARUCO_ORIGINAL"

# The different ArUco dictionaries built into the OpenCV library.
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL
}

# Side length of the ArUco marker in meters
aruco_marker_side_length = 0.0785

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'

def setup():
    # Check that we have a valid ArUco marker
    if ARUCO_DICT.get(aruco_dictionary_name, None) is None:
        print("[INFO] ArUCo tag of '{}' is not supported".format(
            args["type"]))
        sys.exit(0)

    # Load the camera parameters from the saved file
    cv_file = cv2.FileStorage(
        camera_calibration_parameters_filename, cv2.FILE_STORAGE_READ)
    mtx = cv_file.getNode('K').mat()
    dst = cv_file.getNode('D').mat()
    cv_file.release()

    # Side length of the ArUco marker in meters
    aruco_marker_side_length = float(input("Enter aruco_marker_side_length, default value is 0.0785 : "))
    # Load the ArUco dictionary
    print("[INFO] detecting '{}' markers...".format(
        aruco_dictionary_name))
    ### BIG VERSION CHANGES
    # Initialize the detector parameters using default values
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    parameters = cv2.aruco.DetectorParameters()  # Use the constructor to create an instance
    return aruco_dict,parameters,aruco_marker_side_length,mtx,dst



def main():
    """
    Main method of the program.
    """
    aruco_dict,parameters,aruco_marker_side_length,mtx,dst = setup()

    # Start the video stream
    example = cv2.imread('frame_0.jpg')
    marker_ids = None
    puase = 0
    while True:
        while (puase == 0):

            # Capture frame-by-frame
            frame  = webcam_stream.read()
            if webcam_stream.stopped :
                break
            #frame = example.copy()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # Detect ArUco markers in the video frame
            ### BIG VERSION CHANGES
            corners, marker_ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            # Check that at least one ArUco marker was detected

            if marker_ids is None:
                print("No markers detected")
            else:
                #webcam_stream.pause()
                #puase = 1
                print("found one!")
                # Draw a square around detected markers in the video frame
                cv2.aruco.drawDetectedMarkers(frame, corners, marker_ids)
                #for corner, id in zip(corners, marker_ids):
                    #print(f"Marker ID: {marker_ids[0]}")
                    #print(f"Marker Corners: {corner}")

                # Get the rotation and translation vectors
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    corners,
                    aruco_marker_side_length,
                    mtx,
                    dst)


                # Draw the axes on the marker
                #BIG CHANGE
                for i, marker_id in enumerate(marker_ids):
                    frame = cv2.drawFrameAxes(frame, mtx, dst, rvecs[i], tvecs[i], 0.05)
                    # Store the translation (i.e. position) information
                    calculations(tvecs,rvecs,i)

            # Display the resulting frame
            cv2.imshow('frame', frame)
            out.write(frame) ## Save
            key = cv2.waitKey(1)
            if key & 0xFF == ord('p'):
                print("stop pouse")
                webcam_stream.pause()
                puase = not puase
                display_menu()
                choice = verify_int("Enter your choice: ")
                if choice == 1:
                    print(marker_ids)
                if choice == 3:
                    break

            # If "q" is pressed on the keyboard,
            # exit this loop
            if key & 0xFF == ord('q'):
                break
        cv2.imshow('frame', frame)
        key = cv2.waitKey(1)
        if key & 0xFF == ord('p'):
            print("stop pouse")
            webcam_stream.pause()
            puase = not puase

        # If "q" is pressed on the keyboard,
        # exit this loop
        if key & 0xFF == ord('q'):
            print("Goodbye")
            break

    # Close down the video stream
    cv2.destroyAllWindows()



if __name__ == '__main__':
    #SOURCE = 'webcam'
    SOURCE = 0
    # Initialize and start the webcam input stream
    if SOURCE == 'webcam':
        webcam_stream = WebcamStream(stream_id=0)  # 0 is usually the default camera
        print("Using webcam")
        webcam_stream.start()
        webcam_stream.start_recording(output_file='output.avi')
    else:
        webcam_stream = FakeCamStream('./input.avi')
        print("Using fake cam")
        webcam_stream.start()
        #webcam_stream.start_recording(output_file='output_video.avi')
    frame = webcam_stream.read()
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output_with_marks.mp4', fourcc, 80.0, (640, 480))
    print(__doc__)

    main()
    print("End")
    # Cleanup operations
    webcam_stream.stop_recording() if SOURCE == 'webcam' else None
    webcam_stream.stop()
    webcam_stream.vcap.release()
    out.release() if SOURCE != 'webcam' else None
    cv2.destroyAllWindows()