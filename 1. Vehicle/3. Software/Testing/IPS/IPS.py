#!/usr/bin/env python

from argparse import ArgumentParser
import os
import cv2
import numpy as np
import apriltag # https://github.com/Tinker-Twins/AprilTag

################################################################################

def autodrive_ips(input_stream='IPS_Test.mp4',
                  output_stream=True,
                  display_stream=True,
                  detection_window_name='AutoDRIVE IPS',
                 ):

    '''
    AutoDRIVE IPS

    Args:   input_stream [str]: Camera IP address or footage name to run detection algorithm on
            output_stream [bool]: Boolean flag to save/not stream annotated with detections
            display_stream [bool]: Boolean flag to display/not stream annotated with detections
            detection_window_name [str]: Title of displayed (output) tag detection window
    '''

    parser = ArgumentParser(description='AutoDRIVE IPS')
    apriltag.add_arguments(parser)
    options = parser.parse_args()

    '''
    Set up a reasonable search path for the apriltag DLL.
    Either install the DLL in the appropriate system-wide
    location, or specify your own search paths as needed.
    '''

    detector = apriltag.Detector(options, searchpath=apriltag._get_dll_path())

    video = cv2.VideoCapture(input_stream)

    output = None

    if output_stream:
        width = int(video.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(video.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(video.get(cv2.CAP_PROP_FPS))
        codec = cv2.VideoWriter_fourcc(*'XVID')
        output_path = str(os.path.split(input_stream)[1]).replace(str(os.path.splitext(input_stream)[1]), '.avi')
        output = cv2.VideoWriter(output_path, codec, fps, (width, height))

    while(video.isOpened()):

        success, frame = video.read()
        if not success:
            break

        result, overlay = apriltag.detect_tags(frame,
                                               detector,
                                               camera_params=(3156.71852, 3129.52243, 359.097908, 239.736909),
                                               tag_size=0.0762,
                                               vizualization=2,
                                               verbose=0,
                                               annotation=False
                                              )

        if len(result)==8:
            map_x_cam = result[1][0][3] # result[1] is map apriltag pose w.r.t. camera
            map_y_cam = -result[1][1][3]
            vehicle_x_cam = result[5][0][3] - 0.035125 # result[5] is vehicle apriltag pose w.r.t. camera
            vehicle_y_cam = -result[5][1][3]
            vehicle_position = [vehicle_x_cam - map_x_cam,
                                vehicle_y_cam - map_y_cam]
            print(vehicle_position)

        if output_stream:
            output.write(overlay)

        if display_stream:
            cv2.imshow(detection_window_name, overlay)
            if cv2.waitKey(1) & 0xFF == ord(' '): # Press space bar to terminate
                break

################################################################################

if __name__ == '__main__':
    autodrive_ips()
