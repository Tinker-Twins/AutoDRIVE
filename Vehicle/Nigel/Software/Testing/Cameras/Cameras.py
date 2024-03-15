'''
Testing two CSI cameras connected to a NVIDIA Jetson Nano Developer Kit (Rev B01) using OpenCV.

This script connects with two CSI camera modules and reads their frames at the specified configuration.
The camera streams are each read in their own thread, as when done sequentially there is substanntial lag.
This script also displays the frames captured from them side-by-side and saves the footage from both the
cameras independently to the local directory.

Note: CSI camera drivers and OpenCV v3.1.1 are included in the base image (JetPack 4.3+).
'''

import cv2
import threading
import numpy as np

# gstreamer_pipeline returns a GStreamer pipeline for capturing from the CSI camera
# Flip the image by setting the flip_method (most common values: 0 and 2)
# display_width and display_height determine the size of each camera pane in the window on the screen

camera_0 = None
camera_1 = None

class CSI_Camera:

    def __init__ (self) :
        # Initialize instance variables
        # OpenCV video capture element
        self.video_capture = None
        # The last captured image from the camera
        self.frame = None
        self.grabbed = False
        # Thread where the video capture runs
        self.read_thread = None
        self.read_lock = threading.Lock()
        self.running = False

    def open(self, gstreamer_pipeline_string):
        try:
            self.video_capture = cv2.VideoCapture(gstreamer_pipeline_string, cv2.CAP_GSTREAMER)
        except RuntimeError:
            self.video_capture = None
            print("Unable to open camera")
            print("Pipeline: " + gstreamer_pipeline_string)
            return
        # Grab the first frame to start the video capturing
        self.grabbed, self.frame = self.video_capture.read()

    def start(self):
        if self.running:
            print('Video capturing is already running')
            return None
        # Create a thread to read the camera image
        if self.video_capture != None:
            self.running=True
            self.read_thread = threading.Thread(target=self.update)
            self.read_thread.start()
        return self

    def stop(self):
        self.running=False
        self.read_thread.join()

    def update(self):
        # This is the thread to read images from the camera
        while self.running:
            try:
                grabbed, frame = self.video_capture.read()
                with self.read_lock:
                    self.grabbed=grabbed
                    self.frame=frame
            except RuntimeError:
                print("Could not read frame from camera")

    def read(self):
        with self.read_lock:
            frame = self.frame.copy()
            grabbed=self.grabbed
        return grabbed, frame

    def save(self, file_name):
        width = int(self.video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.video_capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = int(self.video_capture.get(cv2.CAP_PROP_FPS))
        codec = cv2.VideoWriter_fourcc(*'XVID')
        output_path = file_name + '.avi'
        output = cv2.VideoWriter(output_path, codec, fps, (width, height))
        return output

    def release(self):
        if self.video_capture != None:
            self.video_capture.release()
            self.video_capture = None
        # Kill the thread
        if self.read_thread != None:
            self.read_thread.join()

# Configuring CSI camera settings on Jetson Nano through gstreamer
def gstreamer_pipeline(
    sensor_id=0,
    sensor_mode=3, # Select sensor_mode 3 (1280x720, 59.9999 fps)
    capture_width=1280,
    capture_height=720,
    display_width=1280,
    display_height=720,
    framerate=30,
    flip_method=2,
):
    return (
        "nvarguscamerasrc sensor-id=%d sensor-mode=%d ! "
        "video/x-raw(memory:NVMM), "
        "width=(int)%d, height=(int)%d, "
        "format=(string)NV12, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            sensor_mode,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def camera_test():
    camera_0 = CSI_Camera()
    camera_0.open(
        gstreamer_pipeline(
            sensor_id=0,
            sensor_mode=3,
            flip_method=2,
            display_height=360,
            display_width=640,
        )
    )
    camera_0.start()

    camera_1 = CSI_Camera()
    camera_1.open(
        gstreamer_pipeline(
            sensor_id=1,
            sensor_mode=3,
            flip_method=2,
            display_height=360,
            display_width=640,
        )
    )
    camera_1.start()

    cv2.namedWindow("Camera Test", cv2.WINDOW_AUTOSIZE)

    output_0 = camera_0.save('Camera_0')
    output_1 = camera_1.save('Camera_1')

    if (not camera_0.video_capture.isOpened()
        or not camera_1.video_capture.isOpened()):
        # Cameras did not open, or no camera attached
        print("Unable to open any cameras")
        SystemExit(0)

    while cv2.getWindowProperty("Camera Test", 0) >= 0 :
        _ , frame_0 = camera_0.read()
        _ , frame_1 = camera_1.read()
        output_0.write(frame_0)
        output_1.write(frame_1)        
        camera_frames = np.hstack((frame_0, frame_1))
        cv2.imshow("Camera Test", camera_frames)
        keyCode = cv2.waitKey(5) & 0xFF
        # Stop the program on the ESC key
        if keyCode == 27:
            break

    camera_0.stop()
    camera_0.release()
    camera_1.stop()
    camera_1.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_test()

