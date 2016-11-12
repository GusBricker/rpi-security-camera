#!/usr/bin/env python

import time
import threading
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera

class FrameGrabber(object):
    def __init__(self, frame_queue, video_rate, xres, yres):
        self.frame_queue = frame_queue
        self.video_rate = video_rate
        self.xres = xres
        self.yres = yres

    def start(self):
        self.can_run = True
        t_cap = threading.Thread(target=self.on_capture_frames)
        t_cap.daemon = True
        t_cap.start()

    def stop(self):
        self.can_run = False

    def on_capture_frames(self):
        print "Grabber starting"
        camera = PiCamera()
        camera.resolution = (self.xres, self.yres)
        camera.framerate = self.video_rate
        raw_capture = PiRGBArray(camera, size=(self.xres, self.yres))

        time.sleep(0.1)

        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            if self.can_run == False:
                break

            image = frame.array
            try:
                self.frame_queue.put(image, block=True, timeout=0.1)
            except Exception, e:
                pass

            # clear the stream in preparation for the next frame
            raw_capture.truncate(0)

        print "Grabber stopping"
