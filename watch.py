#!/usr/bin/env python

import argparse
import cv2
import time
import threading
import Queue
import RPi.GPIO as GPIO
import os
import os.path
from frame_grabber import FrameGrabber
from mjpeg_server import ThreadedHTTPServer, MJPEGHandler
from distutils.version import LooseVersion, StrictVersion
from pb_sender import PushBulletSender

def memory_usage_resource():
    import resource
    rusage_denom = 1024.
    mem = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / rusage_denom
    return mem

def generate_filename(extension):
    return time.strftime("%Y%m%d-%H%M%S") + "." + extension

def on_run(args):
    pir_active_state = (GPIO.HIGH if args.pir_active == True else GPIO.LOW)
    pir_active_px = (GPIO.PUD_DOWN if args.pir_active == True else GPIO.PUD_UP)
    video_extension = "avi"
    video_format = 'XVID'
    pre_num_frames = args.framerate * args.pre_trigger_video_length
    post_num_frames = args.framerate * args.post_trigger_video_length
    frame_stack = []

    if not os.path.exists(args.video_path):
        os.makedirs(args.video_path)
    elif not os.path.isdir(args.video_path):
        print "args.video_path must be a directory!"
        exit(1)

    print "Initializing GPIO"
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(args.pir_gpio_num, GPIO.IN, pull_up_down=pir_active_px)

    print "Initializing OpenCV"
    base_frame = None
    pir_detected = False
    video_frame_count = 1

    format = cv2.cv.CV_FOURCC(*video_format)
    last_rebase_timestamp = time.time()

    grabber_frame_queue = Queue.Queue(10)
    grabber = FrameGrabber(grabber_frame_queue, args.framerate, args.xres, args.yres)
    grabber.start()

    server_frame_queue = Queue.Queue(1)
    server = ThreadedHTTPServer((args.web_addr, args.web_port), MJPEGHandler)
    server.frame_queue = server_frame_queue
    t_web = threading.Thread(target=server.serve_forever)
    t_web.daemon = True
    t_web.start()

    print "Running"
    try:
        while True:
            # Detected via gpio activation
            if not pir_detected:
                if GPIO.input(args.pir_gpio_num) == pir_active_state:
                    video_frame_count = 1
                    pir_detected = True
                    video_name = generate_filename(video_extension)
                    video_path = os.path.join(args.video_path, video_name)
                    video_writer = cv2.VideoWriter(video_path, format, args.framerate, (args.xres, args.yres))
                    print "PIR Object Detected: " + video_path

            try:
                frame = grabber_frame_queue.get(block=True, timeout=0.01)
            except:
                continue

            if args.debug == True:
                cv2.imshow("Security Feed", frame)

            objects_in_frame = 0
            thresh = None
            frame_delta = None

            # Resize the frame, convert it to grayscale, and blur it
            if args.threshold and pir_detected:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                if args.blur:
                    gray = cv2.GaussianBlur(gray, (args.blur, args.blur), 0)

                # Base frame
                time_diff = time.time() - last_rebase_timestamp
                if base_frame is None or time_diff > args.rebase_period:
                    base_frame = gray
                    last_rebase_timestamp = time.time()
                    print "Getting a new base frame"
                else:
                    # Compute the absolute difference between the current frame and
                    # first frame
                    frame_delta = cv2.absdiff(base_frame, gray)
                    thresh = cv2.threshold(frame_delta, args.threshold, 255, cv2.THRESH_BINARY)[1]

                    # Dilate the thresholded image to fill in holes, then find contours
                    # on thresholded image
                    thresh = cv2.dilate(thresh, None, iterations=2)
                    (cnts, _) = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,
                        cv2.CHAIN_APPROX_SIMPLE)

                    # Loop over the contours
                    for c in cnts:
                        # If the contour is too small, ignore it
                        if cv2.contourArea(c) < args.min_area:
                            continue

                        objects_in_frame += 1

                        if args.object_highlight:
                            # Compute the bounding box for the contour, draw it on the frame,
                            # and update the text
                            (x, y, w, h) = cv2.boundingRect(c)
                            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    print "Objects in Frame: " + str(objects_in_frame)


            try:
                server_frame_queue.get_nowait()
            except:
                pass
            server_frame_queue.put_nowait([frame, thresh, frame_delta])

            frame_stack.insert(0, frame)
            print "Frame Stack Length: " + str(len(frame_stack))
            print "Memory Usage: " + str(memory_usage_resource()) + "MB"

            # Once detected, record into a video stream and send via Pushbullet
            if pir_detected == True:
                if len(frame_stack) > 0:
                    f = frame_stack.pop()
                    video_writer.write(f)
                    del f

                    video_frame_count += 1
                    video_required_frames = pre_num_frames + post_num_frames
                    print "Video Frames: " + str(video_frame_count) + ", Needed: " + str(video_required_frames)
                    if video_frame_count > video_required_frames:
                        pir_detected = False

                        del video_writer

                        pb = PushBulletSender(args.api_key, video_path, video_name)
                        pb.send()

            elif len(frame_stack) >= pre_num_frames:
                frame_stack.pop()

    except KeyboardInterrupt:
        print "Quitting"


    print "Cleaning up GPIO's"
    GPIO.cleanup()
    grabber.stop()

    print "Cleaning up OpenCV"
    cv2.destroyAllWindows()

parser = argparse.ArgumentParser(description="Captures a video from a webcam when a person is detected in the camera's vision or\n"
                                             "a GPIO is triggered on the Raspberry PI.\n"
                                             "Then sends the video to the user via Pushbullet.")
parser.add_argument('-pre_trigger_video_length', help='Length of video to create in seconds before a trigger event.', type=int, default=10, required=False)
parser.add_argument('-post_trigger_video_length', help='Length of video to create in seconds after a trigger event.', type=int, default=10, required=False)
parser.add_argument('-framerate', help='Framerate to capture at.', type=int, default=20, required=False)
parser.add_argument('-xres', help='X resolution to capture at.', type=int, default=640, required=False)
parser.add_argument('-yres', help='Y resolution to capture at.', type=int, default=480, required=False)
parser.add_argument('-api_key', help='Pushbullet API key.', required=True)
parser.add_argument('-pir_gpio_num', help='GPIO channel (BCM mode) for PIR sensor to wait for.', type=int, required=True)
parser.add_argument('-pir_active_high', help='Active high for when the capture should begin.', dest='pir_active', required=False, action='store_true')
parser.add_argument('-pir_active_low', help='Active low for when the capture should begin.', dest='pir_active', required=False, action='store_false')
parser.add_argument('-video_path', help='Local path to save videos.', required=True)
parser.add_argument('-object_highlight', help='Enable object highlighting on detection', dest='object_highlight', required=False, action='store_true')
parser.add_argument('-min_area', help='Minimum area to detect.', type=int, required=True)
parser.add_argument('-threshold', help='Threshold to a person.', type=int, required=False)
parser.add_argument('-blur', help='Size of blur to apply to each frame.', type=int, required=False)
parser.add_argument('-rebase_period', help='Frequency in seconds to update our base frame (providing the system isnt in the middle of a detection).', type=int, required=True)
parser.add_argument('-web_addr', help='Web address to bind too, can be 0.0.0.0 or localhost or any other IPV4 address', required=True)
parser.add_argument('-web_port', help='Web port to bind too, usually 8080.', type=int, required=True)
parser.add_argument('-debug', help='Enable debugging.', required=False, action='store_true')
parser.set_defaults(pir_active=False)
parser.set_defaults(func=on_run)
args = parser.parse_args()
args.func(args)
