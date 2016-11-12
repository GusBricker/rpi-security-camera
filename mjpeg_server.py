#!/usr/bin/env python

import Queue
import threading
import cv2
from BaseHTTPServer import BaseHTTPRequestHandler, HTTPServer
from SocketServer import ThreadingMixIn

class MJPEGHandler(BaseHTTPRequestHandler):
    def __init__(self, *args):
        self.frame_queue = args[2].frame_queue
        BaseHTTPRequestHandler.__init__(self, *args)

    def write_header(self):
        self.send_response(200)
        self.separator = "seperate"
        self.send_header("Content-type","multipart/x-mixed-replace;boundary=%s" % self.separator)
        self.end_headers()
        self.wfile.write("--%s\r\n" % self.separator)

    def write_frame(self, frame):
        cv2mat = cv2.imencode(".jpeg", frame, (cv2.IMWRITE_JPEG_QUALITY,80))[1]
        buf = cv2mat.tostring()

        self.wfile.write("Content-type: image/jpeg\r\n")
        self.wfile.write("\r\n")
        self.wfile.write(buf)
        self.wfile.write("\r\n--%s\r\n" % self.separator)

    def do_GET(self):
        try:
            print "Got request: " + self.path
            stream_index = 0
            if self.path.endswith("thresh"):
                stream_index = 1
            elif self.path.endswith("delta"):
                stream_index = 2

            self.write_header()

            while True:
                try:
                    frame = self.frame_queue.get(block=True, timeout=1)[stream_index]
                except Exception, e:
                    print str(e)
                    continue

                if frame == None:
                    return

                self.write_frame(frame)

            return
        except IOError:
            self.send_error(404,'File Not Found: %s' % self.path)

class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    pass
