#!/usr/bin/env python

import os
import threading
from pushbullet import Pushbullet

class PushBulletSender(object):
    def __init__(self, api_key, video_path, msg):
        self.api_key = api_key
        self.video_path = video_path
        self.msg = msg

    def send(self):
        t = threading.Thread(target=self.on_send)
        t.daemon = True
        t.start()

    def on_send(self):
        print "Initializing Pushbullet"
        pb = Pushbullet(self.api_key)

        print "Opening Video"
        with open(self.video_path, "rb") as data:
            print "Uploading Video"
            file_data = pb.upload_file(data, self.msg)

        print "Sending Video"
        push = pb.push_file(**file_data)
        del pb

        print "Pushbullet Done!"

        print "Cleaning up old video!"
        os.remove(self.video_path)
