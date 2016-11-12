#!/bin/bash

sudo apt-get update

sudo apt-get install python-opencv python-picamera || exit 1
sudo pip install RPi.GPIO pushbullet.py "picamera[array]" || exit 1
