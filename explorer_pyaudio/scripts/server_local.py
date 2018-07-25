#!/usr/bin/env python

import socket
import sys
# import thread
import pyaudio
import wave
import rospy
import numpy as np

rospy.init_node("local_listener")

ip_address = rospy.get_param('~address')
ip_port = rospy.get_param('~port')

address = (ip_address, ip_port)
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(address)


CHUNK = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

l = pyaudio.PyAudio()
stream = l.open(format=FORMAT,
                channels=CHANNELS, 
                rate=RATE, 
                output=True)

rospy.loginfo("Ready to hear")

# def ExplorerHear():
while True:
    data,addr = s.recvfrom(2048)
    wavedata = np.fromstring(data, dtype = np.int16)
    if max(wavedata) > 16384 :
        rospy.logerr("i heard!")
    stream.write(data)

    # rospy.loginfo("good status")
    # rospy.loginfo("thread one")
        # print("stream status = ", stream.write(data))
    if not data:
        print "client has exist"
        continue
    # print "received size:", sys.getsizeof(data), "from", addr

# def ExplorerRespond():
#     # while True:
#     rospy.loginfo("good status")

# try:
#     thread.start_new_thread( ExplorerHear,(),)
#     thread.start_new_thread( ExplorerRespond,(), ) # cannot without empty parameters
# except:
#     rospy.loginfo("error")

# multiple thread method have a long time latency so i decided not to use

s.close
