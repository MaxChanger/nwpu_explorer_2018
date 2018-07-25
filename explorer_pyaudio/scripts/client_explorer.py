#!/usr/bin/env python

import socket
import sys
import pyaudio
import wave
import rospy


#address = ('192.168.188.123', 12345)
rospy.init_node("explorer_talker")

ip_address = rospy.get_param('~address')
ip_port = rospy.get_param('~port')

address = (ip_address, ip_port)
s =socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

CHUNK = 256
FORMAT = pyaudio.paInt16
CHANNELS = 2
RATE = 44100

p = pyaudio.PyAudio()
stream = p.open(format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK)

rospy.loginfo("stream start")

#print("stream start")

while True:
    data = stream.read(CHUNK)
    frames = []
    frames.append(data)
    s.sendto(data, address)
    # print("send")

rospy.loginfo("stopped")
#print("stopped")
stream.stop_stream()
stream.close()
p.terminate()
s.close()
