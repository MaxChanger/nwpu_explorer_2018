#! /usr/bin/python
import sys,os
def cur_file_dir():
     path = sys.path[0]
     
     if os.path.isdir(path):
         return path
     elif os.path.isfile(path):
         return os.path.dirname(path)
path = cur_file_dir()

#ip = raw_input("Please input \"ip\" = ")
ip = "127.0.0.1"
#port = raw_input("Please input \"port\" = ")
port = "1234"

print "command: padsp " + path + "/client " + ip +" "+ port

print "listener is start"
os.system("padsp " + path + "/client " + ip +" "+ port)
