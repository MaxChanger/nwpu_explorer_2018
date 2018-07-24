#! /usr/bin/python
import sys,os
def cur_file_dir():
     path = sys.path[0]
     
     if os.path.isdir(path):
         return path
     elif os.path.isfile(path):
         return os.path.dirname(path)
path = cur_file_dir()

#port = raw_input("Please input \"port\" = ")
port = "1234"

print "command: padsp " + path + "/server " + port

print "player is start"
os.system("padsp " + path + "/server " + port)
