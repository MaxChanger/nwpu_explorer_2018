#!/usr/bin/env python

import sys

from rqt_gui.main import Main

main = Main()
sys.exit(main.main(sys.argv, standalone='explorer_app_new/Explorer_App_New'))
