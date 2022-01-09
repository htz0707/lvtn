#! /usr/bin/env python
import rospy
from iq_gnc.py_gnc_functions import *
from iq_gnc.PrintColours import *

def main():

    rospy.init_node('takeoff', anonymous=True)
    drone = gnc_api()
    drone.wait4connect()
    drone.set_mode("GUIDED")
    drone.initialize_local_frame()

    drone.takeoff(2)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        exit()
