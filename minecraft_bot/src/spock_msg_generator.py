#! /usr/bin/env python

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import controller_msg#, mc_msg
from time import sleep
import sys


def main():
    global spock_pub

    # Initialize ROS node
    rospy.init_node('controller')

    # Create publisher to send out ControllerMsg messages
    spock_pub = rospy.Publisher('controller_data', controller_msg)

    # Subscribe to data stream from Minecraft (Spock)
    #rospy.Subscriber('mc_data', mc_msg, mc_callback, queue_size=1)
    
    rospy.loginfo("Controller node inititalized.")
    while not rospy.is_shutdown():
        message = controller_msg()
        #print(current_pos)
        message.action = 1
        
        rospy.loginfo('sending: action ' + str(message.action))
        spock_pub.publish(message)

        # wait 5 seconds between commands
        rospy.sleep(5.)


#def mc_callback(data):

if __name__ == '__main__':
    main()

