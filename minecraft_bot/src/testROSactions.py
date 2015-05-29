#!/usr/bin/env python
"""

create a loop of test actions to test the new movement plugins

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import movement_msg



mv_pub = rospy.Publisher('movement_cmd', movement_msg, queue_size=1)

rospy.init_node('movement_sender')
print("movement sender node initialized")

"""

move around in a triangle
<-25, 13, -40>
<-25, 13, -60>
<-10  , 13, -50>


"""

positions = [
        (-25., 13., -40.),
        (-25., 13., -60.),
        (-10., 13., -50.)]

while not rospy.is_shutdown():

    message = movement_msg()
    
    for coords in positions:
        message.jump = False
        message.speed = 1
        message.x = coords[0]
        message.y = coords[1]
        message.z = coords[2]
    
        mv_pub.publish(message)
        print ("sent command: %2f, %2f, %2f") %(message.x, message.y, message.z)
        rospy.sleep(8.)

