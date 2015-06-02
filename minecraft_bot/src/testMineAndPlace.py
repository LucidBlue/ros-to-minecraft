#!/usr/bin/env python
"""

create a loop of test actions to test the new mine and place plugin

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import mine_block_msg, place_block_msg


m_pub = rospy.Publisher('mine_block_data', mine_block_msg, queue_size=1)
p_pub = rospy.Publisher('place_block_data', place_block_msg, queue_size=1)

rospy.init_node('mine_and_place_sender')
print("mine_and_place sender node initialized")

positions = [
        (-27, 13, -40),
        (-27, 13, -41),
        (-27, 13, -42)]

while not rospy.is_shutdown():

    message_mine = mine_block_msg()
    message_place = place_block_msg()
    
    for coords in positions:
        
        message_mine.status = 0
        message_mine.face = 1
        message_mine.x = coords[0]
        message_mine.y = coords[1]
        message_mine.z = coords[2]
    
        m_pub.publish(message_mine)
        print ("sent command mine_block at: %2f, %2f, %2f") %(message_mine.x, message_mine.y, message_mine.z)
        
        rospy.sleep(8.)

        message_place.id = -1
        message_place.dir = 1
        message_place.loc_x = coords[0]
        message_place.loc_y = coords[1]
        message_place.loc_z = coords[2]
        message_place.pos_x = 8
        message_place.pos_y = 16
        message_place.pos_z = 8
    
        p_pub.publish(message_place)
        print ("sent command place_block at: %2f, %2f, %2f") %(
                    message_place.loc_x,
                    message_place.loc_y,
                    message_place.loc_z)
        
        rospy.sleep(3.)

