"""
the central ROS node that controls Spock. Will register all events from ROS and transfer them over to
its corresponding spock plugin

all ROS related Spock events are prefaced with 'ros_'
the subscriber callbacks should be named by whatever is after the '_' in camelCase

currently the ROS message is passed along unmodified. this may need to change later

"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import movement_msg, place_block_msg, mine_block_msg


from spock.mcp import mcdata
from spock.utils import pl_announce

import logging
logger = logging.getLogger('spock')


class SpockControlCore:

    def __init__(self):

        self.target = None



@pl_announce('SpockControl')
class SpockControlPlugin:
    
    def __init__(self, ploader, settings):
        
        self.event = ploader.requires('Event')
        
        # simply load all of the plugins
        ploader.requires('NewMovement')
        ploader.requires('MineAndPlace')

        self.scc = SpockControlCore()
        ploader.provides('SpockControl', self.scc)
        
        self.initSpockControlNode()


    def initSpockControlNode(self):
        
        rospy.init_node('spock_controller')
	print("spock control node initialized")
	
        # subscribe to Spock related data streams from ROS
	rospy.Subscriber('movement_data', movement_msg, self.moveTo, queue_size=1)
	rospy.Subscriber('mine_block_data', mine_block_msg, self.mineBlock, queue_size=1)
	rospy.Subscriber('place_block_data', place_block_msg, self.placeBlock, queue_size=1)

    # ROS Subscriber callbacks simply pass data along to the Spock event handlers
    def moveTo(self, data):

        self.event.emit('ros_moveto', data)
        print("emit event moveto")

    
    def mineBlock(self, data):

        self.event.emit('ros_mineblock', data)
    
    
    def placeBlock(self, data):

        self.event.emit('ros_placeblock', data)
