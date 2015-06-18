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

        ploader.reg_event_handler('ros_time_update', sendUpdateTime)
        ploader.reg_event_handler('ros_new_dimension', sendNewDimension)
        ploader.reg_event_handler('ros_chunk_data', sendChunkData)
        ploader.reg_event_handler('ros_chunk_bulk', sendChunkBulk)
        ploader.reg_event_handler('ros_block_update', sendBlockUpdate)
        ploader.reg_event_handler('ros_world_reset', sendWorldReset)

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

        pub_time =      rospy.Publisher('time_data', time_msg, queue_size = 1)
        pub_dim =       rospy.Publisher('dimension_data', time_msg, queue_size = 1)
        pub_chunk =     rospy.Publisher('chunk_data', chunk_msg, queue_size = 1000)
        pub_block =     rospy.Publisher('block_data', block_msg, queue_size = 1000)
        pub_bulk =      rospy.Publisher('chunk_bulk', chunk_bulk_msg, queue_size = 1000)
        pub_wstate =    rospy.Publisher('world_state', world_state_msg, queue_size = 1)

    ### ROS Subscriber callbacks simply pass data along to the Spock event handlers
    def moveTo(self, data):

        self.event.emit('ros_moveto', data)
        print("emitting event ros_moveto")

    
    def mineBlock(self, data):

        self.event.emit('ros_mineblock', data)
        print("emitting event ros_mineblock")
    
    
    def placeBlock(self, data):

        self.event.emit('ros_placeblock', data)
        print("emitting event ros_placeblock")


    
    ### Perception handlers. Invoke ROS Publishers

    def sendTimeUpdate(self, data):
        
        pub_time.publish(data)
    
    
    def sendNewDimension(self, data):
        
        pub_dim.publish(data)
    
    
    def sendChunkData(self, data):
        
        pub_chunk.publish(data)
    
    
    def sendChunkBulk(self, data):
        
        pub_bulk.publish(data)
    
    
    def sendBlockUpdate(self, data):
        
        pub_block.publish(data)
    
    
    def sendTimeUpdate(self, data):
        
        pub_wstate.publish(data)

