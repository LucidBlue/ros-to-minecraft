"""

created by Bradley Sheneman

composes actions and sends them to SendAction.py

Hybrid Spock plugin and ROS node that receives movement commands from ROS and pieces together
a frame-by-frame action. Has knowledge of what high level actions like "jump" and "sprint"
are composed of at the low level



"""

import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import movement_msg

from spock.utils import pl_announce
from spock.mcp import mcdata
from spock.utils import Vec3
from spockextras.plugins.actionutils import Vec5

import math, Queue


import logging
logger = logging.getLogger('spock')


# receives movement commands from ROS. currently does not have error checking, but will eventually
# return value of: success, failure, in-progress to the planner node

class CreateActionCore:
    def __init__(self):

        self.target = None
    
    
    def setTarget(self, data):

        # jump can cause some weird behavior. bot will continue to 'jump' until it reaches its target
        # it is supposed to be used as a separate action from moving, but the x, y, z are still there
        # to allow movement while jumping. use at your own risk!
        
        self.jump = bool(data.jump)
        self.speed = data.speed
        
        x = float(data.x)
        y = float(data.y)
        z = float(data.z)
        
        self.target = Vec3(x=x, y=y, z=z)
                
        # print ("target pos: %2f, %2f, %2f") %(
        #        self.target.x,
        #        self.target.y,
        #        self.target.z)



@pl_announce('CreateAction')
class CreateActionPlugin:
    
    def __init__(self, ploader, settings):
        
        self.net = ploader.requires('Net')
        self.clinfo = ploader.requires('ClientInfo')
        self.phys = ploader.requires('NewPhysics')

        ploader.reg_event_handler('client_tick', self.client_tick)
        ploader.reg_event_handler('action_tick', self.action_tick)
        ploader.reg_event_handler('cl_position_update', self.handle_position_update)
        
        self.cac = CreateActionCore()
        ploader.provides('CreateAction', self.cac)    
        self.startMovementNode()

    def startMovementNode(self):

        rospy.init_node('movement_listener')
        print("movement listener node initialized")

        rospy.Subscriber('movement_cmd', movement_msg, self.cac.setTarget)
    
    
    def client_tick(self, name, data):
        
        self.net.push_packet('PLAY>Player Position', self.clinfo.position.get_dict())
    
    
    def handle_position_update(self, name, data):
        
        self.net.push_packet('PLAY>Player Position and Look', data.get_dict())
 
    def getAngle(self, x, z):

        # from positive x axis (east)
        # where z is south
        dx = x - self.clinfo.position.x
        dz = z - self.clinfo.position.z
        
        if dx == 0:
            angle = math.copysign(180, dz)
        else:
            angle = math.degrees(math.atan(dz/dx))
            if dx < 0:
                angle += 180
        
        return angle


    def action_tick(self, name, data):
        
        self.doMovement()


    def doMovement(self):
        
        # as long as we have not reached our target, update position and calculate new frame
        # also push the frame to 'actions' queue to make available for the action sender
        
        """
        if (     self.cac.target.x != math.floor(self.clinfo.position.x)
                and self.cac.target.y != math.floor(self.clinfo.position.y)
                and self.cac.target.z != math.floor(self.clinfo.position.z)):
	"""
        if (self.cac.target != None):
            direction = self.getAngle(self.cac.target.x, self.cac.target.z)
            print ("current pos: " + str(self.clinfo.position))
            self.phys.move(direction, self.cac.speed, self.cac.jump)
