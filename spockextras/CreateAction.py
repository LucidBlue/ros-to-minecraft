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
import math, Queue

import logging
logger = logging.getLogger('spock')


# receives movement commands from ROS. currently does not have error checking, but will eventually
# return value of: success, failure, in-progress to the planner node

class CreateActionCore:
    def __init__(self):
        self.target = Vec3()



@pl_announce('CreateAction')
class CreateActionPlugin:
    
    def __init__(self, ploader, settings):
        
        self.phys = ploader.requires('NewPhysics')
        self.act = ploader.requires('SendAction')
        self.cac = CreateActionCore()
        ploader.provides('CreateAction', self.cac)    
        
        self.mainLoop()

    def mainLoop(self):

        rospy.init_node('movement_listener')
        print("movement listener node initialized")

        rospy.Subscriber('movement_cmd', movement_msg, self.doMovement)
        
        rospy.spin()

    def doMovement(self, data):

        # jump can cause some weird behavior. bot will continue to 'jump' until it reaches its target
        # it is supposed to be used as a separate action from moving, but the x, y, z are still there
        # to allow movement while jumping. use at your own risk!
        jump = bool(data.jump)
        speed = data.speed
        self.cac.target.x = float(data.x)
        self.cac.target.y = float(data.y)
        self.cac.target.z = float(data.z)
        print ("received command: %2f, %2f, %2f") %(data.x, data.y, data.z)
        # as long as we have not reached our target, update position and calculate new frame
        # also push the frame to 'actions' queue to make available for the action sender
        while (     self.cac.target.x != math.floor(self.act.position.x)
                and self.cac.target.y != math.floor(self.act.position.y)
                and self.cac.target.z != math.floor(self.act.position.z)):
	
            direction = self.getAngle(self.cac.target.x, self.cac.target.z)
            self.phys.move(direction, speed, jump)
            print ("moving in direction: %2f") %(direction)
            self.act.actions.put(self.phys.vec)


    def getAngle(self, x, z):

        # from positive x axis (east)
        # where z is south
        dx = x - self.act.position.x
        dz = z - self.act.position.z
        
        angle = math.degrees(math.atan(-dz/dx))
        if dx < 0:
            angle += 180
        
        return angle
