#!/usr/local/bin/python

"""
created by Bradley Sheneman

heavily modified from "move.py", a helper plugin created by the SpockBot devs

combined ROS node and Spock plugin to control bot movement. Receives a movement command from ROS
and sends it to Spock. command should have a packet ID. It only needs to be binary, as the
next command will only be sent if the previous one has completed (or failed)

at this level of abstraction, actions are in the form of frames, where each frame is simply a new
set of coordinates, body pitchation, and 'yaw' direction
(x, y, z, deg, deg)

"in" messages can be the following:
    new frame
    check status
    check location
    stop movement

"out" messages can be the following:
    status (success failure idle) or (0 1 2)

"""

from spock.utils import pl_announce
from spock.mcp import mcdata
from spock.utils import Vec3


import math, Queue

import logging
logger = logging.getLogger('spock')


# adds body pitchation and yaw direction to the Spock Vec3 class
# this is necessary to save the state of a player or mob entity
# I will move it to a utils file when I find it is necessary elsewhere
class Vec5(Info):
    
    def __init__(self, x=0.0, y=0.0, z=0.0, pitch=0.0, yaw=0.0, vec=None):
        
        if vec:
            self.x, self.y, self.z, self.pitch, self.yaw= vec[:5]
        else:
            self.x = x
            self.y = y
            self.z = z
            # assume ALL angles are in degrees. if someone decides they like
            # radians better, this will have to be extended
            self.pitch = self.projectAngleToCircle(pitch)
            self.yaw = self.projectAngleToCircle(yaw)
    
   
    def add_vector(self, x=None, y=None, z=None, pitch=None, yaw=None, vec=None):
        
        if vec:
            self.x += vec.x
            self.y += vec.y
            self.z += vec.z
            self.pitch += pitch
            self.yaw += yaw

            self.pitch = self.projectAngleToCircle(self.pitch)
            self.yaw = self.projectAngleToCircle(self.yaw)
        
        else:
            if x:
                self.x += x
            if y:
                self.y += y
            if z:
                self.z += z
            if pitch:
                self.pitch += pitch
                self.pitch = self.projectAngleToCircle(self.pitch)
            if yaw:
                self.yaw += yaw
                self.yaw = self.projectAngleToCircle(self.yaw)


    # simply takes an angle and puts it in the range 0 to 360
    # aahhh Minecraft angles are weird, will have to adapt to their range (-90 to 90)
    def projectAngleToCircle(angle):

        if angle > 360:
            newangle = angle - 360*math.floor(angle/360.0)
        else if angle < 0:
            newangle = angle + 360*(1 + math.floor(abs(angle/360.0)))
        else:
            newangle = angle

        return newangle
    
    
    def __str__(self):
        
        return "({:.2f}, {:.2f}, {:.2f}, {:.2f}, {:.2f})".format(self.x, self.y, self.z, self.pitch, self.yaw)



class MovementCore:

    def __init__(self):

        self.position = Vec5()
        self.actions = Queue.Queue()
        self.is_moving = False


    def addFrame(frame):

        try:
            actions.put(frame)
        
        except actions.Full:
            print "actions queue is full"

    def stop():
  
        actions.clear()


    def getPosition():

        return self.position



@pl_announce('Movement')
class MovementPlugin:
    
    def __init__(self, ploader, settings):

        # Spock plugins required for basic functionality
        self.net = ploader.requires('Net')
        self.clinfo = ploader.requires('ClientInfo')
        self.physics = ploader.requires('Physics')
        
        # register all event handlers for this class
        ploader.reg_event_handler('client_tick', self.client_tick)
        ploader.reg_event_handler('action_tick', self.action_tick)
        ploader.reg_event_handler('cl_position_update', self.handle_position_update)
        #ploader.reg_event_handler('phy_collision', self.handle_collision)
        
        # make plugin available externally
        self.mvc = MovementCore()
        ploader.provides('Movement', self.mvc)
    
    
    def client_tick(self, name, data):
    
        self.net.push_packet('PLAY>Player Position', self.clinfo.position.get_dict())
    
    
    def handle_position_update(self, name, data):
    
        self.net.push_packet('PLAY>Player Position and Look', data.get_dict())
    
    def moveTo(x, y, z, pitch, yaw):

        self.clinfo.position.x = x
        self.clinfo.position.y = x
        self.clinfo.position.z = z
        self.clinfo.position.pitch = pitch
        self.clinfo.position.yaw = yaw

    def action_tick(self, name, data):
        
        try:
            next_action = self.actions.get()
            # check if new location is different from old
            # some rounding is necessary, might want to change it
            # to be within some predefined error instead of truncating
            if (next_action.x != math.floor(self.clinfo.position.x)
                    or next_action.y != math.floor(self.clinfo.position.y))
                    or next_action.z != math.floor(self.clinfo.position.z)):
                    or next_action.pitch != math.floor(self.clinfo.position.pitch)):
                    or next_action.yaw != math.floor(self.clinfo.position.yaw)):
                self.moveTo(next_action.x,
                            next_action.y,
                            next_action.z,
                            next_action.pitch,
                            next_action.yaw)

        except actions.Empty:
            print "actions queue is empty!"

