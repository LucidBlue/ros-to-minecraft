#!/usr/local/bin/python

"""
created by Bradley Sheneman

heavily modified from "move.py", a helper plugin created by the SpockBot devs

Spock plugin to control bot movement. Receives a movement command from within Spock
and puts it in a queue. a separate callback grabs from the queue and sends to Minecraft server.

at this level of abstraction, actions are in the form of frames where each frame is simply a new
set of coordinates, body pitch, and yaw:

(x, y, z, pitch, yaw)

The "legality" of a move will be checked before sending to this plugin. The SendActionPlugin only
deals with receiving frames from Spock and sending them to the server

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
from spockextras.plugins.actionutils import Vec5

import math, Queue

import logging
logger = logging.getLogger('spock')



class SendActionCore:

    def __init__(self):

        #self.position = Vec5()
        self.actions = Queue.Queue()
        self.is_moving = False


    def addFrame(frame):

        try:
            actions.put(frame)
        
        except actions.Full:
            print "actions queue is full"

    def stop():
  
        actions.clear()

    """
    def getPosition():

        return self.position
    """


@pl_announce('SendAction')
class SendActionPlugin:
    
    def __init__(self, ploader, settings):

        # Spock plugins required for basic functionality
        self.net = ploader.requires('Net')
        self.clinfo = ploader.requires('ClientInfo')
        # no physics necessary here...
        #self.physics = ploader.requires('Physics')
        
        # register all event handlers for this class
        ploader.reg_event_handler('client_tick', self.client_tick)
        ploader.reg_event_handler('action_tick', self.action_tick)
        ploader.reg_event_handler('cl_position_update', self.handle_position_update)
        #ploader.reg_event_handler('phy_collision', self.handle_collision)
        
        # make plugin available externally
        self.mvc = SendActionCore()
        ploader.provides('SendAction', self.mvc)
    
    
    def client_tick(self, name, data):
    
        self.net.push_packet('PLAY>Player Position', self.clinfo.position.get_dict())
    
    
    def handle_position_update(self, name, data):
    
        self.net.push_packet('PLAY>Player Position and Look', data.get_dict())
    
    def updatePos(self, x, y, z, pitch, yaw):
        
        # kind of a hacky way to make the full, 5-element vector available externally
        # I do not want to expose client info to the world, so I make a "Vec5" to hold the pos info
        #self.mvc.position.x       += x
        #self.mvc.position.y       += y
        #self.mvc.position.z       += z
        #self.mvc.position.pitch   += pitch
        #self.mvc.position.yaw     += yaw

        #self.clinfo.position.x += x
        #self.clinfo.position.y += y
        #self.clinfo.position.z += z
        #self.clinfo.position.pitch += pitch
        #self.clinfo.position.yaw += yaw
        self.physics.move()
        #print ("new pos:")
        #print (self.clinfo.position)

    # as of now, action ticks (also physics and client ticks) are every 0.05 seconds
    def action_tick(self, name, data):
        
        try:
            next_action = self.mvc.actions.get()
            #print(next_action)
            # check if new location is different from old
            # some rounding is necessary, might want to change it
            # to be within some predefined error instead of truncating
            if (next_action.x != math.floor(self.clinfo.position.x)
                    or next_action.y != math.floor(self.clinfo.position.y)
                    or next_action.z != math.floor(self.clinfo.position.z)
                    or next_action.pitch != math.floor(self.clinfo.position.pitch)
                    or next_action.yaw != math.floor(self.clinfo.position.yaw)):
                
                self.updatePos(
                        next_action.x,
                        next_action.y,
                        next_action.z,
                        next_action.pitch,
                        next_action.yaw)

        except Queue.Empty:
            print "actions queue is empty!"

