#! /usr/bin/env python

"""
    Created by Bradley Sheneman
    ExecutionOutputLink provides list of arguments for called function
    ExecutionLink connects inputs to outputs
    GroundedSchemaNode function: py: module.function
    but if not being loaded from separate file, just use function name
"""
import roslib; roslib.load_manifest('minecraft_bot')
import rospy
from minecraft_bot.msg import controller_msg#, mc_msg
from time import sleep

import opencog.cogserver
from opencog.atomspace import types, AtomSpace, Atom, TruthValue
from opencog.scheme_wrapper import scheme_eval

import sys
import random
from random import randint

actionvals = {}

class SimpleAgent(opencog.cogserver.MindAgent):

    def __init__(self):
        self.a = AtomSpace()
        self.nodes = {}

        #add 3 nodes with integer values
        self.nodes[0] = self.a.add(types.NumberNode, "0")
        self.nodes[1] = self.a.add(types.NumberNode, "1")
        self.nodes[2] = self.a.add(types.NumberNode, "2")

    def performAction(self):
        #randomly select a link from those available and add the nodes
        fnode = self.a.add_node(types.GroundedSchemaNode, "py:sendValue")
        current_link = self.a.add_link(types.ExecutionOutputLink, [
            fnode,
            self.a.add_link(types.ListLink, [self.nodes[randint(0,2)]])])
        print(self.nodes[randint(0,2)].name)
        #self.a.add_node(types.NumberNode, '5')
        #exec_link = self.a.add_link(types.ExecutionLink, current_link.out)
        #print(exec_link.out)
        #return scheme_eval(self.a, "(cog-execute! exec_link)")

def mainloop():
    global spock_pub

    # Initialize ROS node
    rospy.init_node('controller')

    # Create publisher to send out ControllerMsg messages
    spock_pub = rospy.Publisher('controller_data', controller_msg)
    
    Agent = SimpleAgent()
    
    while not rospy.is_shutdown():
        Agent.performAction()
        rospy.sleep(5.)

def sendValue(atom):
    message = controller_msg()
    message.action = int(atom.name)
    spock_pub.publish(message)
    print("published action: %d" %(message.action))



if __name__ == "__main__":
    mainloop()
