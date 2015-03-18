#! /usr/bin/env python

#import roslib; roslib.load_manifest('minecraft_bot')
#import rospy
#from minecraft_bot.msg import controller_msg

import sys

from spock import Client, PluginLoader
from spock.plugins import DefaultPlugins
from spock.plugins.helpers.move import MovementPlugin
from spock.plugins.helpers.physics import PhysicsPlugin
from spock.plugins.helpers.entities import EntityPlugin
from spock.plugins.helpers.clientinfo import ClientInfoPlugin
from spockbot.plugins.CursesCommand import *
from spockbot.plugins.BaseCommands import BaseCommandsPlugin
from spockbot.plugins.ChatCommand import ChatCommandPlugin
from spockbot.plugins.Chat import ChatPlugin

# connect to localhost server
settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = DefaultPlugins
plugins.append(('basecommand', BaseCommandsPlugin))
plugins.append(('chat', ChatPlugin))
plugins.append(('chatcommand', ChatCommandPlugin))
plugins.append(('Movement', MovementPlugin))
plugins.append(('Physics', PhysicsPlugin))
plugins.append(('Entities', EntityPlugin))
plugins.append(('ClientInfo', ClientInfoPlugin))
client = Client(plugins = plugins, settings = settings)

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)
