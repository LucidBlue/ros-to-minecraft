#Offline connection demo

from sys import path


from spock import Client
from spock.plugins import DefaultPlugins
from examples.demoplugin import DemoPlugin

settings = {'start': {'username': 'Bot',},'auth': {'authenticated': False,},}
plugins = DefaultPlugins
plugins.append(('demo', DemoPlugin))
client = Client(plugins = plugins, settings = settings)

#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)


