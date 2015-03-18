"""
Offline connection demo
"""

from spock import Client
from spock.plugins import DefaultPlugins
from demoplugin import DemoPlugin

settings = {
    'start': {
        'username': 'Bot',
    },
    'auth': {
        'authenticated': False,
    },
}

plugins = DefaultPlugins
plugins.append(('demo', DemoPlugin))
client = Client(plugins = plugins, settings = settings)
#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)
