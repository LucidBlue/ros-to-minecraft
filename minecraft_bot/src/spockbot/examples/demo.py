"""
Basic demo example
"""

from spock import Client
from spock.plugins import DefaultPlugins
from demoplugin import DemoPlugin

start_settings = {
    'username': 'your_username',
    'password': 'your_password',
}

plugins = DefaultPlugins
plugins.append(('demo', DemoPlugin))
client = Client(plugins = plugins, start = start_settings)
#client.start() with no arguments will automatically connect to localhost
client.start('localhost', 25565)
