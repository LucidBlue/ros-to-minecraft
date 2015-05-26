
# ros-to-minecraft
==================

simple demo linking ROS to Spock (an open-source Python implementation of the Minecraft protocol)



Dependencies:
-------------

> ROS | http://www.ros.org/

> minecraft server | https://minecraft.net/download

> SpockBot | https://github.com/SpockBotMC/SpockBot

> also the SpockBot examples may be helpful: | https://github.com/SpockBotMC/SpockBot-Contrib


Instructions
------------

1. Install ROS

2. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory

3. Install the Minecraft server, but remember to: a) add yourself as an admin and b) set the server to "offline mode" in the config file. This last bit is necessary to allow the bot to log in without authentication.

4. Modify the files: event.py and inventory.py (in the SpockBot source) by just copy-pasting from this repo. event.py contains new code to create a ROS node, subscriber, and callback function that triggers an event. I've had trouble getting ROS to work with Python 3, so I had to 'retrofit' inventory.py a little to make it work with 2.7

5. Build the SpockBot source

6. Run the two ROS nodes: cog_sender and spock_listener. cog_sender randomly sends a digit from {0, 1, 2} to spock_listener

7. If everything worked, the bot should be... waving its arm occasionally (on receiving 1)


