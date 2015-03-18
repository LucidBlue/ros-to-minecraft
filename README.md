==================
# ros-to-minecraft
==================
simple demo linking ROS to Spock (an open-source python implementation of the Minecraft protocol)


-------------
Dependencies:
-------------

> ROS | http://www.ros.org/

> minecraft server | https://minecraft.net/download

> SpockBotMC | https://github.com/SpockBotMC/SpockBot

> also the SpockBot examples may be helpful: | https://github.com/SpockBotMC/SpockBot-Contrib

------------
Instructions
------------
The process is a little involved, so bear with me!

1. Install ROS
2. Create a catkin package in your workspace (minecraft_bot) and include all the files from this directory
3. Install the Minecraft server, but remember to: a) add yourself as an admin and b) set the server to "offline mode" in the config file. This last bit is necessary to allow the bot to log in without authentication.
4. Modify the files: event.py and inventory.py (in the SpockBot source) by just copy-pasting from this repo.
5. Build the SpockBot source
6. Run the two ROS nodes: spock_listener and spock_msg_generator
7. Hope that everything works!


