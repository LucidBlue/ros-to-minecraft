"""
Follow a simple plugin for giving commands for following a player or walking to a location
"""
__author__ = "Morgan Creekmore"
__copyright__ = "Copyright 2015, The SpockBot Project"
__license__ = "MIT"

import logging
logger = logging.getLogger('spock')

class FollowPlugin:
	def __init__(self, ploader, settings):
		self.net = ploader.requires('Net')
		self.clinfo = ploader.requires('ClientInfo')
		self.entities = ploader.requires('Entities')
		self.movement = ploader.requires('Movement')

		self.follow_ent = None
		ploader.reg_event_handler('cmd_tp', self.handle_tp)
		ploader.reg_event_handler('client_tick', self.client_tick)
		ploader.reg_event_handler('cmd_follow', self.handle_follow)
		ploader.reg_event_handler('PLAY<Spawn Player', self.handle_spawn_player)

	def handle_spawn_player(self, name, packet):
		name = "Unknown"
		if packet.data['uuid'] in self.clinfo.player_list:
			name = self.clinfo.player_list[packet.data['uuid']].name
		logger.info("Spawn Player: %s %s", packet.data['eid'], name)

	def handle_tp(self, event, data):
		args = data['args']
		self.movement.move_to(int(args[0]), 0, int(args[1]))

	def handle_follow(self, event, data):
		args = data['args']
		if args[0] == 'stop':
			self.follow_ent = None
		else:
			for uuid, player in self.clinfo.player_list.items():
				if args[0].lower() in player.name.lower():
					for ent_id, ent in self.entities.players.items():
						if ent.uuid == uuid:
							self.follow_ent = ent_id
							logger.debug("Following: %s", player.name)

	def client_tick(self, name, data):
		if self.follow_ent != None:
			if self.follow_ent in self.entities.players:
				ent = self.entities.players[self.follow_ent]
				self.movement.move_to(int(ent.x), int(ent.y), int(ent.z))
			else:
				self.follow_ent = None
			