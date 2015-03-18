"""
Handful of useful commands
CursesCommand or ChatCommand need to be loaded for this plugin to do anything
"""
__author__ = "Morgan Creekmore"
__copyright__ = "Copyright 2015, The SpockBot Project"
__license__ = "MIT"

import datetime

class BaseCommandsPlugin:
	def __init__(self, ploader, settings):
		self.net = ploader.requires('Net')
		self.physics = ploader.requires('Physics')

		ploader.reg_event_handler('cmd_jump', self.handle_jump)
		ploader.reg_event_handler('cmd_speak', self.handle_speak)
		ploader.reg_event_handler('cmd_date', self.handle_date)
		ploader.reg_event_handler('cmd_command', self.handle_command)
		ploader.reg_event_handler('cmd_slot', self.handle_slot)
		ploader.reg_event_handler('cmd_place', self.handle_place)
		ploader.reg_event_handler('cmd_break', self.handle_break)
		ploader.reg_event_handler('cmd_animation', self.handle_animation)

	def handle_jump(self, event, data):
		self.physics.jump()

	def handle_speak(self, event, data):
		self.net.push_packet('PLAY>Chat Message', {'message': ' '.join(data['args'])})

	def handle_date(self, event, data):
		self.net.push_packet('PLAY>Chat Message', {'message': 'Current Date: ' + str(datetime.datetime.now())})

	def handle_command(self, event, data):
		self.net.push_packet('PLAY>Chat Message', {'message': '/' + ' '.join(data['args'])})

	def handle_slot(self, event, data):
		args = data['args']
		if len(args) == 1 and (int(args[0]) >= 0 and int(args[0]) <= 8):
				self.net.push_packet('PLAY>Held Item Change', {'slot': int(args[0])})

	def handle_place(self, event, data):
		args = data['args']
		block_data = {'location': {'x': int(args[0]),'y': int(args[1]),'z': int(args[2])}, 'direction':1, 'held_item': {'id': -1}, 'cur_pos_x': 8, 'cur_pos_y': 16, 'cur_pos_z': 8}
		self.net.push_packet('PLAY>Player Block Placement', block_data)	

	def handle_break(self, event, data):
		args = data['args']
		block_data = {'location': {'x': int(args[0]),'y': int(args[1]),'z': int(args[2])}, 'status':0, 'face': 1}
		self.net.push_packet('PLAY>Player Digging', block_data)
		block_data['status'] = 2
		self.net.push_packet('PLAY>Player Digging', block_data)

	def handle_animation(self, event, data):
		self.net.push_packet('PLAY>Animation', '')
