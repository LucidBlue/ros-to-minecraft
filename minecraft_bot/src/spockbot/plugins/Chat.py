"""
Parses chat for you and fires a handy chat_message event
"""
__author__ = "Morgan Creekmore"
__copyright__ = "Copyright 2015, The SpockBot Project"
__license__ = "MIT"

from spock.utils import pl_event,string_types

import logging
logger = logging.getLogger('spock')

@pl_event('Chat')
class ChatPlugin:
	def __init__(self, ploader, settings):
		self.event = ploader.requires('Event')
		ploader.reg_event_handler(
			'PLAY<Chat Message', self.handle_chat_message
		)

	def handle_chat_message(self, name, packet):
		chat_data = packet.data['json_data']
		message = self.parse_chat(chat_data)
		if message != "":
			logger.info('Chat: %s', message)
		self.event.emit('chat_message', {'message': message, 'data':chat_data})

	def parse_chat(self, chat_data):
		message = ''
		if type(chat_data) is dict:
			if 'text' in chat_data:
				message += chat_data['text']
				if 'extra' in chat_data:
					message += self.parse_chat(chat_data['extra'])
			elif 'translate' in chat_data:
				if 'with' in chat_data:
					message += self.parse_chat(chat_data['with'])
		elif type(chat_data) is list:
			for text in chat_data:
				if type(text) is dict:
					message += self.parse_chat(text)
				elif type(text) is string_types:
					message += ' ' + text		
		return message
