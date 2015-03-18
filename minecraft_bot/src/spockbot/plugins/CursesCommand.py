"""
Curses console for sending commands local and remote in the format `l command args` or `r command args`
"""
__author__ = "Morgan Creekmore"
__copyright__ = "Copyright 2015, The SpockBot Project"
__license__ = "MIT"

import curses,os,sys,traceback
from spock.mcp.mcdata import (
	GM_SURVIVAL, GM_CREATIVE, GM_ADVENTURE, GM_SPECTATOR
)

import logging
logger = logging.getLogger('spock')

PROMPT = '> '

class Screen:
	def __init__(self, stdscr, processor):
		self.timer = 0
		self.statusText = "SpockBot"
		self.searchText = PROMPT
		self.commands = []
		self.commandindex = 0
		self.cursorpos = len(self.searchText)
		self.stdscr = stdscr
		self.cmdprocessor = processor
		self.ignorekeys = [curses.KEY_MOUSE]

		# set screen attributes
		self.stdscr.nodelay(1) # this is used to make input calls non-blocking
		curses.cbreak()
		self.stdscr.keypad(1)
		curses.curs_set(1)     # no annoying mouse cursor

		self.rows, self.cols = self.stdscr.getmaxyx()
		self.lines = []

		curses.start_color()

		# create color pair's 1 and 2
		curses.init_pair(1, curses.COLOR_BLACK, curses.COLOR_WHITE)
		curses.init_pair(2, curses.COLOR_CYAN, curses.COLOR_BLACK)

		self.paintStatus(self.statusText)

	def connectionLost(self, reason):
		self.close()

	def addLine(self, text):
		""" add a line to the internal list of lines"""

		self.lines.append(text)
		self.redisplayLines()

	def redisplayLines(self):
		""" method for redisplaying lines 
			based on internal list of lines """

		self.stdscr.clear()
		self.paintStatus(self.statusText)
		try:
			i = 0
			index = len(self.lines) - 1
			while i < (self.rows - 3) and index >= 0:
				self.stdscr.addstr(self.rows - 3 - i, 0, self.lines[index], 
								   curses.color_pair(2))
				i = i + 1
				index = index - 1
		except:
			pass
		self.stdscr.refresh()

	def paintStatus(self, text):
		if len(text) > self.cols: text = text[self.cols:]
		self.stdscr.addstr(self.rows-2,0,text + ' ' * (self.cols-len(text)), 
						   curses.color_pair(1))
		# move cursor to input line
		self.stdscr.move(self.rows-1, self.cursorpos)

	def setSearchText(self, text):
		self.searchText = text
		self.cursorpos = len(self.searchText)

	def doRead(self):
		""" Input is ready! """
		curses.noecho()
		self.timer = self.timer + 1
		c = self.stdscr.getch() # read a character
		if c in self.ignorekeys:
			pass

		elif c == curses.KEY_BACKSPACE or c == 127:
			if len(self.searchText) > len(PROMPT):
				if self.cursorpos == len(self.searchText):
					self.setSearchText(self.searchText[:-1])
				elif self.cursorpos < len(self.searchText):
					self.searchText = self.searchText[:self.cursorpos-1] + self.searchText[self.cursorpos:]
					self.cursorpos-=1

		elif c == curses.KEY_ENTER or c == 10:
			self.cmdprocessor.process_command(self.searchText[2:])
			self.commands.append(self.searchText[2:])
			self.commandindex = len(self.commands)
			self.stdscr.refresh()
			self.setSearchText(PROMPT)
		elif c == curses.KEY_UP:
			if self.commandindex-1 >= 0:
				self.commandindex -= 1
				self.setSearchText(PROMPT + self.commands[self.commandindex])
		elif c == curses.KEY_DOWN:
			if self.commandindex+1 < len(self.commands):
				self.commandindex += 1
				self.setSearchText(PROMPT + self.commands[self.commandindex])
		elif c == curses.KEY_LEFT:
			self.cursorpos -= 1
			if self.cursorpos < len(PROMPT):
				self.cursorpos = len(PROMPT)
		elif c == curses.KEY_RIGHT:
			self.cursorpos += 1
			if self.cursorpos > len(self.searchText):
				self.cursorpos = len(self.searchText)
		else:
			if len(self.searchText) == self.cols-2: return
			try:
				if self.cursorpos == len(self.searchText):
					self.setSearchText(self.searchText + chr(c))
				elif self.cursorpos < len(self.searchText):
					self.searchText = self.searchText[:self.cursorpos] + chr(c) + self.searchText[self.cursorpos:]
					self.cursorpos+=1
			except:
				pass

		self.stdscr.addstr(self.rows-1, 0, 
						   self.searchText + (' ' * (
						   self.cols-len(self.searchText)-2)))
		self.stdscr.move(self.rows-1, self.cursorpos)
		self.paintStatus(self.statusText)
		self.stdscr.refresh()

	def close(self):
		""" clean up """
		curses.nocbreak()
		self.stdscr.keypad(0)
		curses.echo()
		curses.endwin()

class CommandProcessor:
	def __init__(self, event, net):
		self.event = event
		self.net = net
	
	def process_command(self, line):
		msg = line.split(' ')
		if len(msg) < 2:
			logger.info("Command: Not enough arguments")
			return
		loc = msg[0]
		command = msg[1]
		args = []
		if len(msg) > 2:
			args = msg[2:]
		if loc == 'l':
			logger.info("Command: %s Args: %s", command, args)
			self.event.emit('cmd_'+command, {'args': args})
		elif loc == 'r':
			self.net.push_packet('PLAY>Chat Message', {'message': command + ' ' + ' '.join(args)})


class CursesHandler(logging.Handler):
		def __init__(self, screen):
			logging.Handler.__init__(self)
			self.screen = screen
		def emit(self, record):
			msg = self.format(record)
			self.screen.addLine(msg)

class CursesCommandPlugin:
	def __init__(self, ploader, settings):
		self.event = ploader.requires('Event')
		self.net = ploader.requires('Net')
		self.clinfo = ploader.requires('ClientInfo')
		stdscr = curses.initscr() # initialize curses
		cmd = CommandProcessor(self.event, self.net)
		self.screen = Screen(stdscr,cmd)   # create Screen object
		stdscr.refresh()
		cursesHandler = CursesHandler(self.screen)
		formatter = logging.Formatter('[%(levelname)s]: %(message)s')
		cursesHandler.setFormatter(formatter)
		logger.addHandler(cursesHandler)


		ploader.reg_event_handler('event_tick', self.tick)
		ploader.reg_event_handler('kill', self.kill)
		self.set_uncaught_exc_handler()

	def tick(self, event, data):
		c = self.clinfo
		gamemode = ""
		gm = c.game_info.gamemode
		if gm == GM_CREATIVE:
			gamemode = "Creative"
		elif gm == GM_SURVIVAL:
			gamemode = "Survival"
		elif gm == GM_ADVENTURE:
			gamemode = "Adventure"
		elif gm == GM_SPECTATOR:
			gamemode = "Spectator"
		pos = "(%s, %s, %s)" % (c.position.x, c.position.y, c.position.z)
		self.screen.statusText = "%s Mode:%s Pos:%s Health:%s Food:%s" % (c.name, gamemode, pos, c.health.health, c.health.food)
		self.screen.doRead()

	def kill(self, event, data):
		self.screen.close()

	# try exiting curses and restore console before printing stack and crashing
	def set_uncaught_exc_handler(self):
		""" Call this function to setup the `sys.excepthook` to exit curses and
		restore the terminal before printing the exception stack trace. This way
		your application does not mess up the users terminal if it crashes. (And
		you can use assertions for debugging, etc...)"""
		def handle(exec_type, exec_value, exec_traceback):
			try: self.screen.close()
			except Exception: pass
			sys.__excepthook__(exec_type, exec_value, exec_traceback)
		sys.excepthook = handle
