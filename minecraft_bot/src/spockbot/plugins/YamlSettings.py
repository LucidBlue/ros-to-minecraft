"""
A more advanced settings mixin that can parse yaml files and obeys setting
heirarchies. Janky enough that I don't feel comfortable putting it in SpockBot
proper, if a project wants that they can build their own settings parser around
the generic SpockBot settings manager.
"""

__author__ = "Nick Gamberini"
__copyright__ = "Copyright 2015, The SpockBot Project"
__license__ = "MIT"

#requires PyYAML
import yaml
from spock.utils import get_settings, pl_announce
from spock.plugins.core.settings import PloaderFetch

@pl_announce('PloaderFetch')
class YamlSettings:
	def __init__(self, ploader, kwargs):
		del kwargs['settings_mixin']
		self.plugin_list = []
		temp_set = {}
		if 'settings' in kwargs:
			settings = get_settings(kwargs, kwargs['settings'])
			del kwargs['settings']
		else:
			settings = kwargs
		if 'file_path' in settings:
			temp_set = self._import_yaml(kwargs['file_path'], 'plugins')
			del settings['file_path']
		for key, val in temp_set.items():
			settings[key] = get_settings(settings.get(key, {}), val)
		if 'plugins' in settings:
			self.plugin_list = settings['plugins']
			del settings['plugins']

		plugins = []
		plugin_settings = {}
		for plugin in self.plugin_list:
			plugins.append(plugin[1])
			plugin_settings[plugin[1]] = temp_set[plugin[0]]
		ploader.provides('PloaderFetch', PloaderFetch(plugins, plugin_settings))

	def _import_yaml(self, path, obj):
		temp_set = {}
		data = yaml.load(open(path).read())
		for key, plugin in data[obj].items():
			if plugin[0] == 'yaml':
				for key, val in self._import_yaml(plugin[1], key).items():
					temp_set[key] = get_settings(temp_set.get(key, {}), val)
			elif plugin[0] == 'python':
				for key, _ in self._import_python(plugin[1], key):
					temp_set[key] = data.get(key, {})
			else:
				self._import_plugin(plugin[1], plugin[0], key)
				temp_set[key] = data.get(key, {})
		return temp_set

	def _import_python(self, path, obj):
		pylist = __import__(path, fromlist=[obj]).__dict__[obj]
		for plugin in pylist:
			self.plugin_list.append(plugin)
		return pylist

	def _import_plugin(self, path, obj, name):
		plugin = __import__(path, fromlist=[obj]).__dict__[obj]
		self.plugin_list.append((name, plugin))

if __name__ == '__main__':
	from spock import Client
	Client(settings_mixin = YamlSettings, file_path = 'config.yaml').start()
