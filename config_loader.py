import json

config = None

def load_config():
    global config
    if config is None:
        with open('config.json', 'r') as config_file:
            config = json.load(config_file)
    return config
