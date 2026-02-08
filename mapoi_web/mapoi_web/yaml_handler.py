"""YAML handler for mapoi config files.

Reads/writes mapoi_config.yaml, replacing only the 'poi' section
while preserving 'map' and 'route' sections and unknown fields.
"""

import yaml


def load_config(config_path):
    """Load a mapoi_config.yaml file and return parsed dict."""
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def save_pois(config_path, pois):
    """Replace only the 'poi' section in config_path, preserving other sections.

    Args:
        config_path: Path to mapoi_config.yaml
        pois: List of POI dicts to write
    """
    config = load_config(config_path)
    config['poi'] = pois
    with open(config_path, 'w') as f:
        yaml.dump(config, f, default_flow_style=None, allow_unicode=True, sort_keys=False)


def get_pois(config_path):
    """Read POI list from config file."""
    config = load_config(config_path)
    return config.get('poi', [])


def get_routes(config_path):
    """Read route list from config file."""
    config = load_config(config_path)
    return config.get('route', [])
