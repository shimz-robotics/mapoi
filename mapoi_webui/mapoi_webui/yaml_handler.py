"""YAML handler for mapoi config files.

Reads/writes mapoi_config.yaml, replacing only the 'poi' section
while preserving 'map' and 'route' sections and unknown fields.
"""

import os
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


def save_routes(config_path, routes):
    """Replace only the 'route' section in config_path, preserving other sections.

    Args:
        config_path: Path to mapoi_config.yaml
        routes: List of route dicts to write
    """
    config = load_config(config_path)
    config['route'] = routes
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


def get_tag_definitions(system_tags_path, config_path):
    """Read tag definitions from system tag_definitions.yaml and per-map custom_tags.

    Args:
        system_tags_path: Path to mapoi_server's maps/tag_definitions.yaml
        config_path: Path to current map's mapoi_config.yaml

    Returns:
        List of dicts: [{'name': str, 'description': str, 'is_system': bool}, ...]
    """
    tags = []

    # System tags
    if os.path.exists(system_tags_path):
        with open(system_tags_path, 'r') as f:
            data = yaml.safe_load(f)
        for tag in (data or {}).get('tags', []):
            tags.append({
                'name': tag.get('name', ''),
                'description': tag.get('description', ''),
                'is_system': True,
            })

    # User tags from per-map config
    if os.path.exists(config_path):
        config = load_config(config_path)
        for tag in config.get('custom_tags', []):
            tags.append({
                'name': tag.get('name', ''),
                'description': tag.get('description', ''),
                'is_system': False,
            })

    return tags
