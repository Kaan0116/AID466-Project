"""Shared loader for config/study_params.yaml (single source of truth)."""

import os

import yaml


def default_config_path() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('following_study'),
                        'config', 'study_params.yaml')


def load(path: str = '') -> dict:
    """Return the whole YAML as a dict."""
    with open(path or default_config_path()) as f:
        return yaml.safe_load(f)


def follower_params(cfg: dict) -> dict:
    return cfg['follower_node']['ros__parameters']


def driver_params(cfg: dict) -> dict:
    return cfg['person_driver']['ros__parameters']


def study(cfg: dict) -> dict:
    return cfg['study']['ros__parameters']
