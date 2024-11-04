"""
Simple module for returning relative paths.
"""
from pathlib import Path


def supercharger_build() -> Path:
    """
    Returns the full path of the supercharger build directory.
    """
    return supercharger_root() / 'build'


def supercharger_root() -> Path:
    """
    Returns the full path of the supercharger root directory.
    """
    return Path(__file__).resolve().parent.parent.parent.parent.parent