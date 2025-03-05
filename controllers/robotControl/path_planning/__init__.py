"""
Path Planning Module

This module handles the generation of cleaning paths and routes
for the pressure washing robot.

Components:
- cleaning_path_generator: Generates cleaning paths for defined areas
- path_utils: Utility functions for path manipulation and conversion
"""

from .cleaning_path_generator import CleaningPathGenerator, generate_cleaning_path

__all__ = [
    'CleaningPathGenerator',
    'generate_cleaning_path'
] 