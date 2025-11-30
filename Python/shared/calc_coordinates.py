"""
Created by: Balassa Hodi
Short description comes here
"""

from .classes.coordinates import Coordinates
import math


def calc_coordinates(x1: float, y1: float, theta: float, l: float):
    """
    Some description comes here.
    """

    x = x1 + l * math.cos(math.radians(theta))
    y = y1 + l * math.sin(math.radians(theta))

    return Coordinates(x, y)
