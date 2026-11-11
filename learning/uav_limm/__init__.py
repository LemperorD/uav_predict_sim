"""Standalone, ROS-independent implementation of Tang et al. (2025) LIMM."""

from .config import ModelConfig
from .model import DifferentialForecaster

__all__ = ["ModelConfig", "DifferentialForecaster"]
