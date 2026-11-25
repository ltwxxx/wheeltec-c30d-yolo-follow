"""Console entry point for the ROS 2 actuator bridge."""
from .follow_actuator import Actuator, main

__all__ = ["Actuator", "main"]


if __name__ == "__main__":
    main()
