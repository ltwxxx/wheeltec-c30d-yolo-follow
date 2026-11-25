"""Console entry point for the read-only sensor bridge."""
from .follow_sensors import Sensors, main

__all__ = ["Sensors", "main"]

if __name__ == "__main__":
    main()
