from drone.core import Drone
from drone.visualization import DroneVisualizer
from drone.waypoints import generate_waypoints
from drone.logging_utils import setup_logger


def main():
    all_waypoints, main_waypoints = generate_waypoints()
    logger = setup_logger()
    drone = Drone()
    visualizer = DroneVisualizer(drone, all_waypoints, main_waypoints, logger)
    visualizer.run()


if __name__ == "__main__":
    main()
