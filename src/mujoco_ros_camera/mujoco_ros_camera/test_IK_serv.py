import rclpy
from rclpy.node import Node
from mujoco_ros_camera.srv import IKService
import numpy as np
from rrt_algorithms.rrt.rrt_star import RRTStar
from rrt_algorithms.search_space.search_space import SearchSpace
from rrt_algorithms.utilities.plotting import Plot


class RRTStarService(Node):
    def __init__(self):
        super().__init__('rrt_star_service')
        self.srv = self.create_service(IKService, 'rrt_star_service', self.rrt_star_callback)

    def rrt_star_callback(self, request, response):
        X_dimensions = np.array(request.x_dimensions)
        Obstacles = np.array(request.obstacles)
        x_init = tuple(request.x_init)
        x_goal = tuple(request.x_goal)
        q = request.q
        r = request.r
        max_samples = request.max_samples
        rewire_count = request.rewire_count
        prc = request.prc

        # create Search Space
        X = SearchSpace(X_dimensions, Obstacles)

        # create rrt_search
        rrt = RRTStar(X, q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path = rrt.rrt_star()

        if path is not None:
            response.path = [list(point) for point in path]
        else:
            response.path = []

        return response


def main(args=None):
    rclpy.init(args=args)
    rrt_star_service = RRTStarService()
    rclpy.spin(rrt_star_service)
    rclpy.shutdown()

