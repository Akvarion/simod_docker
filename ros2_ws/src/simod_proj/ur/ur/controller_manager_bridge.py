#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers

class ControllerManagerBridge(Node):
    def __init__(self):
        super().__init__('controller_manager_bridge')

        # Create service server for /controller_manager/list_controllers
        self.srv = self.create_service(
            ListControllers,
            '/controller_manager/list_controllers',
            self.handle_list_controllers
        )

        # Create clients for the namespaced controller managers
        self.left_client = self.create_client(ListControllers, '/left/controller_manager/list_controllers')
        self.right_client = self.create_client(ListControllers, '/right/controller_manager/list_controllers')

        self.get_logger().info('ControllerManagerBridge node started.')

    async def handle_list_controllers(self, request, response):
        # Wait for both services to be available
        while not self.left_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /left/controller_manager/list_controllers...')
        while not self.right_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Waiting for /right/controller_manager/list_controllers...')

        # Call both services asynchronously
        left_future = self.left_client.call_async(ListControllers.Request())
        right_future = self.right_client.call_async(ListControllers.Request())

        await left_future
        await right_future

        # Merge the results
        controllers = []
        if left_future.result():
            controllers.extend(left_future.result().controller)
        if right_future.result():
            controllers.extend(right_future.result().controller)

        response.controller = controllers
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ControllerManagerBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()