import sys
import rclpy
from rclpy.node import Node
from irobot_create_msgs.srv import ResetPose


class ResetPoseClient(Node):

    def __init__(self):
        super().__init__('reset_pose_client')
        self.cli = self.create_client(ResetPose, 'reset_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = ResetPose.Request()

    def send_request(self, pose_x, pose_y, goal_rotation_angle):
        dist = self.req.pose
        dist.position.x = pose_x
        dist.position.y = pose_y
        dist.orientation.w = goal_rotation_angle

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    reset_pose_client = ResetPoseClient()
    response = reset_pose_client.send_request(float(0),float(0),float(1))

    reset_pose_client.destroy_node()
    rclpy.shutdown()
    print("node shut down")


if __name__ == '__main__':
    main()
