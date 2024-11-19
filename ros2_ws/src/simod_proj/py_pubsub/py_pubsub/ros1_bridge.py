import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import argparse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import time 
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class Ros1Bridge(Node):
    def __init__(self, model_name_right,model_name_left, callback):
        super().__init__('ros1_bridge')
        
        self.model_name_right = model_name_right
        self.model_name_left = model_name_left

        self.msg           = None
        self.pose_right    = Pose()
        self.pose_left     = Pose()
        self.joint_command_right = Float64MultiArray()
        self.joint_command_left = Float64MultiArray()

        self.gazebo_pose_pub_right = self.create_publisher(Pose, "/from_gazebo_to_rviz_right", 1)
        self.gazebo_pose_pub_left  = self.create_publisher(Pose, "/from_gazebo_to_rviz_left", 1)
        self.joint_right_ros1_pub  = self.create_publisher(Float64MultiArray, "/right_summit/right_ur_joint_group_pos_controller/command", 1)
        self.joint_left_ros1_pub   = self.create_publisher(Float64MultiArray, "/left_summit/left_ur_joint_group_pos_controller/command", 1)
        
        
        self.gazebo_state_sub     = self.create_subscription(ModelStates, "/gazebo/model_states", self.gazebo_state_callback, 1)
        self.joint_right_ros2_sub = self.create_subscription(JointState, "/right/joint_states", self.joint_right_callback, 1)
        self.joint_left_ros2_sub  = self.create_subscription(JointState, "/left/joint_states", self.joint_left_callback, 1)

        self.scene_service = self.create_client(SetEntityState, '/state/set_entity_state', callback_group=callback)
        self.scene_service.wait_for_service()


    def gazebo_state_callback(self, msg):
        self.msg = msg
        self.set_entity_state_left()
        self.set_entity_state_right()

    def joint_right_callback(self, msg):       
        self.joint_command_right.data = msg.position
        self.joint_right_ros1_pub.publish(self.joint_command_right)
    
    def joint_left_callback(self, msg):       
        self.joint_command_left.data = msg.position
        self.joint_left_ros1_pub.publish(self.joint_command_left)

    def set_entity_state_right(self):   
        while (self.msg is None):
            time.sleep(0.0001)

        model_state_right = SetEntityState.Request()
        entity_state_right = EntityState()
        entity_state_right.name = self.model_name_right
        model_state_right.state.name = self.model_name_right
        entity_state_right.reference_frame = "world"

        for i in range(len(self.msg.name)):
            if self.msg.name[i] == "right_summit":
                entity_state_right.pose.position.x = self.msg.pose[i].position.x
                entity_state_right.pose.position.y = self.msg.pose[i].position.y
                entity_state_right.pose.position.z = self.msg.pose[i].position.z

                entity_state_right.pose.orientation.x = self.msg.pose[i].orientation.x
                entity_state_right.pose.orientation.y = self.msg.pose[i].orientation.y
                entity_state_right.pose.orientation.z = self.msg.pose[i].orientation.z
                entity_state_right.pose.orientation.w = self.msg.pose[i].orientation.w

                entity_state_right.twist.linear.x = self.msg.twist[i].linear.x
                entity_state_right.twist.linear.y = self.msg.twist[i].linear.y
                entity_state_right.twist.linear.z = self.msg.twist[i].linear.z

                entity_state_right.twist.angular.x = self.msg.twist[i].angular.x
                entity_state_right.twist.angular.y = self.msg.twist[i].angular.y
                entity_state_right.twist.angular.z = self.msg.twist[i].angular.z

                self.pose_right.position.x = self.msg.pose[i].position.x
                self.pose_right.position.y = self.msg.pose[i].position.y
                self.pose_right.position.z = self.msg.pose[i].position.z

                self.pose_right.orientation.x = self.msg.pose[i].orientation.x
                self.pose_right.orientation.y = self.msg.pose[i].orientation.y
                self.pose_right.orientation.z = self.msg.pose[i].orientation.z
                self.pose_right.orientation.w = self.msg.pose[i].orientation.w

                break

        
        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        
        # Send the transformation
        self.gazebo_pose_pub_right.publish(self.pose_right)

        model_state_right.state = entity_state_right
        self.scene_service.call_async(model_state_right)

    def set_entity_state_left(self):   

        while (self.msg is None):
            time.sleep(0.0001)

        model_state_left = SetEntityState.Request()
        entity_state_left = EntityState()
      
        entity_state_left.name = self.model_name_left
        model_state_left.state.name = self.model_name_left
        entity_state_left.reference_frame = "world"
        
        for i in range(len(self.msg.name)):
            if self.msg.name[i] == "left_summit":
                entity_state_left.pose.position.x = self.msg.pose[i].position.x
                entity_state_left.pose.position.y = self.msg.pose[i].position.y
                entity_state_left.pose.position.z = self.msg.pose[i].position.z

                entity_state_left.pose.orientation.x = self.msg.pose[i].orientation.x
                entity_state_left.pose.orientation.y = self.msg.pose[i].orientation.y
                entity_state_left.pose.orientation.z = self.msg.pose[i].orientation.z
                entity_state_left.pose.orientation.w = self.msg.pose[i].orientation.w

                entity_state_left.twist.linear.x = self.msg.twist[i].linear.x
                entity_state_left.twist.linear.y = self.msg.twist[i].linear.y
                entity_state_left.twist.linear.z = self.msg.twist[i].linear.z

                entity_state_left.twist.angular.x = self.msg.twist[i].angular.x
                entity_state_left.twist.angular.y = self.msg.twist[i].angular.y
                entity_state_left.twist.angular.z = self.msg.twist[i].angular.z

                self.pose_left.position.x = self.msg.pose[i].position.x
                self.pose_left.position.y = self.msg.pose[i].position.y
                self.pose_left.position.z = self.msg.pose[i].position.z

                self.pose_left.orientation.x = self.msg.pose[i].orientation.x
                self.pose_left.orientation.y = self.msg.pose[i].orientation.y
                self.pose_left.orientation.z = self.msg.pose[i].orientation.z
                self.pose_left.orientation.w = self.msg.pose[i].orientation.w

                break

        self.gazebo_pose_pub_left.publish(self.pose_left)

        model_state_left.state = entity_state_left
        self.scene_service.call_async(model_state_left)

   
def main(args=None):
    rclpy.init(args=args)

    callback_group_right   = ReentrantCallbackGroup()
    args_without_ros = rclpy.utilities.remove_ros_args(args)
   
    parser = argparse.ArgumentParser(
        description='model publisher')
    parser.add_argument('-model_name_right')
    parser.add_argument('-model_name_left')

    args = parser.parse_args(args_without_ros[1:])
    model_name_right = args.model_name_right
    model_name_left = args.model_name_left

    executor = MultiThreadedExecutor()

    node = Ros1Bridge(model_name_right, model_name_left, callback_group_right)
    executor.add_node(node)

    executor.spin()
    rclpy.shutdown()
if __name__ == '__main__':
    main()