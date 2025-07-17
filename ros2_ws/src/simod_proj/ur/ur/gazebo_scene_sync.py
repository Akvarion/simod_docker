#!/usr/bin/env python3

import rclpy,time,trimesh, tf_transformations
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates, LinkStates  # or appropriate message for your setup
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive,Mesh,MeshTriangle
from geometry_msgs.msg import Pose,Point, Quaternion, TransformStamped
from visualization_msgs.msg import Marker
from moveit_msgs.msg import ObjectColor, PlanningScene
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster

class GazeboSceneSync(Node):
    def __init__(self):
        super().__init__('gazebo_scene_sync')
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.planning_scene_pub = self.create_publisher(PlanningScene, '/monitored_planning_scene', 10)

        self.subscription = self.create_subscription(
            ModelStates,
            '/state/model_states',
            self.model_states_callback,
            10
        )
        self.subscription = self.create_subscription(
            LinkStates,
            '/state/link_states',
            self.link_states_callback,
            10
        )
        self.collision_object_pub = self.create_publisher(
            CollisionObject,
            '/collision_object',
            10
        )
    def broadcast_tf(self, pose, parent_frame, child_frame):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z
        t.transform.rotation = pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish_collision_object(self, co):
        # Publish the collision object to the planning scene
        self.collision_object_pub.publish(co)
        self.get_logger().info(f"Published collision object: {co.id}")
    
    def rviz_load_mesh(self, co, mesh_path):
        # Load the mesh from the specified path
        mesh = trimesh.load_mesh(mesh_path)

        # Create a ROS mesh message
        ros_mesh = Mesh()
        for v in mesh.vertices:
            p = Point()
            p.x, p.y, p.z = float(v[0]), float(v[1]), float(v[2])
            ros_mesh.vertices.append(p)

        for t in mesh.faces:
            mesh_t = MeshTriangle()
            mesh_t.vertex_indices = [int(t[0]), int(t[1]), int(t[2])]
            ros_mesh.triangles.append(mesh_t)

        # Set the meshes of the collision object
        co.meshes = [ros_mesh]

    def rotate_pose_z_90(self, pose):
        # Convert pose orientation to quaternion
        q_orig = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        # 90 deg in radians
        q_rot = tf_transformations.quaternion_from_euler(0, 0, -1.5708)  # -90 deg
        # Multiply quaternions
        q_new = tf_transformations.quaternion_multiply(q_rot, q_orig)
        pose.orientation.x = q_new[0]
        pose.orientation.y = q_new[1]
        pose.orientation.z = q_new[2]
        pose.orientation.w = q_new[3]
        return pose

    # Allows for colored collision objects in the planning scene
    # Example usage:
    # self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))  # Red
    def publish_colored_object(self, co, color_rgba):
        # Initialize the planning scene message
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True
        planning_scene_msg.world.collision_objects.append(co)

        object_color = ObjectColor()
        object_color.id = co.id
        object_color.color = ColorRGBA(r=color_rgba[0], g=color_rgba[1], b=color_rgba[2], a=color_rgba[3])
        planning_scene_msg.object_colors.append(object_color)

        self.planning_scene_pub.publish(planning_scene_msg)
        self.get_logger().info(f"Published colored collision object: {co.id}")

    def model_states_callback(self, msg):
        for name, pose in zip(msg.name, msg.pose):
            match name:
                case "pallet2": 
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pallet2"
                    self.rviz_load_mesh(co, '/ros2_ws/install/ur/share/ur/xacro/./Models/pallet/meshes/pallet.dae')
                    co.mesh_poses = [pose]  # Use the pose from Gazebo

                    co.operation = CollisionObject.ADD

                    self.publish_colored_object(co, (0.82, 0.71, 0.55, 1.0))
                    self.broadcast_tf(pose, "world", "pallet2")

                case "aws_robomaker_warehouse_GroundB_01_0" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "warehouse_ground"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [14.0, 20.9, 0.123316]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD

                    self.publish_colored_object(co, (0.8, 0.7, 1.0, 1.0))

                case "target_plane" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "target_plane"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [1.97121, 1.96626, 0.019484]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co)
                    self.broadcast_tf(pose, "world", "target_plane")

                case "pacco" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))
                    self.broadcast_tf(pose, "world", "pacco")

                case "pacco_clone" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))
                    self.broadcast_tf(pose, "world", "pacco_clone")

                case "pacco_clone_0" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_0"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))
                    self.broadcast_tf(pose, "world", "pacco_clone_0")

                case "pacco_clone_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_1"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))
                    self.broadcast_tf(pose, "world", "pacco_clone_1")

                case "pacco_clone_2" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_2"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_colored_object(co, (1.0, 0.0, 0.0, 1.0))
                    self.broadcast_tf(pose, "world", "pacco_clone_2")

                case "muraUse1":
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "muraUse1"
                    
                    self.rviz_load_mesh(co, '/ros2_ws/assets/usecase1(mura+pallet).stl')
                    pose=self.rotate_pose_z_90(pose)  # Rotate the pose by 90 degrees around Z-axis
                    co.mesh_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD

                    self.publish_collision_object(co)
                    self.broadcast_tf(pose, "world", "muraUse1")
                case _:
                    self.get_logger().info(f"Model {name} not handled in callback.")

    def link_states_callback(self, msg):
        # Example: update a box named "box::box_link" in the planning scene
        for name, pose in zip(msg.name, msg.pose):
            match name:
                case "pacco::link_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co)
                
                case "pacco_clone::link_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co)
                
                case "pacco_clone_0::link_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_0"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co)
                
                case "pacco_clone_1::link_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_1"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co) 
                
                case "pacco_clone_2::link_1" :
                    co = CollisionObject()
                    co.header.frame_id = "world"
                    co.id = "pacco_clone_2"
                    primitive = SolidPrimitive()
                    primitive.type = SolidPrimitive.BOX
                    primitive.dimensions = [0.8, 0.3, 0.2]  # Fill in actual dimensions
                    co.primitives = [primitive]
                    co.primitive_poses = [pose]  # Use the pose from Gazebo
                    co.operation = CollisionObject.ADD
                    self.publish_collision_object(co)
                
                case _:
                    self.get_logger().info(f"Model {name} not handled in callback.")
                

def main(args=None):
    rclpy.init(args=args)
    node = GazeboSceneSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()