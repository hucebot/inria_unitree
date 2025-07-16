#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException

class RightHandMarker(Node):
    def __init__(self):
        super().__init__('right_hand_marker')
        self.server = InteractiveMarkerServer(self, "right_hand_goal_marker")
        self.pose_pub = self.create_publisher(PoseStamped, '/right_hand_goal', 1)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.timer = self.create_timer(0.2, self.check_and_spawn_marker)
        self.marker_spawned = False

    def check_and_spawn_marker(self):
        if self.marker_spawned:
            return
        try:
            trans = self.tf_buffer.lookup_transform(
                'pelvis',
                'right_hand_point_contact',
                rclpy.time.Time())
            
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "pelvis"
            int_marker.name = "right_hand_goal"
            int_marker.description = "Right Hand Goal"
            int_marker.scale = 0.2
            
            # Position
            int_marker.pose.position.x = trans.transform.translation.x
            int_marker.pose.position.y = trans.transform.translation.y
            int_marker.pose.position.z = trans.transform.translation.z
            # Orientation
            int_marker.pose.orientation = trans.transform.rotation

            box_marker = Marker()
            box_marker.type = Marker.CUBE
            box_marker.scale.x = 0.05
            box_marker.scale.y = 0.05
            box_marker.scale.z = 0.05
            box_marker.color.r = 0.2
            box_marker.color.g = 0.8
            box_marker.color.b = 0.2
            box_marker.color.a = 1.0

            control = InteractiveMarkerControl()
            control.always_visible = True
            control.markers.append(box_marker)
            int_marker.controls.append(control)

            # Allow 6-DOF movement
            for axis, vec in zip(
                ['x', 'y', 'z'],
                [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
            ):
                for mode in [InteractiveMarkerControl.ROTATE_AXIS, InteractiveMarkerControl.MOVE_AXIS]:
                    c = InteractiveMarkerControl()
                    c.orientation.x = float(vec[0])
                    c.orientation.y = float(vec[1])
                    c.orientation.z = float(vec[2])
                    c.orientation.w = float(vec[3])
                    c.name = f"{mode}_{axis}"
                    c.interaction_mode = mode
                    int_marker.controls.append(c)

            self.server.insert(int_marker)
            self.server.setCallback(int_marker.name, self.process_feedback)
            self.server.applyChanges()
            self.marker_spawned = True

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().info("Waiting for TF: pelvis -> right_hand_point_contact")

    def process_feedback(self, feedback):
        pose = PoseStamped()
        pose.header = feedback.header
        pose.pose = feedback.pose
        self.pose_pub.publish(pose)

def main(args=None):
    rclpy.init(args=args)
    node = RightHandMarker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
