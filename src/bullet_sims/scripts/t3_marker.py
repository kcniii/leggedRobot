
import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

import tf
from geometry_msgs.msg import PoseStamped
import math

def main():
    """Create a interactive marker and add controls for all 6 axis of motion
    """
# TODO
    # Create a TF listener
    tf_listener = tf.TransformListener()

    # Wait for the TF data to become available
    tf_listener.waitForTransform("base_link", "arm_right_7_link", rospy.Time(), rospy.Duration(1.0))

    # Retrieve the pose of arm_right_7_joint in the base_link frame
    (trans, rot) = tf_listener.lookupTransform("base_link", "arm_right_7_link", rospy.Time(0))
    norm = math.sqrt(rot[0]**2 + rot[1]**2 + rot[2]**2 + rot[3]**2)
    rot = [rot[0]/norm, rot[1]/norm, rot[2]/norm, rot[3]/norm]

    # Create a pose publisher
    # TODO publisher for pose
    pose_publisher = rospy.Publisher('marker_pose', PoseStamped, queue_size=10)
    
# /TODO

    # Create an InteractiveMarkerServer
    server = InteractiveMarkerServer('marker_server')

    # Create a marker
    print("Creating marker...")
    marker = Marker()
    marker.type = Marker.CUBE
    marker.scale.x = 0.05
    marker.scale.y = 0.05
    marker.scale.z = 0.05
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # Create an InteractiveMarker
    print("Creating InteractiveMarker...")
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "arm_right_7_link"
    # int_marker.pose.position.x = trans[0]
    # int_marker.pose.position.y = trans[1]
    # int_marker.pose.position.z = trans[2]
    # int_marker.pose.orientation.x = rot[0]
    # int_marker.pose.orientation.y = rot[1]
    # int_marker.pose.orientation.z = rot[2]
    # int_marker.pose.orientation.w = rot[3]
    int_marker.pose.position.x = 0
    int_marker.pose.position.y = 0
    int_marker.pose.position.z = 0
    int_marker.pose.orientation.x = 0
    int_marker.pose.orientation.y = 0
    int_marker.pose.orientation.z = 0
    int_marker.pose.orientation.w = 1
    int_marker.scale = 0.2
    int_marker.name = "marker"
    int_marker.description = "Interactive Marker"

    # Create a control for the marker
    print("Creating control...")
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(marker)
    control.orientation_mode = InteractiveMarkerControl.INHERIT
    control.interaction_mode = InteractiveMarkerControl.FIXED
    int_marker.controls.append(control)

# TODO: add new InteractiveMarkerControl() for each axis
    # Create a control for the x-axis
    control_x = InteractiveMarkerControl()
    control_x.orientation.w = 1
    control_x.orientation.x = 1
    control_x.orientation.y = 0
    control_x.orientation.z = 0
    control_x.name = "move_x"
    control_x.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control_x)
    control_x.name = "rotate_x"
    control_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control_x)

    # Create a control for the y-axis
    control_y = InteractiveMarkerControl()
    control_y.orientation.w = 1
    control_y.orientation.x = 0
    control_y.orientation.y = 0
    control_y.orientation.z = 1
    control_y.name = "move_y"
    control_y.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control_y)
    control_x.name = "rotate_y"
    control_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control_y)

    # Create a control for the z-axis
    control_z = InteractiveMarkerControl()
    control_z.orientation.w = 1
    control_z.orientation.x = 0
    control_z.orientation.y = 1
    control_z.orientation.z = 0
    control_z.name = "move_z"
    control_z.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
    int_marker.controls.append(control_z)
    control_x.name = "rotate_z"
    control_x.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    int_marker.controls.append(control_z)
# /TODO

    # Add the InteractiveMarker to the server
    print("Adding InteractiveMarker to server...")
    server.insert(int_marker, lambda feedback: handle_feedback(feedback))

    # Function to handle feedback from the InteractiveMarker
    def handle_feedback(feedback):
        # TODO publish pose to ros
        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = feedback.header.frame_id
        pose_stamped.pose = feedback.pose
        # Publish the pose
        pose_publisher.publish(pose_stamped)

    # Start the server
    print("Starting server...")
    server.applyChanges()

    # Spin the node
    print("Spinning node...")

    # TODO
    rospy.spin()
    
if __name__=="__main__":
    rospy.init_node('interactive_marker_node')
    rospy.sleep(1.0)
    main()
