from geometry_msgs.msg import Pose
import math

def compare_poses(pose1, pose2, position_threshold, orientation_threshold):
    """
    Compare two poses to determine if they are similar within given thresholds.

    :param pose1: The first pose to compare.
    :param pose2: The second pose to compare.
    :param position_threshold: The maximum distance allowed between the positions of the two poses.
    :param orientation_threshold: The maximum angle difference allowed between the orientations of the two poses.
    :return: True if the poses are similar within the thresholds, False otherwise.
    """
    # Calculate position difference
    position_diff = math.sqrt(
        (pose1.position.x - pose2.position.x) ** 2 +
        (pose1.position.y - pose2.position.y) ** 2 +
        (pose1.position.z - pose2.position.z) ** 2
    )

    # Calculate orientation difference (simple method, more complex calculations may involve quaternions)
    orientation_diff = math.sqrt(
        (pose1.orientation.x - pose2.orientation.x) ** 2 +
        (pose1.orientation.y - pose2.orientation.y) ** 2 +
        (pose1.orientation.z - pose2.orientation.z) ** 2 +
        (pose1.orientation.w - pose2.orientation.w) ** 2
    )

    return position_diff <= position_threshold and orientation_diff <= orientation_threshold
