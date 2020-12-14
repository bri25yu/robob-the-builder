import numpy as np

import rospy

from gazebo_msgs.srv import SetModelState, SpawnModel
from gazebo_msgs.msg import ModelState, ModelStates


def get_content_between(s, start_flag, end_flag):
    """
    Returns the substring from the first instance of the input start_flag
    to the first instance of end_flag.

    Parameters
    ----------
    s: str
    start_flag: str
    end_flag: str

    """
    start_i = s.find(start_flag)
    end_i = s.find(end_flag, start_i)
    return s[start_i + len(start_flag): end_i]


def add_block_gazebo(service_name, xml, pose, reference_frame, name):
    rospy.wait_for_service(service_name)
    try:
        spawn = rospy.ServiceProxy(service_name, SpawnModel)
        spawn(name, xml, "/", pose, reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("{} service call failed: {}".format(service_name, e))


def move_camera_gazebo(name, pose):
    state_msg = ModelState()
    state_msg.model_name = name
    state_msg.pose = pose
    flattened_pose = flatten_pose(pose)
    link_name = "{}::link".format(name)

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state(state_msg)

        def check(msg, moved):
            moved[0] = np.all(np.isclose(flatten_pose(msg.pose[msg.name.index(link_name)]), flattened_pose))

        moved = [False]
        model_state_sub = rospy.Subscriber("/gazebo/link_states", ModelStates, check, moved)
        while not moved[0]:
            pass
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e


def flatten_pose(pose):
    return np.array([pose.position.x, pose.position.y, pose.position.z,\
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
