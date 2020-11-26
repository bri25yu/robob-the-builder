import rospy

from gazebo_msgs.srv import SpawnModel


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
