#!/usr/bin/env python
import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
import moveit_commander
print("here!")
moveit_scene = moveit_commander.PlanningSceneInterface()
#TODO: should add to move it and to gazebo
def add_block(block_pose, block_reference_frame):
    #add to gazebo
    model_path = rospkg.RosPack().get_path('world_simulation')+"/models/"
    # Load Block URDF
    block_xml = ''
    with open (model_path + "block/block.urdf", "r") as block_file:
        block_xml=block_file.read().replace('\n', '')
    print(block_xml)
    exit()
    # Spawn Block URDF
    rospy.wait_for_service('/gazebo/spawn_urdf_model')
    try:
        spawn_urdf = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        resp_urdf = spawn_urdf("block", block_xml, "/",
                               block_pose, block_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn URDF service call failed: {0}".format(e))

    #add to moveit
    moveit_scene.add_box("box", PoseStamped(Header(frame_id = block_reference_frame), block_pose), (.45, .45, .45))
block_pose = Pose(position=Point(x=0.4225, y=-0.1265, z=0.7725))
add_block(block_pose, "world")
