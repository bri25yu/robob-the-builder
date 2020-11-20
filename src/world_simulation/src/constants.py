from enum import Enum


class Gazebo:
    SPAWN_URDF_MODEL = "/gazebo/spawn_urdf_model"
    DELETE_MODEL = "/gazebo/delete_model"


class Colors(Enum):
    # See http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
    BLUE = "Blue"
    RED = "Red"
    ORANGE = "Orange"
    GREEN = "Green"
