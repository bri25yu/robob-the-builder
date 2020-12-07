from enum import Enum


class Gazebo:
    SPAWN_URDF_MODEL = "/gazebo/spawn_urdf_model"
    SPAWN_SDF_MODEL = "/gazebo/spawn_sdf_model"


class Colors(Enum):
    # See http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
    BLUE = "Blue"
    RED = "Red"
    ORANGE = "Orange"
    GREEN = "Green"
    DARKGRAY = "DarkGrey"
    REDBRIGHT = "RedBright"
    SKYBLUE = "SkyBlue"
    YELLOW = "Yellow"
    INDIGO = "Indigo"
    PURPLE = "Purple"
    TURQUOISE = "Turquoise"


class Block:
    class Color:
        DEFAULT = "<material>Gazebo/Red</material>"
        TEMPLATE = "<material>Gazebo/{}</material>"

    class ColorGeometry:
        INITIAL = "{COLOR_REPLACE_GEOMETRY}"
        ARUCO = '<mesh filename="package://world_simulation/models/aruco_cube/meshes/aruco_cube_5cm.dae" scale="1 1 8"/>'
        COLORED = '<box size="0.05 0.05 0.40" />'

    class ColorMaterial:
        INITIAL = "{COLOR_REPLACE_MATERIAL}"
        ARUCO = ""
        COLORED = '<gazebo reference="block">' +\
        "<material>Gazebo/Red</material>" +\
        "<mu1>1000</mu1>" +\
        "<mu2>1000</mu2>" +\
      "</gazebo>"

    class Size:
        START_FLAG = "<box size="
        END_FLAG = "/>"

    class Inertia:
        DEFAULT = '<inertia  ixx="0.0" ixy="0.0"  ixz="0.0"  iyy="0.0"  iyz="0.0"  izz="0.0" />'
        TEMPLATE = '<inertia  ixx="{}" ixy="0.0"  ixz="0.0"  iyy="{}"  iyz="0.0"  izz="{}" />'

    class Mass:
        START_FLAG = "<mass value="
        END_FLAG = "/>"

    class Origin:
        DEFAULT = '<origin xyz="0.0 0.0 0.0" />'
        TEMPLATE = '<origin xyz="{} {} {}" />'


CAMERA_DEFAULT_NAME = "{INPUT_CAMERA_NAME}"


BLOCK_URDF_PATH = "block/block.urdf"
CAMERA_SDF_PATH = "kinect/model.sdf"
