from itertools import product, cycle
from enum import Enum


class El:
    POSE = "pose"
    COLOR = "color"

class Gazebo:
    SPAWN_URDF_MODEL = "/gazebo/spawn_urdf_model"
    DELETE_MODEL = "/gazebo/delete_model"


class Colors(Enum):
    # See http://wiki.ros.org/simulator_gazebo/Tutorials/ListOfMaterials
    BLUE = "Blue"
    RED = "Red"
    ORANGE = "Orange"
    GREEN = "Green"

block_size = 0.045
start_x, start_y = 1, 0
square_side_length = 3  # in number of blocks
indices = list(range(square_side_length))
color_cycle = cycle(Colors)

TEST_STRUCTURE = []
for (i, j), c in zip(product(indices, indices), color_cycle):
    TEST_STRUCTURE.append({
        El.POSE: {"x": start_x + i * block_size, "y": start_y + (j - 1) * block_size, "z": 0},
        El.COLOR: c.value,
    })
TEST_STRUCTURE.append({
    El.POSE: {"x": start_x - block_size, "y": start_y, "z": 0},
    El.COLOR: Colors.RED.value,
})
