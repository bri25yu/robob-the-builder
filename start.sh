if [ $1 -ne 1 ]
then
    source ~/tiago_public_ws/devel/setup.bash
    rm -rf build devel

    catkin_make
    source devel/setup.bash
    chmod +x src/world_simulation/src/add_block.py
fi

rosrun world_simulation add_block.py
