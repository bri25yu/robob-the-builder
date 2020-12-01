if [ $1 -ne 1 ]
then
    source ~/tiago_public_ws/devel/setup.bash
    rm -rf build devel

    catkin_make
    source devel/setup.bash

    chmod +x src/world_simulation/src/world_simulation.py
    chmod +x src/world_simulation/src/take_photos.py
    chmod +x src/generate_schematic/src/generate_schematic.py

    chmod +x stop_gazebo.sh
fi
source devel/setup.bash

# Start necessary services: roscore and Gazebo
roscore &
rosrun gazebo_ros gazebo &

# Create world. Make any world modifications in src/global_constants/src/constants.py
rosrun world_simulation world_simulation.py
rosrun world_simulation take_photos.py

# Generate the schematic to build
rosrun generate_schematic generate_schematic.py

# Shut down services
./stop_gazebo.sh
killall -9 roscore & killall -9 rosmaster
