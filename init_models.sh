# Camera files from http://models.gazebosim.org/camera/
# SDF format reference http://sdformat.org/spec?ver=1.7&elem=sensor
# Maybe useful for how to use SDF files? http://gazebosim.org/tutorials?tut=ros_roslaunch

rm -rf ~/.gazebo/models/kinect
rm -rf ~/.gazebo/models/kinect2
cp -R ./src/world_simulation/models/kinect ~/.gazebo/models
cp -R ./src/world_simulation/models/kinect2 ~/.gazebo/models
