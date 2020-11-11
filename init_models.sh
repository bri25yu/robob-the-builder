# Camera files from http://models.gazebosim.org/camera/
# SDF format reference http://sdformat.org/spec?ver=1.7&elem=sensor
# Maybe useful for how to use SDF files? http://gazebosim.org/tutorials?tut=ros_roslaunch

rm -rf ~/.gazebo/models/camera
cp -R ./src/world_simulation/models/camera ~/.gazebo/models
