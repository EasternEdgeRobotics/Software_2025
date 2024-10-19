source /opt/ros/jazzy/setup.sh

# TODO: Add new launch script for 2025 bot
if [ -e /app/install/setup.sh ]
then 
  source /app/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml 
else
  cd /app
  colcon build
  source /app/install/setup.sh
  ros2 launch beaumont_pkg beaumont_startup.xml
fi
