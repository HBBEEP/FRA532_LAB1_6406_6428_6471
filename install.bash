Green="\033[32m"
Cyan="\033[36m"
NC="\033[0m"

echo -e "${Cyan}Controller packages installing...${NC}"
sudo apt -y install ros-humble-controller-manager ros-humble-joint-state-broadcaster ros-humble-ros2-control ros-humble-joint-trajectory-controller ros-humble-velocity-controllers ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control  ros-humble-xacro ros-humble-diff-drive-controller ros-humble-gazebo-ros-pkgs
echo -e "${Green}Controller packages were installed successfully${NC}"