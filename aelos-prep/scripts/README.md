#The steps of running ROS2 for the new computers 

## 1. Download WSL in windows_10 or windows_11
- open PowerShell(mannager),execute`wsl --install Ubuntu-22.04`,if you want to use Ubuntu-24.04,the ROS2's name should switch to version_JAZZY
- And then,set up your users_name and the password for your own Ubuntu

## 2. Download ROS2 humble(for 22.04) or jazzy(for 24.04)

-1. Open Ubuntu in WSL,update the system:
	```bash 
	sudo apt update && sudo apt upgrade -y ```
-2. Put up your Locale:
	```bash
	locale

	sudo apt update && sudo apt install locales
	sudo locale-gen en_US en_US.UTF-8
	sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
	export LANG=en_US.UTF-8

	locale```
-3. Put up the software:
	```bash
	sudo apt install software-properties-common
	sudo add-apt-repository universe```
-4. Put up the git of ROS2's github to achieve auto-update:
	```bash
	sudo apt update && sudo apt install curl -y
	export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastrucure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
	curl -L -o /tmp/ros2-apt-source.deb "http://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
	sudo dpkg -i /tmp/ros2-apt-source.deb```
-5. Put up the tools_package:
	```base
	sudo apt update && sudo apt install ros-dev-tools```
-6. Update APT
	```base
	sudo apt update
	sudo apt upgrade```
-7. Install ROS2_desktop system
	```base
	sudo apt install ros-humble-desktop```
-8. Set up the source with bash
	```base
	source /opt/ros/humble/setup.bash```
or use the `scripts/setup_env.sh`.
-9. Test the source of ubuntu's ROS2
	```base
	sudo apt install -y ros-humble-demo-nodes-cpp
	ros2 run demo_nodes_cpp talker
	ros2 --help
	ros2 topic list
	ros2 -h```
 
