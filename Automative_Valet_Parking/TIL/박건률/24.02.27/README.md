# Today I learned

# wsl Ubuntu-18.04 설치
```
wsl --install -d Ubuntu-18.04
```

# wsl 실행
```
wsl
```

# ros-latest.list에 ROS 저장소를 추가
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc ) main" > /etc/apt/sources.list.d/ros-latest.list'
```
```
sudo apt-get install curl
```
# ROS 저장소로부터 패키지를 내려받기 위해 공개키를 추가
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
```
sudo apt-get update
sudo apt-get upgrade -y
```
# 데스크탑용 ROS 패키지 및 rqt 패키지 설치
```
sudo apt install ros-melodic-desktop-full
sudo apt-get install ros-melodic-rqt*
sudo apt-get install ros-melodic-velodyne
sudo apt-get install ros-melodic-rosbridge-server
```
# 기타 도구 및 필요 패키지 설치
```
sudo apt-get install git
sudo apt-get install net-tools
sudo apt-get install python-pip
sudo apt install terminator
sudo apt install libvulkan1
```
```
pip install pyproj
pip install scikit-learn
```

# 설치한 ROS의 사용을 위해 환경 설정 파일을 불러온다.
```
source /opt/ros/melodic/setup.bash
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

# ROS 패키지 빌드 시 필요한 의존성 패키지 설치 및 초기화한다.
```
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```
# rosdep 초기화
```
sudo apt install python-rosdep
```
```
sudo rosdep init
rosdep update
```

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
cd ~/catkin_ws/
catkin_make
```

# MORAI ROS 통신 예제 코드 및 메시지 파일 다운로드 및 센서 데이터 파싱 및 Visualization 예제 파일 다운로드
```
cd ~/catkin_ws/src
git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_ROS.git
git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs
git clone https://github.com/MORAI-Autonomous/MORAI-SensorExample.git 
```
# Package 및 message build
```
cd ~/catkin_ws/
catkin_make
```
# catkin 환경 변수 선언
```
source ~/catkin_ws/devel/setup.bash
```
# catkin 패키지 재구축
```
rospack profile
```