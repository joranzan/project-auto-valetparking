# ROS 통신 환경 구성

---

- 시뮬레이터에서 ROS 통신을 하기 위한 사전 구성

### ROS 및 종속 프로그램 설치

---

- OS : Ubuntu-18.04 (WSL2)
    
    ```bash
    # ros-latest.list에 ROS 저장소를 추가
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc ) main" > /etc/apt/sources.list.d/ros-latest.list’
    $ sudo apt-get install curl
    # ROS 저장소로부터 패키지를 내려받기 위해 공개키를 추가
    $ curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –
    $ sudo apt-get update
    $ sudo apt-get upgrade
    # 데스크탑용 ROS 패키지 및 rqt 패키지 설치
    $ sudo apt install ros-melodic-desktop-full
    $ sudo apt-get install ros-melodic-rqt*
    $ sudo apt-get install ros-melodic-velodyne
    #기타 도구 및 필요 패키지 설치
    $ sudo apt-get install git
    $ sudo apt-get install net-tools
    $ sudo apt-get install python-pip
    $ pip install scikit-learn
    ```
    
- 터미널을 실행할 때 마다 Bash 설정을 위해 아래 코드 실행
    
    ```bash
    $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```
    

- 의존성 패키지 설치
    
    ```bash
    $ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
    # rosdep 초기화
    $ sudo apt install python-rosdep
    $ sudo rosdep init
    $ rosdep update
    ```
    

### ROS 작업 공간 구성

---

- catkin_ws 폴더 생성
    
    ```bash
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
    $ catkin_init_workspace
    $ cd ~/catkin_ws/
    $ catkin_make
    ```
    
- 자율 주행 예제 코드 + 메시지 파일 다운
    
    ```bash
    $ cd ~/catkin_ws/src
    # MORAI ROS 통신 예제 코드 및 메시지 파일 다운로드
    $ git clone https://github.com/MORAI-Autonomous/MORAI-DriveExample_ROS.git
    $ git clone https://github.com/MORAI-Autonomous/MORAI-ROS_morai_msgs
    # 센서 데이터 파싱 및 Visualization 예제 파일 다운로드
    $ git clone https://github.com/MORAI-Autonomous/MORAI-SensorExample.git 
    # Package 및 message build
    $ cd ..
    $ catkin_make
    # catkin 환경 변수 선언
    $ source ~/catkin_ws/devel/setup.bash
    # catkin 패키지 재구축
    $ rospack profile
    ```
    
- rosbridge 및 기타 패키지 설치
    
    ```bash
    $ sudo apt-get install ros-melodic-rosbridge-server
    $ sudo apt install terminator
    $ pip install pyproj
    $ sudo apt install libvulkan1
    $ cd ~/catkin_ws
    $ catkin_make
    ```
    

- Bash 환경 재 설정
    
    ```bash
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    $ source ~/.bashrc
    ```
    
- 재부팅(권장)
    
    ```bash
    $ sudo reboot
    ```
    
- 시뮬레이터 연동 확인
    
    ```bash
    $ roslaunch rosbridge_server rosbridge_websocket.launch
    ```
    
    - 터미널로 rosBridge 실행
    - 시뮬레이터에서 맵에 접속한 다음 ‘F4’를 눌러 네트워크 세팅
    - IP를 입력하고 Connect 클릭 (위 세팅은 local에서 동작하므로 127.0.0.1 입력)
    
-