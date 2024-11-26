# ROS2 Humble 환경 세팅

---

- BASH 환경 세팅
    
    ---
    
    - bash 환경 셋업
        
        ```bash
        source /opt/ros/humble/setup.bash
        ```
        
    

참조 문서 : [https://docs.ros.org/en/humble/index.html](https://docs.ros.org/en/humble/index.html)

- 참조 문서의 Installation을 따라서 설치

### Locale 설정

---

- UTF-8로 인코딩 형식을 변경해줘야 함

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```

### ROS2 패키지 설치

---

```bash

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt upgrade

sudo apt install ros-humble-desktop

source /opt/ros/humble/setup.bash
```

### 설치 테스트

---

- 두 개의 터미널을 실행하여 각각 talker와 listener를 실행시켜 연결 여부 확인

```bash
ros2 run demo_nodes_cpp talker
```

```bash
ros2 run demo_nodes_py listener
```

### bash 환경 자동 세팅

---

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### 환경 설정 확인

---

```bash
printenv | grep -i ROS
```

- 위 명령을 실행한 다음 아래 변수가 올바르게 적용되어 있는지 확인할 것
    
    ```bash
    ROS_VERSION=2
    ROS_PYTHON_VERSION=3
    ROS_DISTRO=humble
    ```
    

### [Localhost](http://Localhost) 제한

---

- ROS 2 통신을 Localhost로 제한할 경우 사용
    
    ```bash
    export ROS_LOCALHOST_ONLY=1
    
    echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
    ```