# ROS
## ROS(Robot Operating System)
- 로봇 소프트웨어를 개발하기 위한 소프트웨어 프레임워크
- Low Level 제어, 하드웨어 추상화, 프로세스간 메시지 전달 기능 등을 제공한다.
- Node라고 하는 프로세스의 분산 프레임워크

### 메시지와 토픽
- 메시지를 주고 받기 위해서는 토픽(메시지의 이름)과 메시지 타입을 맞춰 주어야 한다.
- Mater 노드를 통해 Publisher와 Subscriber의 통신이 수행된다.
- Publish는 하나의 노드에서만 하지만 Subscribe는 여러 노드에서 가능하다.


### Rosbridge
- ROS 메시지와 JSON 데이터의 상호 변환을 수행하는 ROS 패키지이다.
- ROS Local ip 내부에서만 아니라 외부 프로그램과 통신을 할 수 있도록 하는 역할을 한다.
- 시뮬레이터를 연결하면 ROS 메시지 형식에 맞춰 JSON 데이터를 전송하고, rosbridge를 통해 ROS 형식의 메시지로 수신한다.

### rviz, rqt
- 시각화를 위한 툴로서, 토픽을 수신하면 데이터 타입에 따라 시각화한다.
- rqt_plot: 수신한 토픽으로 그래프를 그림
- rviz: 라이다, tf, image 등의 메시지를 3D로 시각화함
- rqt_graph: 노드와 메시지와의 관계를 한 눈에 보기 쉽게 그래프를 그림