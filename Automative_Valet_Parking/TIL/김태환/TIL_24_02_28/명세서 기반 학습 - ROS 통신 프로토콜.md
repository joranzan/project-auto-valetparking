# 명세서 기반 학습 - ROS 통신 프로토콜

---

## ROS (Robot Operating System)

---

### ROS 개요

---

- 로봇 S/W를 개발하기 위한 S/W 프레임워크
- Node라고 하는 프로세스의 분산 프레임워크
- 실행 프로그램을 독립적으로 설계 → 프로세스간 결합도를 낮춤
- C++ / Python 지원

### 메시지, 토픽

---

- 각 Node들은 마스터를 통해서 데이터를 주고 받을 수 있음
- 보낼 데이터(Message)와 주제(Topic)를 마스터로 전송하고, 원하는 Topic에 대한 데이터를 마스터로 부터 받음
- 데이터를 전송하는 Publisher와 데이터를 수신(구독)하는 Subscriber가 있으며, 각 Node들은 pub, sub에 대한 정보를 가지고 있음

Ex. node1이 “test”라는 Topic으로 “hi”라는 데이터를 담아 Publish하면,

마스터는 test라는 주제를 구독(Subscribe)하는 다른 Node에게 “hi” 데이터를 전송

### RosBridge

---

- 외부 IP와 ROS 통신을 할 수 있도록 연결하는 역할
    
    ```jsx
    rosbridge rosbridge_server rosbridge_websocket.launch
    ```
    
    - 연결된 IP의 WebSocket에 연결하여 JSON 형식의 데이터를 송수신할 수 있는 환경을 구축

### Rviz, RQT

---

- 메시지를 발행하면 수신된 데이터를 시각화하여 보여줌
- RQT_plot : Topic을 이용해 그래프 생성
- RQT_graph : 노드와 메시 관계 보기 쉽게 그래프 그려줌
- Rviz : 라이다, tf, image등의 메시지를  3D로 시각화

## 센서(Sensor)

---

### 센서 개요

---

- 자율주행에 사용되는 센서는 5종류가 주류
    - 카메라(Camera)
    - 라이다(LiDAR)
    - 레이더(Radar)
    - GPS(Global Positioning System)
    - IMU(Interial Measurement Unit)
    
- Ground Truth
    - 직접 관찰 및 측정에 의해 제공되는 실제 or 사실로 알려진 정보
    - AI 에서 모델 출력 값 훈련 및 테스트에 사용
    - 영상 데이터, 음향신호도 포함

### 카메라(Camera)

---

- 시각적으로 보이는 다양한 정보를 얻음
- 객체 인식을 위해 여러 연산이 필요
    - 표지판, 신호등 등 다른 센서로 얻을 수 없는 정보를 객체탐지로 습득 가능
    - 거리 추정이 힘들며, 환경에 영향을 많이 받음

### 라이다(LiDAR)

---

- 레이저 펄스를 쏘아 반사되어 돌아오는 시간을 이용하여 거리를 인지
- 정밀도가 높고 형태 인식이 가능
- 정확한 3D 이미지 제공
- 비싸고, 크며, 환경 변화에 약함 → 점점 효율이 좋아지는 중

### 레이더(Radar)

---

- 레이저 전파를 매개체로 사물간 거리 형태 파악
- 라이다에 비해 소형화 가능, 날씨 영향 적음
- 라이다에 비해 성능은 좋지 못함

### GPS(Global Positioning System)

---

- 여러 중 궤도 위성망에서 보내는 신호를 수신하여 받은 시간을 측정
- 측정 결과 순간의 위치를 계산하여 거리 특정
- 최소 24개 이상의 위성으로 연산

### IMU(Inertial Measurement Unit)

---

- 관성 측정 장비라고 불림
- 가속도계, 회전 속도계, 자력계의 조합을 사용 → 힘과 방향 자기장을 측정
- 구성
    - 자이로 센서 : 3축 각 속도 측정
    - 가속도 센서 : 3축 가속도 측정
    - 지자기 센서 : 지구의 자기장 측정
- 시간이 지날수록 오차가 누적되어 정확도가 떨어지는 단점 존재

### Ground Truth

---

- 직접 관찰 및 측정에 의해 제공되는 실제 or 사실로 알려진 정보
- 카메라, 라이다, 레이더는 시뮬레이터에서 3가지 GT 선택 가능
- 종류
    - RGB
        - 빛의 3원색
        - 카메라 센서를 통해 나오는 기본 이미지
    - Semantic
        - class based segmentation 된 객체들끼리 구분하여 보여줌
        - 객체들을 묶어 주행할 수 있는 영역을 판단
    - Instance
        - instance based segmentation 된 객체를 각 객체별 식별함
        - 각 객체에 대해 바운딩할 수 있고, 위치/속도 정보를 가지고 있음
        - Semantic과 유사하나 리소스를 많이 먹음 → 적절히 사용해야 함
    - Intensity
        - LiDAR, 레이더에서 사용
        - 펄스나 전파가 물체에 반사되어 돌아오는 신호의 강도에 따라 색깔 다르게 표현
        - 물체 표면 거칠기, 각도에 따라 반사율 다르게 적용
    

## 개발 환경 구성

---

### 기본 환경 세팅

---

ROS 환경 세팅 페이지

- 위 Guide에 따라 WSL / Ubuntu / ROS / 종속 패키지 설치
- 설치 후 ROS workspace 생성 (Guide에서 생성한 catkin_ws WorkSpace 사용해도 무관)

```bash
$ mkdir -p catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ cd ~/catkin_ws
$ catkin_make
```

- 스켈레톤 코드 clone 후 WorkSpace에 추가

```bash
$ git clone [스켈레톤 코드 github 주소]
# workspace의 패키지 목록에 ssafy_1 폴더 추가
$ cp -r [옮길 폴더] [타겟 경로]
$ 
$ cd ~/catkin_ws
$ catkin_make
```