# 명세서 기반 학습 - 기능 명세

---

### 목차

1. 위치 인식
2. 정밀도로 지도
3. 경로 계획
4. 판단/제어

### 위치 인식

---

| req 1-1 | Localization | GPS 경위도 데이터 → UTM좌표계로 변환  |
| --- | --- | --- |
| req 1-2  | Odometry ROS | UTM좌표계, 쿼터니언 자세 데이터 → ROS Odometry 메시지 형식으로 차량데이터 변환 후 송신  |
| req 1-3 | ROS TF 좌표계 | ROS Odometry 메시지 수신 → TF 브로드캐스터 생성, TF 브로드캐스팅 |

### Localization

- pyproj 라이브러리 설치

```python
pip install pyproj
```

- pyproj 라이브러리를 사용하여 GPS 데이터 → UTM 좌표계로 변환

```python
self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
# proj = "변환할 좌표계", zone= UTM좌표계 zone 번호 (한국은 52), ellps="입력 좌표계"
# preserve_units = True : 입력한 좌표 단위를 유지 / False : 일정한 단위로 변경하여 변환
```

- Subscribe 중인 Topic의 메시지 형식 확인 → callback 함수에서 경위도 데이터 수신

```python
# GPSMessage.msg

Header header

float64 latitude
float64 longitude
float64 altitude

float64 eastOffset
float64 northOffset
int16 status
```

```python
self.lat = gps_msg.latitude 
self.lon = gps_msg.longitude
```

<aside>
💡 Float32MultiArray()

- ROS에서 사용하는 메시지 유형 중 하나
- header와 data로 이루어짐
- data에는 Float32 배열 데이터를 넣을 수 있음
- utm 좌표를 받기 위한 빈 객체를 생성함
</aside>

- convert 메서드

```python
def convertLL2UTM(self):
        
        xy_zone = self.proj_UTM(self.lat, self.lon)

        self.x = xy_zone[0]
        self.y = xy_zone[1]
```

### Odometry ROS

---

- gps_parser Node에서 IMU 데이터 추가
- GPS와 IMU 데이터로 차량의 위치 자세 데이터 획득
    
    ```python
    self.gps_sub = rospy.Subscriber("/gps", GPSMessage, self.navsat_callback)
    self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
    ```
    
- ROS Odometry 형식으로 Publish
    
    ```python
    self.odom_pub = rospy.Publisher('/odom',Odometry, queue_size=1)
    ```
    

- Odometry 메시지 형식
    
    ```python
    Header header
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
    geometry_msgs/TwistWithCovariance twist
    ```
    
    - geometry_msgs/PoseWithCovariance
        
        ```python
        Pose pose
        float64[36] covariance
        ```
        
        - Pose
            
            ```python
            Point position
            Quaternion orientation
            ```
            
            - Point
                
                ```python
                float64 x
                float64 y
                float64 z
                ```
                
            - Quaternion
                
                ```python
                float64 x
                float64 y
                float64 z
                float64 w
                ```
                
    - geometry_msgs/TwistWithCovariance
        
        ```python
        Twist twist
        float64[36] covariance
        ```
        
        - Twist
            
            ```python
            Vector3  linear
            Vector3  angular
            ```
            

- UTM 변환 객체
    
    ```python
    self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)
    ```
    
    - UTM 변환 함수
        
        ```python
        def convertLL2UTM(self):
                
            xy_zone = self.proj_UTM(self.lon, self.lat)
        
            if self.lon == 0 and self.lat == 0:
                self.x = 0.0
                self.y = 0.0
            else:
                self.x = xy_zone[0] - self.e_o
                self.y = xy_zone[1] - self.n_o
        
            self.odom_msg.header.stamp = rospy.get_rostime()
            self.odom_msg.pose.pose.position.x = self.x
            self.odom_msg.pose.pose.position.y = self.y
            self.odom_msg.pose.pose.position.z = 0.0
        ```
        
        - position 값은 GPS에서 받아오는 x, y 값을 사용
        - z값은 차가 공중에 떠있는 것이 아니니 0.0 입력
        - GPS 값은 offset을 적용하여 연산해야 함
            - 시뮬레이터와 실제 좌표 간 차이 보간
            - gps 센서에서 보내줌
        
- Odometry 메시지 저장 객체 생성
    
    ```python
    self.odom_msg = Odometry()
    self.odom_msg.header.frame_id = '/odom' # <- Odometry 메시지 형식에 포함 frame_id : 해당 객체에 대한 ID
    self.odom_msg.child_frame_id = '/base_link' # <- ??
    ```
    

- IMU 센서 데이터 저장
    
    ```python
    def imu_callback(self, data):
    
        if data.orientation.w == 0:
            self.odom_msg.pose.pose.orientation.x = 0.0
            self.odom_msg.pose.pose.orientation.y = 0.0
            self.odom_msg.pose.pose.orientation.z = 0.0
            self.odom_msg.pose.pose.orientation.w = 1.0
        else:
            self.odom_msg.pose.pose.orientation.x = data.orientation.x
            self.odom_msg.pose.pose.orientation.y = data.orientation.y
            self.odom_msg.pose.pose.orientation.z = data.orientation.z
            self.odom_msg.pose.pose.orientation.w = data.orientation.w
    
        
        self.is_imu=True
    ```
    

### 정밀 도로 지도

---

| req 2-1 | MGeo 시각화 | Json 형식 MGeo 데이터 확인
Node, Link 데이터 → ROS Point Cloud 형식으로 변환
Rviz로 Node, Link 확인 |
| --- | --- | --- |

### 경로 계획

---

| req 3-1 | 주행 경로 기록 | ROS Odometry 메시지 통한 차량 데이터 수신 → txt파일로 저장 |
| --- | --- | --- |
| req 3-2  | 전역 경로 생성 | txt파일 확인 → ROS Path 메시지 형식으로 할당 → ROS Path 메시지 형식으로 송신 |
| req 3-3 | 지역 경로 생성 | 차량 상태 + 전역 경로 수신 → 가까운 Point 탐색 → Point 기준으로 지역 경로 생성 → 생성된 경로 ROS Path 형식으로 송신 |
| req 3-4 | Dijkstra 적용 | 시작/종료 Node 설정 → Link의 발생 비용 계산 → Dijkstra 적용 최단 거리 탐색 → 최단 거리 Path 생성 → ROS Path 형식으로 송신 |

### 판단 제어

---

| req 4-1 | Pure pursuit
횡 방향 제어 | 차량 상태 + 지역 경로 수신 → LFD 설정 → Point 확인 → 조향각 계산 → 제어 입력 값 ROS 형식으로 송신 |
| --- | --- | --- |
| req 4-2  | PID
종 방향 제어 | ‘’  → PID Gain값 설정 → 오차 비교 → PID 제어 수식 완성 → ‘’ |
| req 4-3 | 주행 속도 계획 | ‘’ → 주행 경로 곡률 및 최대 속도 계산 → ‘’ |
| req 4-4 | Advanced Pure pursuit | ‘’ → LFD min/max 설정 → LFD Gain 설정 → 속도 비례 LFD 계산 → ‘’ |
| req 4-5 | ACC | ‘’ → 경로 상 장애물 유무 파악 → 장애물 위치, 상대 거리, 상대 속도 기반 목표 속도 설정 → 제어 입력 결정 → ‘’ |