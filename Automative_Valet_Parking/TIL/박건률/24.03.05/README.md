# Today I learned

Pure pursuit
경로 위의 한 점을 원 호를 그리며 따라가는 방법입니다.

전방주시거리 하나로 -> 조향각을 계산가능

조향각 = 경로, 차량의 위치, 축거, ld로 구하기 가능

---

PID

원하는 값에 도달하기 위한 자동 피드백 제어 방법 - 구현 난이도 대비 성능 우수

우리가 얻을수 있는 파라미터 - 목표속도, 현재속도, 목표속도-현재속도(오차)

오차값으로 P(비례), I(적분), D(미분)을 통해서 현재 값을 목표값에 수렴 시키는것

이득값을 적절히 튜닝 시키는게 중요하다.

비례항: 현재 상태에서의 오차 값의 크기에 비례한 제어작용을 한다.
적분항: 정상상태(steady-state) 오차를 없애는 작용을 한다.
미분항: 출력 값의 급격한 변화에 제동을 걸어 오버슛(overshoot)을 줄이고 안정성을 향상시킨다.

PID의 성능을 측정하는 4가지 지표
Rise time: 목표값의 10% -> 90% 까지 걸리는 시간
Overshoot: 현재값이 목표값보다 커졌을때의 값
Setting time: 목표 값의 5% 이내에 들어갈 때의 시간
Steady-State Error: 정상상태에 도달하고 나서 존재하는 에러

---

경로기반 속도 계획 = 경로점에서 얼마만큼 속도를 내야 하는가?(원심력이 존재하기 때문)
속도 클수록, 회전 반경 작을수록 -> 전복 확률 상승

따라서 경로기반 속도 계획으로 전복을 방지해야함

곡률 반지름 -> 최소자승법을 통해서 구할 수 있음

따라서
차량 x좌표 = a
차량 y좌표 = b
a^2 + b^2 - r^2 = c

a,b,c를 알면 곡률 반지름을 알 수 있다.

---

ACC = 센서로 앞차를 인식하고 거리를 유지하는 시스템

Safe distance : 내가 설정한 안전거리(현재속도 + time.gap(판단에서 제어까지 차량을 안전하게 멈추는 시간 보통 2초) + defaultSpace(기본 유지 거리))
Relative distance : 상대방과 실제 거리

안전거리 > 실제거리 = Speed 모드
안전거리 < 실제거리 = Spacing 모드

적절한 이득값 조절로 앞차의 급격한 속도 변화에도 안전거리를 유지하게 하는게 목표
(장애물에도 적용가능 속도가 0인 차량이라고 생각)

---
gps_parser.py

#TODO 0
pip install pyproj

#TODO 1
위도, 경도 파라미터를 이용해 -> UTM 좌표로 변환(WGS84 -> UTM)

Proj(proj='utm',zone=52,ellps='WGS84', preserve_units=False) # 한반도는 zone 52 # 기본이 M:

#TODO 2
시뮬레이터에서 GPS 데이터를 받아오는 Callback 함수 생성

#TODO 3
위도 경도 데이터를 UTM 좌표로 변환
xy_zone = self.proj_UTM(self.lon, self.lan) # TODO 1에서 만든 객체를 이용해 변환

#TODO 4
print(' lat : ', self.lat)
print(' lon : ', self.lon)
print(' utm X : ', utm_msg.data[0])
print(' utm Y : ', utm_msg.data[1])

---
gpsimu_parser.py

#TODO 1
변환 하고자 하는 좌표계를 선언
# GPS 센서에서 수신되는 위도, 경도 데이터를 UTM 좌표료 변환 하기 위한 예제이다.
# 해당 예제는 WGS84 좌표계에서 UTM 좌표계로의 변환을 진행한다.
# 시뮬레이터 K-City Map 의 경우 UTM 좌표계를 사용하며 실제 지도 상 위치는 UTM 좌표계의 52 Zone 에 존제한다.
# 맵 좌표계는 m 단위를 사용한다.
# 아래 주소의 링크를 클릭하여 Ptoj 의 사용 방법을 확인한다.
# https://pyproj4.github.io/pyproj/stable/api/proj.html
# " proj= , zone= , ellps =  , preserve_units = "
self.proj_UTM = Proj(proj='utm',zone=52,ellps='WGS84', preserve_units=False)

#TODO 2
송신 될 Odometry 메세지 변수 생성

# ROS 메세지 중 물체의 위치와 자세 데이터를 나타내는 Odometry 메세지를 사용한다.
# 차량의 현재 위치와 자세 데이터를 GPS IMU 센서에 담아서 Publsih 한다.
# 이때 frame_id 는 '/odom' child_frame_id 는 '/base_link' 로 한다.

self.odom_msg = Odometry()
self.odom_msg.header.frame_id = '/odom'
self.odom_msg.child_frame_id = '/base_link'

#TODO 3 위도 경도 데이터 UTM 죄표로 변환
def convertLL2UTM(self):

    # pyproj 라이브러리를 이용해 정의한 좌표 변환 변수를 이용하여 위 경도 데이터를 변환한다.
    # 변환 시 이전 gps_parser.py 예제와 달리 시뮬레이터 GPS 센서의 offset 값을 적용 한다.
    # GPS 센서에서 출력되는 Offset 값은 시뮬레이터에 맵 좌표계로 변경을 위한 값이다.
    # UTM 좌표로 변환 된 x, y 값에 offset 값을 빼주면 된다.
    xy_zone = self.proj_UTM(self.lon, self.lat)

    # if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
    if self.lon == 0 and self.lat == 0:
        self.x = 0.0
        self.y = 0.0
    else:
        self.x = xy_zone[0] - self.e_o
        self.y = xy_zone[1] - self.n_o

    #TODO: (4) Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기

    # Offset 을 적용하여 시뮬레이터 맵 좌표계 값으로 변환 된 좌표 데이터를 Odometry 메세지에 넣는다.
    self.odom_msg.header.stamp = rospy.get_rostime()
    self.odom_msg.pose.pose.position.x = self.x
    self.odom_msg.pose.pose.position.y = self.y
    self.odom_msg.pose.pose.position.z = 0.0

#TODO 4 Odometry 메세지 변수에 차량의 위치 및 상태 데이터 담기

# IMU 를 통해 받은 물체의 자세 데이터를 Odometry 메세지에 넣는다.
# if 문을 이용 예외처리를 하는 이유는 시뮬레이터 음영 구간 설정 센서 데이터가 0.0 으로 나오기 때문이다.
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

#TODO 5 Odometry 메세지 Publish

# Odometry 메세지 를 전송하는 publisher 를 만든다.
self.odom_pub.publish(self.odom_msg)

GPS 데이터
GPS 데이터 구조는 아래와 같다.

1: Latitude (단위: deg) 위도 - North
2: Longitude (단위: deg) 경도 - East
3: Altitude (단위: m)
4: EastOffset (단위 : m), * UTM 맵 원점을기준으로, 동쪽 방향의 Offset 값 (Map 별로 고정된 상수값)
5: NorthOffset (단위 : m), * UTM 맵 원점을기준으로, 북쪽 방향의 Offset 값 (Map 별로 고정된 상수값)

---

tf_pub.py

