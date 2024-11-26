# Today I learned

tf_pub.py

#TODO 1
Callback 함수 생성
def odom_callback(self,msg):
    self.is_odom = True

    # gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
    # Odometry 메세지 에 담긴 물체의 위치 와 자세 데이터를 아래 변수에 넣어준다.
    self.x = msg.pose.pose.position.x
    self.y = msg.pose.pose.position.y

    self.orientation_x = msg.pose.pose.orientation.x
    self.orientation_y = msg.pose.pose.orientation.y
    self.orientation_z = msg.pose.pose.orientation.z
    self.orientation_w = msg.pose.pose.orientation.w

gpsimu_parser.py 에서 연산한 맵 좌표계의 x,y,z 와 쿼터니언 좌표계로 변환산 x,y,z,w 를 가져와 넣어준다

#TODO 2
브로드캐스터 생성 및 Ego 상태 tf 브로드캐스팅

# TF 데이터를 broadcast 해주는 변수를 선언한다.
# TF 데이터에 물체의 좌표와 자세 데이터를 시간 그리고 Frame ID 를 넣어주면 된다.
# TF 예제는 map 좌표 를 기준으로 Ego 차량의 위치를 좌표를 나타낸다
br = tf.TransformBroadcaster()
br.sendTransform((self.x, self.y, 0),
                (self.orientation_x, self.orientation_y, self.orientation_z, self.orientation_w),
                rospy.Time.now(),
                "Ego",
                "map")

---

mgeo_pub.py

#TODO 2
Link 정보 Point Cloud 데이터로 변환

# Point Cloud 형식으로 Link 의 좌표 정보를 변환합니다.
# Link 의 개수 만큼 반복하는 반복 문을 이용해 Link 정보를 Point Cloud 형식 데이터에 넣습니다.

for link_idx in self.links.values():
    for i in link_idx.points:
        tmp = Point32()
        tmp.x = i[0]
        tmp.y = i[1]
        tmp.z = i[2]
        all_link.points.append(tmp)

return all_link

#TODO 3
Node 정보 Point Cloud 데이터로 변환

# Point Cloud 형식으로 Node 의 좌표 정보를 변환합니다.
# Node 의 개수 만큼 반복하는 반복 문을 이용해 Node 정보를 Point Cloud 형식 데이터에 넣습니다.

for node_idx in self.nodes.values():
    tmp = Point32()
    tmp.x = node_idx.point[0]
    tmp.y = node_idx.point[1]
    tmp.z = node_idx.point[2]
    all_node.points.append(tmp)

return all_node

#TODO 4
변환한 Link, Node 정보 Publish

# 변환한 Link, Node 정보 를 전송하는 publisher 를 만든다.
self.link_pub.publish(self.link_msg)
self.node_pub.publish(self.node_msg)

---

path_maker.py

#TODO 1 저장할 경로 및 텍스트파일 이름을 정하고, 쓰기 모드로 열기

# Path 데이터를 기록 하고 저장 할 경로와 txt 파일의 이름을 정한다.
# 이후 쓰기 모드로 연다.
# pkg_name 과 path_name 은 22 번 줄 참고한다.
rospack = rospkg.RosPack()
pkg_path = rospack.get_path(pkg_name)
full_path = os.path.join(pkg_path, path_name)
self.f = open(full_path, 'w')

#TODO 2 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

# gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
# Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
self.x = msg.pose.pose.position.x
self.y = msg.pose.pose.position.y

#TODO 3 콜백함수에서 이전 위치와 현재 위치의 거리 계산

# 현재 차량의 위치와 이전에 지나온 위치의 좌표 데이터를 구한다.
# 구해진 좌표 사이의 거리를 계산한다.
# 이전 위치 좌표는 아래 #TODO: (4)에서 정의 한다.
distance = sqrt((x - self.prev_x)**2 + (y - self.prev_y)**2)

#TODO 4 이전 위치보다 0.5m 이상일 때 위치를 저장
if distance >0.5:

    # distance 가 0.5 보다 커야지만 동작한다.
    # 현재 위치 좌표를 data 에 담은 뒤 txt 파일로 작성한다.
    # data 는 문자열 이며 x y z 사이는 \t 로 구분한다
    data ='{0}\t{1}\t{2}\n'.format(x,y,z)
    self.f.write(data)
    self.prev_x = x
    self.prev_y = y
    self.prev_z = z

    print("Recorded position: x={}, y={}, z={}".format(x, y, z))
