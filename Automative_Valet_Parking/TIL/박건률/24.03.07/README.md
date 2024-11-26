# Today I learned

global_path_pub.py

def __init__(self, pkg_name = 'ssafy_2', path_name = 'kcity.txt'):
    rospy.init_node('global_path_pub', anonymous = True)

    #TODO 1 Global Path publisher 선언 및 Global Path 변수 생성

    # Global Path 데이터를 Publish 하는 변수와 메세지를 담고있는 변수를 선언한다.
    # 이때 Global Path 는 map 좌표계를 기준으로 생성한다.
    self.global_path_pub = rospy.Publisher('/global_path',Path, queue_size=1)
    self.global_path_msg = Path()
    self.global_path_msg.header.frame_id = '/map'

    #TODO 2 읽어올 경로 의 텍스트파일 이름을 정하고, 읽기 모드로 열기

    # Path 데이터가 기록 된 txt 파일의 경로와 이름을 정한다.
    # 이후 읽기 모드로 연다.
    # pkg_name 과 path_name 은 21 번 줄 참고한다.
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path(pkg_name)
    full_path = os.path.join(pkg_path, path_name)
    self.f = open(full_path, 'r')
    lines = self.f.readlines()

    #TODO 3 읽어 온 경로 데이터를 Global Path 변수에 넣기

    # 읽어온 x y z 좌표 데이터를 self.global_path_msg 변수에 넣는다.
    # 넣어준 반복 문을 이용하여 작성한다.
    for line in lines :
        tmp = line.split()
        read_pose = PoseStamped()
        read_pose.pose.position.x = float(tmp[0])
        read_pose.pose.position.y = float(tmp[1])
        read_pose.pose.orientation.w = 1
        self.global_path_msg.poses.append(read_pose) # read_pose = PoseStamped
    self.f.close()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #TODO 4 Global Path 정보 Publish

        # Global Path 메세지 를 전송하는 publisher 를 만든다.
        self.global_path_pub.publish(self.global_path_msg)

        rate.sleep()

local_path_pub.py

#TODO 1 Global Path 와 Odometry 데이터 subscriber 생성

# Global Path 와 Odometry 데이터 subscriber 를 생성한다.
# 콜백 함수의 이름은 self.global_path_callback, self.odom_callback 로 한다.
rospy.Subscriber("/global_path", Path, self.global_path_callback)
rospy.Subscriber("/odom", Odometry, self.odom_callback)

#TODO 2 Local Path publisher 선언

# local Path 데이터를 Publish 하는 변수를 선언한다.
self.local_path_pub = rospy.Publisher('/local_path',Path, queue_size=1)

# 초기화
self.is_odom = False
self.is_path = False

#TODO 3 Local Path 의 Size 결정

# Local Path 의 크기를 지정한다.
# 차량이 주행 시 Local Path 의 크기 만큼의 정보를 가지고 주행하게 된다
# 너무 작지도 크기지도 않은 값을 사용한다 (50 ~ 200)
self.local_path_size = 50

#TODO 4 콜백함수에서 처음 메시지가 들어오면 초기 위치를 저장

# gpsimu_parser.py 예제에서 Publish 해주는 Odometry 메세지 데이터를 Subscrib 한다.
# Odometry 메세지 에 담긴 물체의 위치 데이터를 아래 변수에 넣어준다.
self.x = msg.pose.pose.position.x
self.y = msg.pose.pose.position.y

#TODO 5 Global Path 에서 차량 위치와 가장 가까운 포인트(current Waypoint) 탐색

# global Path 에서 차량의 현재 위치를 찾습니다.
# 현제 위치는 WayPoint 로 기록하며 현재 차량이 Path 에서 몇번 째 위치에 있는지 나타내는 값이 됩니다.
# 차량의 현재 위치는 Local Path 를 만드는 시작 위치가 됩니다.
# 차량의 현재 위치를 탐색하는 반복문은 작성해 current_waypoint 찾습니다.
min_dis = float('inf')
current_waypoint = -1
for idx, point in enumerate(self.global_path_msg.poses):
    global_x = point.pose.position.x
    global_y = point.pose.position.y
    distance = ((x - global_x)**2 + (y - global_y)**2)**0.5

    if distance < min_dis:
        min_dis = distance
        current_waypoint = idx


#TODO 6 가장 가까운 포인트(current Waypoint) 위치부터 Local Path 생성 및 예외 처리

# 차량의 현재 위치 부터 local_path_size 로 지정한 Path 의 크기 만큼의 Path local_path 를 생성합니다.
# 차량에 남은 Path 의 길이가 local_path_size 보다 작은 경우가 있음으로 조건 문을 이용하여 해당 조건을 예외 처리 합니다.
if current_waypoint != -1 :
    if current_waypoint + self.local_path_size < len(self.global_path_msg.poses):
        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:current_waypoint + self.local_path_size]
    else :
        local_path_msg.poses = self.global_path_msg.poses[current_waypoint:]

print(x,y, len(local_path_msg.poses))

#TODO 7 Local Path 메세지 Publish

# Local Path 메세지 를 전송하는 publisher 를 만든다.
self.local_path_pub.publish(local_path_msg)