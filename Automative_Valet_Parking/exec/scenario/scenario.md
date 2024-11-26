# 시연 시나리오

작성자: 태환 김
최종 편집 일시: 2024년 4월 4일 오전 8:43

### 시연 순서

---

1. 도로 주행
2. 전진 주차(RRT)
3. 후진 주차(GR)
4. 출차 및 장애물 회피
5. 후진 주차(RRT)

### 환경 세팅 순서

---

- 사전 준비
    - ROS 및 MORAI 시뮬레이터 설치 (WSL2, Ubuntu 18.04)
    - WorkSpace 파일 설치 및 패키지 빌드 (catkin_make)
    - script 파일 실행 권한 부여
        - 파일 위치 : WorkSpace/src/패키지명/script/
        
        ```bash
        # 파일 위치에서
        $ chmod +x [파일명]
        ```
        

1. MORAI SIM 실행
    - Map : R_KR_PG_K-City
    - Vehicle : 2020_Kia_Niro(EV) (권장)

1. Network Settings (F4)
    - Ego Network
        - 통신 방식 : ROS
        - 모든 Msg Type, Topic 기본 설정 값 사용 (Reset 버튼 클릭 시 초기화)
        - TF2Publisher 항목 비활성화
        - 그 외 Pub, Sub, Ser 항목 모두 활성화
        - 설정 후 Connect 클릭하여 설정 저장
    - Simulator Network
        - 통신 방식 : ROS
        - 모든 Msg, Topic 기본 설정 값 사용 (Reset 버튼 클릭 시 초기화)
        - 모든 항목 활성화
        - 설정 후 Connect 클릭하여 설정 저장
    
    <aside>
    💡 세팅 완료 후 Connect 상태에서 Save 기능으로 설정 저장 가능
    세팅 파일 저장 후 Ego 및 Simulator 연결 해제 후 해제 상태 유지
    
    </aside>
    
2. Sensor Edit (F3)
    - 필요 센서
        - GPS, IMU
        - 통신 방식 : ROS
        - 설정 값 : 기본 설정
        - 부착 위치
            
            X : 1.60
            
            Y : 0.00
            
            Z : 1.15
            
    
    <aside>
    💡 세팅 완료 후 Connect 상태에서 Save 기능으로 설정 저장 가능
    세팅 파일 저장 후 모든 센서 연결 해제 후 해제 상태 유지
    
    </aside>
    
3. WSL 터미널 실행 (MobaXterm / Vscode 등)
    - 터미널1 - rosbridge 실행
        
        ```bash
        roslaunch rosbridge-server rosbridge-websocket.launch
        ```
        
    
4. 네트워크 및 센서 세팅 설정
    - Network Settings (F4) → 저장한 세팅 파일 Load 기능으로 불러오기
    - Sensor Edit (F3) → 저장한 세팅 파일 Load 기능으로 불러오기
    - rosbridge 터미널에서 connection 확인 (상기 세팅 기준 31개 Client 연결)
    
5. 시연 시나리오 자동 스크립트 파일 실행
    - 터미널2 - 자동 스크립트 launch파일 실행
        - integration_test 패키지의 testInit.launch
        
        ```bash
        $ roslaunch integration_test testInit.launch
        ```
        
    
    - 시나리오 시작 마다 터미널에 ‘1’ 입력 시 시나리오 진행 시작

### 시나리오 실행 자동 스크립트

---

- 패키지 : integration_test
- 실행 파일 : testInit.launch
    
    ```html
    <launch>
        <node pkg="rrt_drive1" type="mgeo_pub.py" name="mgeo_pub"  />
        <node pkg="rrt_drive1" type="gpsimu_parser.py" name="gpsimu_parser" />
        <node pkg="rrt_drive1" type="tf_pub.py" name="tf"  />
        <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rrt_drive2)/rviz/lane_detection_rviz.rviz" />
        <node pkg="integration_test" type="testInit" name="testinit" output="screen" />
    </launch>
    ```
    
    - 공통 script 파일 실행 후 스크립트 파일 실행
    
    - 스크립트 파일
        - 코드
            
            ```cpp
            #include <unistd.h>
            #include <iostream>
            #include <sys/wait.h>
            #include <errno.h>
            #include <cstdio>
            #include "ros/ros.h"
            #include "std_msgs/Int8.h"
            #include <mutex>
            #include <cstring>
            #include <vector>
            #include <thread>
            #define MAX_CMD_NUM 6
            #define MAX_ARG_NUM 10
            #define MAX_PATH_LENGTH 50
            
            using namespace std;
            
            pid_t pidList[MAX_CMD_NUM];
            
            vector<string> cmdList[MAX_CMD_NUM] = {
            	{"",""},
            	{"/opt/ros/melodic/bin/rosrun", "rosrun", "ssafy_3", "load_named_scenario.py", "scenario1.json", "1"},
            	{"/opt/ros/melodic/bin/roslaunch","roslaunch","rrt_drive2", "parking_lot_v_rrt_forward.launch"},
            	{"/opt/ros/melodic/bin/rosrun", "rosrun", "ssafy_3", "load_named_scenario.py", "scenario3.json", "3"},
            	{"/opt/ros/melodic/bin/roslaunch","roslaunch","ssafy_3", "parking_lot_se3.launch"},
            	{"/opt/ros/melodic/bin/roslaunch","roslaunch","ssafy_3", "parking_lot_exit.launch"}
            };
            
            int currentCmdIdx = 1;
            mutex idxMutex;
            
            void endMsgCallback(const std_msgs::Int8::ConstPtr& msg){
            	cout<<"in callback\n";
            	ROS_INFO("Receive data = %d", msg->data);
            	idxMutex.lock();
            	if((int) msg->data == currentCmdIdx){
            		//kill process
            		char killCmd[100];
            		sprintf(killCmd, "kill -2 %d", pidList[currentCmdIdx]);
            		system(killCmd);
            		currentCmdIdx++;
            		ROS_INFO("%d\n",currentCmdIdx);
            		idxMutex.unlock();
            	}
            	else{
            		idxMutex.unlock();
            	}
            }
            
            void rosThread(int thread_idx){
            	ros::NodeHandle nh;
            	ros::Subscriber sub = nh.subscribe("end_topic", 100, endMsgCallback);
            	ros::Rate loop_rate(10);
            
            	while(true){
            		loop_rate.sleep();
            		ros::spinOnce();
            
            		idxMutex.lock();
            		if(thread_idx != currentCmdIdx){
            			idxMutex.unlock();
            			break;
            		}
            		//cout<<"currentCmdIdx: "<<currentCmdIdx<<'\n';
            		idxMutex.unlock();
            	}
            
            	sub.shutdown();
            }
            
            int main(int argc, char **argv){
            
            	ros::init(argc, argv, "testinit");
            
            	while(currentCmdIdx < MAX_CMD_NUM){
            		pid_t c_pid;
            		int status;
            
            		c_pid = fork();
            
            		// parent precess
            		if(c_pid > 0){
            
            			pid_t wait_pid;
            
            			cout<<"\n\nparent with child"<<c_pid<<"\n\n";
            
            			thread rt = thread(rosThread, currentCmdIdx);
            
            			while(((wait_pid = wait(&status)) == -1) && errno == EINTR);
            
            			rt.join();
            
            			if(wait_pid == -1){
            				cout<<"child process killed\n";
            			}
            			else{
            				if(WIFEXITED(status)){
            					cout<<"successful exit\n";
            				}
            				else if(WIFSIGNALED(status)){
            					cout<<"wrong exit\n";
            				}
            			}
            		}
            		else if (c_pid == 0){
            			const char *cmdArgs[MAX_ARG_NUM];
            
            			for(int i = 0; i < cmdList[currentCmdIdx].size(); i++){
            				cmdArgs[i] = cmdList[currentCmdIdx][i + 1].c_str();
            			}
            
            			cmdArgs[cmdList[currentCmdIdx].size()] = NULL;
            
            			execv(cmdList[currentCmdIdx][0].c_str(), (char **)cmdArgs);
            		}
            		else{
            			perror("failed to fork....\n");
            			return 1;
            		}
            	}
            
            	return 0;
            }
            
            ```
            
        - load_named_scenario.py
            - 저장된 시나리오 불러오기
        - parking_lot_OO.launch
            - 시나리오 진행을 위한 launch파일

### 시나리오1

---

- 도로 주행 + RRT 전진 주차
- 실행 파일
    - Scenario : scenario1.json (load_named_scenario.py, 1)
    - roslaunch : rrt_drive2 parking_lot_v_rrt_forward.launch
        
        ```html
        <launch>
            <!-- <node pkg="rrt_drive1" type="mgeo_pub.py" name="mgeo_pub"  /> -->
            <!-- <node pkg="rrt_drive1" type="gpsimu_parser.py" name="gpsimu_parser" /> -->
            <node pkg="rrt_drive2" type="assignment_star.py" name="assignment_star" />
            <node pkg="rrt_drive1" type="global_path_pub_v_rrt.py" name="global_path_pub_v_rrt"  />
            <node pkg="rrt_drive1" type="mgeo_dijkstra_path_parking.py" name="mgeo_dijkstra_path"  />
            <node pkg="rrt_drive2" type="map_change_controller.py" name="map_change_controller"  />
            <!-- <node pkg="rrt_drive2" type="lattice_planner.py" name="lattice_planner" output="screen"/> -->
            <node pkg="rrt_drive1" type="local_path_pub.py" name="local_path_pub" />
            <node pkg="rrt_drive2" type="advanced_purepursuit_v_rrt_fw.py" name="advanced_purepursuit_v_rrt_fw" output="screen"/>
            <node pkg="rrt_drive2" type="srv_event_cmd.py" name="srv_event_cmd" />
            <node pkg="rrt_drive2" type="advanced_purepursuit_forward.py" name="advanced_purepursuit_forward"  />
            <!-- <node pkg="rrt_drive1" type="tf_pub.py" name="tf"  /> -->
        
            <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find rrt_drive1)/rviz/kcity_rviz.rviz" /> -->
        </launch>
        ```
        
    - 주행 + 주차 종료 후
        - parking_lot_v_rrt_forward.launch 종료
        - scenario3.json 불러오기
        - parking_lot_se3.launch 실행 후 시작 입력 대기

### ~~시나리오2~~

---

- RRT 후진 주차
- 기술상 문제로 인한 시나리오 제외

- 별도 실행 방법
    - 실행 파일
        - Scenario : scenario2.json
        - roslaunch
            
            ```bash
            # rosbridge 및 Network 세팅 후
            # 터미널2
            $ roslaunch rrt_drive2 common.launch
            
            # 터미널3
            $ roslaunch avp parking_rear.launch
            ```
            

### 시나리오3

---

- 주차장 내 주행 + 후진 주차
- 실행 파일
    - Scenario : scenario3.json (load_named_scenario2.py, 3)
    - roslaunch : ssafy_3 parking_lot_se3.launch
        
        ```html
        <launch>
            <!-- <node pkg="ssafy_2" type="mgeo_pub.py" name="mgeo_pub"  /> -->
            <!-- <node pkg="ssafy_2" type="gpsimu_parser.py" name="gpsimu_parser" /> -->
            <node pkg="ssafy_2" type="global_path_pub_se2.py" name="global_path_pub"  />
            <node pkg="ssafy_3" type="lattice_planner.py" name="lattice_planner" output="screen"/>
            <node pkg="ssafy_3" type="advanced_purepursuit_se3.py" name="advanced_purepursuit_se3"/>
            <node pkg="ssafy_3" type="auto_parking.py" name="auto_parking"/>
            <node pkg="ssafy_2" type="local_path_pub.py" name="local_path_pub" />
            <!-- <node pkg="ssafy_2" type="tf_pub.py" name="tf"  /> -->
        
            <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ssafy_2)/rviz/kcity_rviz.rviz" /> -->
        </launch>
        ```
        
        - 주차 종료 후
            - parking_lot_se3.launch 종료
            - parking_lot_exit.launch 실행 후 시작 입력 대기

- 출차
- 실행 파일
    - roslaunch : parking_lot_exit.launch
        
        ```html
        <launch>
            <!-- <node pkg="ssafy_2" type="mgeo_pub.py" name="mgeo_pub"  /> -->
            <!-- <node pkg="ssafy_2" type="gpsimu_parser.py" name="gpsimu_parser" /> -->
            <node pkg="ssafy_2" type="global_path_pub_exit.py" name="global_path_pub"  />
            <node pkg="ssafy_3" type="lattice_planner.py" name="lattice_planner" output="screen"/>
            <node pkg="ssafy_3" type="advanced_purepursuit_se3_exit.py" name="advanced_purepursuit_se3_exit" />
            <node pkg="ssafy_2" type="local_path_pub.py" name="local_path_pub" />
            <!-- <node pkg="ssafy_2" type="tf_pub.py" name="tf"  /> -->
        
            <!-- <node pkg="rviz" type="rviz" name="rviz" args="-d $(find ssafy_2)/rviz/kcity_rviz.rviz" /> -->
        </launch>
        ```