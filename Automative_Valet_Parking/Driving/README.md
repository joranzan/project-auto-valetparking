## 전진 주차

#### 장애물 시나리오
![장애물 위치](./images/장애물%20위치.PNG)

<br>

#### 하차지점 (ego차량 주행 시작 지점)
![하차지점](./images/하차지점_위치.PNG)


<br>

#### launch 파일

```
roslaunch rrt_forward_3 parking_lot_v_rrt_forward.launch
```


#### 사용 경로

1. 하차지점 ---- 로타리 끝점 : dikjstra 경로
2. 로타리 끝점 ---- 주차장 진입 : parkinglot_entrance_tiny.txt
3. 주차장 내부 뺑뺑이 경로 : parkinglot_driving_test_tiny.txt
4. RRT로 찾은 경로 저장되는 파일 : ssafy_test_path_star.txt

<br>

#### Node Graph

```mermaid
graph LR;
    t___local_path((/local_path)) -->|publishes| n___lattice_planner
    t___local_path -->|publishes| n___advanced_purepursuit_forward
    t___local_path -->|publishes| n___ctrl_cmd
    t___lattice_path((/lattice_path)) -->|publishes| n___advanced_purepursuit_v_rrt_fw
    t___global_path((/global_path)) -->|publishes| n___assignment_star
    t___global_path -->|publishes| n___local_path_pub
    t___global_path -->|publishes| n___advanced_purepursuit_forward
    t___global_path -->|publishes| n___advanced_purepursuit_v_rrt_fw
    t___odom((/odom)) -->|broadcasts| n___tf
    t___odom -->|publishes| n___assignment_star
    t___odom -->|publishes| n___local_path_pub
    t___odom -->|publishes| n___map_change_controller
    t___odom -->|publishes| n___advanced_purepursuit_forward
    t___odom -->|publishes| n___advanced_purepursuit_v_rrt_fw
    t___global_change_cmd((/global_change_cmd)) -->|publishes| n___global_path_pub_v_rrt
    t___global_change_cmd -->|publishes| n___advanced_purepursuit_v_rrt_fw
    t___done((/done)) -->|publishes| n___global_path_pub_v_rrt
    t___Object_topic((/Object_topic)) -->|publishes| n___assignment_star
    t___Object_topic -->|publishes| n___lattice_planner
    t___imu((/imu)) -->|publishes| n___gpsimu_parser
    t___mode_cmd((/mode_cmd)) -->|publishes| n___srv_event_cmd
    t___start_rrt((/start_rrt)) -->|publishes| n___assignment_star
    t___start_rrt -->|publishes| n___global_path_pub_v_rrt
    t___ready_to_park((/ready_to_park)) -->|publishes| n___local_path_pub
    t___ready_to_park -->|publishes| n___srv_event_cmd
    t___ready_to_park -->|publishes| n___advanced_purepursuit_forward
    t___visualization_marker((/visualization_marker)) -->|publishes| n___assignment_star
    t___visualization_marker -->|publishes| n___global_path_pub_v_rrt
    t___Ego_topic((/Ego_topic)) -->|publishes| n___assignment_star
    t___Ego_topic -->|publishes| n___lattice_planner
    t___Ego_topic -->|publishes| n___advanced_purepursuit_forward
    t___Ego_topic -->|publishes| n___advanced_purepursuit_v_rrt_fw
    t___gps((/gps)) -->|publishes| n___gpsimu_parser
    t___rosbridge_websocket((/rosbridge_websocket)) -->|publishes| n___Object_topic
    t___rosbridge_websocket -->|publishes| n___imu
    t___rosbridge_websocket -->|publishes| n___Ego_topic
    t___rosbridge_websocket -->|publishes| n___gps
    t___gpsimu_parser -->|publishes| n___odom
    t___advanced_purepursuit_forward -->|publishes| n___ctrl_cmd
    t___advanced_purepursuit_forward -->|publishes| n___mode_cmd
    t___rosapi((/rosapi)) -->|publishes| n___assignment_star
    t___rosapi -->|publishes| n___lattice_planner
    t___mgeo_pub((/mgeo_pub)) -->|publishes| n___link
    t___mgeo_pub -->|publishes| n___node
    t___lattice_planner -->|subscribes| t___lattice_path
    t___advanced_purepursuit_v_rrt_fw -->|publishes| t___ctrl_cmd
```

