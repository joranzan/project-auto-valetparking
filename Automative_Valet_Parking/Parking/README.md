# RRT* 를 활용한 후진 주차 시나리오

## 시나리오 파일 저장 위치
MoraiLauncher_Win/MoraiLauncher_Win_Data/SaveFile/Scenario

## 실행 방법
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
roslaunch avp parking_rear.launch
```

## RRT *
- 최단거리를 보장하지 않는 RRT의 단점을 보완하기 위해 만들어진 알고리즘
- 랜덤 포인트와 이전 노드와의 거리만을 계산하는 RRT와 달리, RRT*는 주변 노드들의 원점에서부터의 거리를 통해 가장 가까운 노드를 결정

## 차량 제어
- 기존 Pure-Pursuit 알고리즘을 Calibration을 통해 후진 주차에 최적화
