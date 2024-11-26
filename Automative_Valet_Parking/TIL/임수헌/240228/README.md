# 다익스트라 알고리즘 문제 풀이

## Path Planning
- 목표 지점까지 이동하는 최적의 경로를 계획하는 과정
- 이 과정에서 다익스트라 알고리즘을 활용할 수 있음

## 다익스트라 알고리즘
- 우선순위 큐와 BFS를 활용해서 최단 경로를 계산하는 알고리즘

### 풀이법
1. 인접 리스트를 생성한다. (행렬이 아니라 인덱스로 접근할 수 있도록 하는 것)
2. 누적합을 위한 배열 (인덱스 크기만큼)을 생성하고 1e9로 초기화한다. **d**
3. 우선 순위 큐를 위해 heap을 import 하고 출발지 정보(누적합 : **d[출발지] = 0**, 출발지)를 힙에 넣는다.

    **→ 누적합을 기준으로 우선순위를 생성해야 하기 때문에 그 위치에서의 누적합을 튜플 앞 원소에 넣는다.**

4. 그리고  **d[출발지]** 0으로 초기화한다. (시작임)
5. (현재까지의 누적합, 현재 위치)를 힙에서 꺼낸다.
6. d[현위치] < 현재까지의 누적합 ⇒ 이미 방문한 지점 + 누적 거리가 더 짧게 방문한 적이 있다면 pass
    
    ```python
    if distance[now] < dist:
                continue
    ```
    

    **→ d[현위치] 값이 INF가 아니라면 다른 경로를 통해 이미 방문했다는 말**

    **→ d[현위치] 값(다른 경로)이 현재경로에서의 현 위치 누적합(dist)보다 작다는 말은 다른 경로가 최단 경로라는 뜻**

### ⇒ continue

7. 인접 리스트를 순회하면서 인접 노드 확인
8. 다음 노드로 가기 위한 누적 거리를 계산
    
    ```python
    next_node = next[0]
    cost = next[1]
    # next_node 로 가기 위한 누적 거리
     new_cost = dist + cost
    ```
    
8. next_node의 누적합을 갱신하기 전 그 노드의 방문 여부와 갈 필요가 있는지 없는지 체크
    
    ```python
    # 누적 거리가 기존보다 크네?
    if distance[next_node] <= new_cost:
                continue
    ```
    

    → 갱신하려는 값이 같거나 크면 이미 그 노드에서는 최소이기 때문에 힙에 넣어서 탐색할 필요가 없다.


### ⇒ continue


10. d[다음노드]를 계산한 누적거리로 갱신해주고 힙에 (누적거리, 다음노드)를 넣어준다.
  
### 문제

https://www.acmicpc.net/problem/4485
```python
'''
다익스트라
'''
import heapq


def dijk(si,sj):
    # 우선순위 큐
    pq = []
    # 출발점 누적 합
    mm[si][sj] = arr[si][sj]
    heapq.heappush(pq, (mm[si][sj],si,sj))

    while pq:
        s, i, j = heapq.heappop(pq)

        if mm[i][j] < s:
            continue

        for di, dj in [[0,1],[1,0],[0,-1],[-1,0]]:
            ni = i+di
            nj = j+dj
            # 범위
            if ni < 0 or N <= ni or nj < 0 or N <= nj:
                continue

            new_cost = s + arr[ni][nj]

            if mm[ni][nj] <= new_cost:
                continue
            mm[ni][nj] = new_cost
            heapq.heappush(pq, (new_cost,ni,nj))


# 테스트 케이스 끝이 0이다
T = 1
while True:
    N = int(input())
    # 테스트 케이스 끝
    if N == 0:
        break
    arr = [list(map(int ,input().split())) for _ in range(N)]

    # 다익스트라 초기 설정 -> 최소 누적합 배열
    INF = int(1e9)
    mm = [[INF] * N for _ in range(N)]

    # 0,0 출발
    dijk(0,0)

    print(f'Problem {T}: {mm[N-1][N-1]}')
    T += 1
```