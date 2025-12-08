# ROS_DOMAIN_ID 같을 때 가능한 것들

## 🔗 4대 노트북 통신 시나리오

### 시나리오 1: 분산 작업
```
노트북 1 (메인)          노트북 2 (카메라)       노트북 3 (제어)        노트북 4 (모니터링)
────────────            ────────────            ────────────           ────────────
로봇 드라이버 런치  →   카메라 노드 실행   →   제어 명령 발행    →   모든 데이터 수신
                                                                        rviz 시각화
```

**노트북 1에서:**
```bash
ros2 launch dsr_bringup2 dsr_bringup2.launch.py mode:=real host:=192.168.137.100
# /dsr01/joint_states, /dsr01/state 등 토픽 발행
```

**노트북 2에서:**
```bash
ros2 launch realsense2_camera rs_launch.py
# /camera/color/image_raw, /camera/depth/image_rect_raw 토픽 발행
```

**노트북 3에서:**
```bash
ros2 topic pub /dsr01/movej dsr_msgs2/msg/MoveJ "..."
# 로봇 제어 명령 발행
```

**노트북 4에서:**
```bash
# 모든 토픽 확인 가능!
ros2 topic list
# 출력:
# /dsr01/joint_states      (노트북1에서)
# /dsr01/state             (노트북1에서)
# /camera/color/image_raw  (노트북2에서)
# ...

# 데이터 실시간 확인
ros2 topic echo /dsr01/joint_states
ros2 topic echo /camera/color/image_raw

# RViz에서 모두 시각화
rviz2
```

---

## ✅ 가능한 것들

### 1. 토픽 공유
**노트북 A:**
```bash
ros2 topic pub /hello std_msgs/msg/String "data: 'Hello from A'"
```

**노트북 B, C, D에서 모두 수신:**
```bash
ros2 topic echo /hello
# 출력: data: 'Hello from A'
```

### 2. 서비스 호출
**노트북 A에서 서비스 실행:**
```bash
ros2 run demo_nodes_cpp add_two_ints_server
```

**노트북 B에서 호출:**
```bash
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"
# 결과: sum: 8
```

### 3. 분산 제어
**노트북 1: 로봇 제어**
```python
# robot_controller.py
import rclpy
from dsr_msgs2.msg import MoveJ

node = rclpy.create_node('robot_controller')
pub = node.create_publisher(MoveJ, '/dsr01/movej', 10)
# 로봇 제어 명령 발행
```

**노트북 2: 비전 처리**
```python
# vision_node.py
import rclpy
from sensor_msgs.msg import Image

def image_callback(msg):
    # 이미지 처리
    detected_object = process_image(msg)
    # 노트북1의 로봇에게 위치 전송
    pub.publish(detected_object)
```

**노트북 3: 그리퍼 제어**
```python
# gripper_controller.py
def object_detected_callback(msg):
    # 노트북2에서 보낸 물체 감지 신호 받음
    gripper.close_gripper()
```

**노트북 4: 모니터링**
```bash
# 모든 상태 확인
ros2 topic echo /robot/state
ros2 topic echo /vision/detected_objects
ros2 topic echo /gripper/status
```

### 4. 실시간 협업
```
[시나리오: Pick and Place 작업]

노트북1 (로봇제어) → 물체 위로 이동
노트북2 (카메라)   → 물체 위치 실시간 피드백
노트북3 (그리퍼)   → 그리핑 타이밍 제어
노트북4 (안전감시) → 충돌 감지시 긴급정지 명령
```

---

## 🎯 실전 예제

### 예제 1: 4명이 협업하는 Pick and Place

**학생 A (노트북1):**
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py
```

**학생 B (노트북2):**
```bash
ros2 launch realsense2_camera rs_launch.py
python3 object_detection.py  # 물체 위치를 /object_pose 토픽으로 발행
```

**학생 C (노트북3):**
```python
# 물체 위치 받아서 로봇 이동
ros2 topic echo /object_pose
# 받은 좌표로 로봇 제어 명령 발행
ros2 topic pub /dsr01/movel ...
```

**학생 D (노트북4):**
```python
# 전체 시스템 모니터링
rqt_graph  # 노드 연결 상태 시각화
ros2 topic hz /camera/color/image_raw  # 카메라 fps 체크
ros2 topic bw /dsr01/joint_states  # 대역폭 체크
```

### 예제 2: 데이터 수집 분산

**노트북1: 로봇 동작**
```bash
python3 robot_motion.py  # 다양한 자세로 이동
```

**노트북2, 3: 카메라 녹화**
```bash
ros2 bag record /camera/color/image_raw /camera/depth/image_rect_raw
```

**노트북4: 로봇 상태 기록**
```bash
ros2 bag record /dsr01/joint_states /dsr01/state
```

→ **3대가 동시에 다른 데이터 녹화!**

---

## 🚫 다른 조와 격리된다

**1조 (DOMAIN_ID=60):**
```bash
ros2 topic pub /robot_cmd std_msgs/msg/String "data: '1조 명령'"
```

**2조 (DOMAIN_ID=61)에서:**
```bash
ros2 topic list
# /robot_cmd 안 보임! (격리됨)
```

---

## 💡 실용적인 팀 작업 분담

### 역할 분담 예시

| 노트북 | 역할 | 실행 내용 |
|--------|------|-----------|
| 1 | 로봇 드라이버 | 로봇 launch, 기본 제어 |
| 2 | 비전 시스템 | 카메라, 객체 인식 |
| 3 | 그리퍼 제어 | 그리핑 전략, 힘 제어 |
| 4 | 통합 제어 | 전체 시퀀스, 디버깅 |

**장점:**
- ⚡ 부하 분산 (각 노트북이 다른 작업)
- 🔄 동시 작업 (4명이 동시에 개발)
- 🐛 쉬운 디버깅 (한 노트북 문제시 다른 노트북으로 모니터링)
- 💾 데이터 분산 저장

---

## 🔍 통신 확인 방법

### 테스트 1: 간단한 메시지
**노트북 A:**
```bash
ros2 topic pub /test std_msgs/msg/String "data: 'Hello World'" -r 1
```

**노트북 B, C, D:**
```bash
ros2 topic echo /test
# Hello World가 계속 출력되면 성공!
```

### 테스트 2: 노드 발견
**노트북 A:**
```bash
ros2 run demo_nodes_cpp talker
```

**노트북 B:**
```bash
ros2 node list
# /talker 노드가 보이면 성공!

ros2 topic echo /chatter
# Hello World 메시지 수신되면 성공!
```

### 테스트 3: 대역폭 확인
```bash
ros2 topic bw /camera/color/image_raw
# 다른 노트북에서 발행하는 카메라 데이터 확인
```

---

## ⚠️ 주의사항

1. **같은 WiFi 필수**: 4대 모두 같은 네트워크(192.168.10.x)
2. **같은 DOMAIN_ID**: 모두 60번
3. **방화벽**: Ubuntu 방화벽이 ROS2 통신 차단할 수 있음
4. **대역폭**: 카메라 4대 동시 전송시 네트워크 부하 주의

---

## 🎓 학습 효과

- 분산 시스템 이해
- ROS2 통신 메커니즘 체험
- 팀 협업 능력 향상
- 실전 로봇 시스템 개발 경험
