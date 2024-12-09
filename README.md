# Follower-robot
전북대학교 2024년 2학기 산학실전캡스톤에서 진행한 프로젝트

# 주제 : 모바일 로봇을 이용한 사람 추종 기술 개발
ROS를 기반으로한 로봇이 사람을 따라다니며 보조한다.

## 팀 : 김나우의 로봇추종
|이름|학과|학번||
|:-:|:-:|:-:|:-:|
|우지훈|전자공학부|201918169|팀장|
|김성민|컴퓨터공학부|201812048|팀원|
|나영범|컴퓨터공학부|201812078|팀원|

---

### 주제 선정 배경 : 노동력을 모바일 로봇으로 대체함으로써 생산성, 효율성, 편의성 향상

### 프로젝트 목표 : 로봇의 사람 추종

### 주요 기능
* 로봇이 객체를 인식
* 로봇이 인식한 사람의 움직임을 따라 추종
* 사람 추종 중 대상이 이탈 시 제자리에서 회전하며 사람 탐색
* Running Mate 기능 : 주행 시간, 주행 거리, 현재 속도, 평균 속도 표시
---

개발기간 : 2024. 9. 26 ~ 2024. 11. 30
|기간|내용|
|:-:|:-:|
|**9. 26 ~ 10. 10**|ROS2 기초 학습|
|**10. 11 ~ 10. 28**|로봇 구동, 객체 인식|
|**10. 29 ~ 11. 11**|사람 추종 알고리즘 개발|
|**11. 12 ~ 11. 25**|러닝 메이트 기능 추가|
|**11. 26 ~ 11. 30**|실사용 테스트, 디버깅 및 튜닝|

---
# 개발환경
* 사용 로봇 기종 : [LIMO Pro (WeGo 로보틱스)](https://wego-robotics.com/wego/wego01.php)
* 개발 언어 : Python 3.11.9
* 운영체제 : Ubuntu 22.04
* ROS2 Humble
* Pytorch : 딥러닝 프레임워크
* ultralytics (YOLOv8) : 객체 탐지 라이브러리
* Torchvision : 이미지 처리
* OpenCV : 이미지 처리
---
### [ROS2 humble 설치](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html#) (Robot Operating System)
```
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```
```
# 워크스페이스 생성
mkdir ~p ros2_ws/src
cd ros2_ws/src

# 로봇 구동
git clone -b humble-dev https://github.com/agilexrobotics/limo_ros2.git

# 로봇 카메라 이용
git clone https://github.com/orbbec/OrbbecSDK_ROS2.git

# 사람 추종, 달린 거리 측정
git clone https://github.com/ksm0076/KNW-follower-robot.git

cd ..
colcon build
```
```
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```
터미널 다시 실행

---
각 터미널에서 실행
```
ros2 launch limo_base limo_base.launch.py
ros2 launch orbbec_camera dabai.launch.py
python3 segment_human.py
python3 Running_Mate.py
```
![image](https://github.com/user-attachments/assets/c58e4a99-b743-441d-9fd5-05c94c928e89)

Start/Stop Move : 사람 추종 시작/정지

Start/Stop Measure : 측정 시작/정지

---
## 사람 추종 알고리즘의 전체적인 흐름

**1. yolo를 통한 사람 객체인식, 객체인식한 이미지를 토픽으로 발행(실시간 확인)**

**2. 객체 인식된 이미지에서 바운딩 박스의 좌표를 3D 좌표로 변환**

**3. 3D 좌표를 이용해 각도와 거리 계산, 로봇 이동 토픽 발행**

![image](https://github.com/user-attachments/assets/361eaa70-ea7b-4e47-9b21-034eaa430ac7)
---

## 1. segment_human.py
- 카메라에서 발행되는 이미지를 구독
- 그 이미지에서, yolo를 이용해 사람 객체 검출
- 검출한 사람의 bounding box 좌표를 이용해 3D 좌표로 변환 -> publish
  
**def image_listener_callback(self, msg)**

0. 사람을 표시한 boungding box가 존재하지 않는다면, No Human, 함수 종료
1. 존재한다면, target_pixel(bounding box의 가운데 좌표) 기록
2. 사람 감지 여부 publish (사람을 못찾았을 경우 회전을 하기 위함)
3. bounding box를 그린 이미지 publish (필수x, 시각적으로 확인용)

**def depth_callback(self, msg)**

0. target_pixel이 image_listener_callback에서 기록되지 않았다면, 함수 종료 (= 사람을 발견하지 못했다면)
1. 기록되었다면 깊이를 이용해 z축을 포함한 3D 공간 좌표로 변환 -> ROS2 좌표계로 변환 (디폴트 옵티컬 좌표계)
2. cmd_robot.py에서 구독할 수 있도록 좌표 publish (x, y, z)

**def imageDepthInfoCallback(self, cameraInfo)**

* 3D 좌표로 변환시키기 위해 필요한 카메라 정보 저장

## 2. cmd_robot.py
- segment_human.py 에서 받은 3D 좌표 -> (x,y) 값 이용해서 각도, 거리 계산  
- 선속도, 각속도 조절
- cmd_vel 발행해서 로봇 이동

**def pallet_callback(self, msg)**
1. segment_human.py 에서 발행된 좌표 구독
2. 좌표 받은 시간 기록
3. 거리 계산
4. 거리가 1.5m 초과라면, 각도 계산
5. 거리, 각도에 따른 속도 조절
6. cmd_vel 토픽 publish, 로봇 이동
7. 거리가 1.5m 이하라면 stop_robot

**def status_callback(self, msg)**

1. 사람을 찾지 못했다는 정보가 들어오면 시간 기록
2. 사람을 1초동안 찾지 못했다면 rotate_to_find_person

**def rotate_to_find_person(self)**

* 마지막으로 검출한 바운딩 박스의 좌표에 따라 방향을 선택해서 회전

**def stop_robot(self)**

* 로봇 정지 cmd_vel 토픽 발행

**def calculate_distance(self)**

* 피타고라스 정리를 이용해 거리 계산

**def calculate_yaw_error(self)**

* math.atan2 를 이용해서 각도 계산
