# autorace_gazebo — Blender v014 시뮬레이션

팀원의 Gazebo 패키지와 `autorace_v014_ubuntu_handoff.zip`의 Blender 레퍼런스를 바탕으로 한 ROS 2 / Gazebo Classic 11 시뮬레이션이다. 기본 월드는 바닥, 신호등, 좌우 교차로 표지판, 차단바, 벽·터널·장애물, 판정 영역과 경기 관리자를 포함한다. 기본 로봇은 `burger_cam_bag`다.

## 정적 표지판의 Blender 원본 형상 복원 — 2026-11-11

공사·교차로·진입금지·주차·주차장 옆 좌회전·정지·터널 표지 7종은 이제
`autorace_mcp_v014_mission_devices.blend`의 실제 형상을 사용한다. 이전에는 이 7종에
`turtlebot3_autorace_2020`의 직육면체 모델을 사용해 표지와 기둥 그림이 옆면까지 늘어났다.
접이식 교차로 방향 표지 2개는 이미 Blender 메시를 사용하고 있었으며 이번 변경 대상이 아니다.

- 판의 윤곽·두께·뒷면, 기둥·받침·고정부·볼트, bevel과 법선, UV와 내장 텍스처를 원본에서 가져왔다.
- 숨겨진 옛 직육면체와 충돌 참조는 내보내지 않는다. 진입금지는 원본의 뒤쪽 표지판도 포함한다.
- 주차·터널·진입금지 표지의 방향은 Blender 원본을 따른다. 수평 위치에 기존 맵 배율260/227만 적용하며, 표지 크기는 확대하지 않는다. 받침 높이는 현재 평탄화된 바닥에 맞춘다.
- 정적 표지 충돌은 판·기둥·소켓·받침의 메시로 구성한다. 원래 직육면체 전체가 막던 빈 공간은 충돌 형상에서도 제거했다.
- 주행·인식 코드, 카메라 설정, 바닥, 동적 장치와 경기 판정은 변경하지 않는다. 이 변경만으로 자동 주행 성공이 검증되는 것은 아니다.

실행에 Blender는 필요 없다. `models/autorace_traffic_*_v014`와 `config/static_signs_v014.json`이 포함돼 있다.
원본에서 다시 내보낼 때만 Blender 5를 사용한다. 입력 `.blend`는 저장하거나 수정하지 않는다.

```bash
blender -b --factory-startup --disable-autoexec --python tools/export_static_signs.py -- \
  --blend /path/to/autorace_mcp_v014_mission_devices.blend
```

이후 `tools/build_assets.py`로 전체 월드를 재생성하면 해당 표지판을 다시 연결한다.
생성 도구는 예전 직육면체 모델로 되돌아가지 않는다. 모델을 추가한 뒤 패키지를 다시 빌드하고 Gazebo를 재시작한다.

## 0.4.0 bag 기준 두 카메라 배치

`bag_11_4_all`의 실제 영상과 사용자 측정값을 적용했다. 설정은 `config/camera_rig.json`이며, 지면 높이와 SDF 모델 좌표를 구분한다. 모델 원점은 평지에서 명목상 지면 위 10mm다.

| 카메라 | 지면 기준 센서 높이 | 아래쪽 각도 | 모델 기준 위치 x/y/z | 근거 |
|---|---:|---:|---|---|
| `/camera` 차선 | 135mm | 31.6° | 76 / 0 / 125mm | 높이 실측, 각도 실측 범위30~35° 안에서 bag 직선 차선 소실점으로 추정 |
| `/camera2` 표지판 | 약181mm | 약0.8° | −32 / −57 / 171mm | 라이다 높이·오른쪽 장착 설명과 bag 바닥 구도로 추정 |

- `/camera`는 bag과 같은 **보정 후 640×480** 시야다. fx=426.355736, fy=536.586855, cx=324.860029, cy=287.506008이며 기존 단순1280×720/FOV 모델의 세로 비율과 중심점 차이를 반영했다. 왜곡된 `/camera_raw_16_9`가 아닌 실제 주행 입력 `/camera`를 기준으로 했다. 앞뒤 위치76mm와 yaw2.26°는 기존 설정을 유지한 값으로 실측으로 확정하지 않았다.
- `/camera2`도640×480이며 yaw는 약5.2°다. 오른쪽57mm·앞뒤−32mm는 초기 bag의 직선 차선/첫 커브를 이용한 **잠정 위치 값**이다. `CameraInfo`나 camera2 보정 파일이 bag에 없어 실제 하드웨어 내부 파라미터를 확정한 것은 아니다. 라이다 옆 장착 위치를 다시 측정하거나 camera2 캘리브레이션을 확보하면 갱신한다. 렌즈 왜곡은 현재0으로 둔다.
- **2026-11-11 임시 화각 확대:** `/camera2`의 수평 화각을 기존34.518°에서80°로 넓혔다. 현재 fx=381.361150, fy=335.079457, cx=320, cy=240이다. 기존 fx1030/fy905도 실측이 아닌 bag 구도 추정값이었으며, 이번 값은 교차로 방향 표지를 시야에 넣기 위한 시뮬레이션 전용 설정이다. 가로·세로 초점거리에 같은 배율을 적용해 영상 비율을 유지했다. 장착 위치·각도·해상도·30Hz와 차선용 `/camera`는 그대로다. 표지는 화면에서 작아지므로 시야 확보와 기존 인식 조건 충족/자동 통과는 별도로 확인해야 한다.
- 이전 화각으로 복구하려면 `config/camera_rig.json`의 camera2에서 fx=1030, fy=905로 되돌리고, `temporary_fov_override.previous_bag_estimate.evidence`의 설명을 `evidence`로 복구한 뒤 `temporary_fov_override` 항목을 제거한다. 아래 생성기를 실행하고 Gazebo를 재시작한다.
- 원본 맵은 현재260/227배로 확대돼 있다. bag 초기 영상과 센서 높이로 계산한 차선 중심 간격은 약249mm인데 현재 맵은 약286mm이므로 두 영상의 차선 폭이 완전히 겹치지는 않는다. 카메라 높이를 임의로 바꿔 이 차이를 상쇄하거나 이번에 맵 크기를 바꾸지는 않았다.
- 센서는 `/camera/image_raw`, `/camera/image_raw/compressed`, `/camera2/image_raw`, `/camera2/image_raw/compressed`도 발행한다. 시뮬레이션 전용 `camera_adapter.py`가 JPEG를 그대로 `/camera`·`/camera2`에 전달하며 실제 렌더링과 같은 K/P의 `/camera/camera_info`·`/camera2/camera_info`를 발행한다. 기존 Gazebo ROS 플러그인은 K에서 fx=fy를 가정하므로 그 근사 CameraInfo는 `/autorace/internal/*_info`로 분리했다. 영상 처리·주행 제어는 이 어댑터에 없다.
- SDF 센서 자세와 동일한 `camera_sensor_frame`/`camera2_sensor_frame` 및 optical TF를 시뮬레이션 launch에서 생성한다. raw URDF에 남던 `${namespace}`도 여기서 치환했다. 기존 주행·미션·실카메라 소스는 변경하지 않았다.

설정을 조정할 때는 패키지 소스에서 다음을 실행한 뒤 Gazebo를 재시작한다. 생성기는 카메라 센서만 갱신하고 바퀴·충돌·관성·LiDAR·구동 플러그인을 보존한다.

```bash
python3 tools/make_bag_camera_model.py
```

## 0.3.0 시각 개선

- Blender 원본의 4096×4096 검정 바닥·흰색/노란색 차선 텍스처를 그대로 사용한다. 이미지 파일을 다시 칠하지 않았다.
- 바닥 COLLADA 72개에 빠져 있던 면 법선을 추가하고, 텍스처 재질의 주변광 반사값을 명시했다. 조명이 반영되지 않아 흰 차선이 회색, 노란 차선이 어둡게 보이던 문제를 보완한다.
- 월드의 강한 점광원 3개를 제거하고 중성 방향광과 약한 보조광을 사용한다. 주변 바닥은 어두운 중성색으로 정리했다.
- 신호등은 Blender의 빨강·노랑·초록 렌즈 색과 발광/비발광 구분을 반영한다. 점등 색이 하얗게 번지는 효과를 줄였다.
- 기존 도로 좌표·폭·높이, 충돌 형상, 로봇과 카메라 설정, 장치 조인트, 경기 판정 및 주행·미션 패키지는 보존했다. 이 패키지는 자동 주행 노드를 실행하지 않는다.

## 현재 Ubuntu 환경에서 빌드·실행

```bash
source /opt/ros/humble/setup.bash
source /usr/local/share/gazebo/setup.bash
source /home/ubuntu/gazebo_ros_ws/install/setup.bash
cd /home/ubuntu/turtlebot3_ws
export CMAKE_PREFIX_PATH="/home/ubuntu/opt/sdformat9:/usr/local:$CMAKE_PREFIX_PATH"
export PYTHONNOUSERSITE=1 MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon build --symlink-install --executor sequential --parallel-workers 1 \
  --packages-select autorace_gazebo --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
export ROS_DOMAIN_ID=112 ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11362
export DISPLAY="${DISPLAY:-:0}"
ros2 launch autorace_gazebo autorace.launch.py
```

같은 master에서 이미 실행 중이면 중복 실행하지 않는다. 예전 검증 폴더의 `sim_env.bash`는 원본 패키지를 마지막에 source하므로 이 개선본 실행에는 사용하지 않는다. `ros2 pkg prefix autorace_gazebo`의 결과가 `/home/ubuntu/turtlebot3_ws/install/autorace_gazebo`인지 확인한다.

다른 Classic 11/Humble 설치 환경에서는 위의 로컬 underlay 경로를 해당 환경에 맞춘다. GUI가 필요 없으면 `gui:=false`, 경기 관리자 없이 장치 명령만 시험하려면 `referee:=false`를 추가한다. 모델은 `model:=burger_cam_bag|burger_cam|burger|waffle|waffle_pi`, 시작 위치는 `x:= y:= z:= yaw:=`로 지정한다. 두 카메라 영상은 위 토픽의 640×480 JPEG다. `/camera`와 `/camera2` 별칭을 제공하며 주행·미션 노드는 별도로 실행해야 한다.

## 좌표와 물리 설정

- 바닥 모델 yaw는 기존 −3.14이며 월드 좌표는 대략 Blender 좌표의 (−x, −y)다.
- 맵은 기존 Gazebo 패키지와 동일하게 원본 대비 수평 260/227배다. 차선 안쪽 폭 약 260mm, 차선 굵기 약 26mm, 전체 약 4.58×4.58m다. 장치 크기/높이는 원본 설정을 유지한다.
- 기본 스폰은 `(0.687, −1.999, 0.010)`, yaw 0이며 빨간 출발선 뒤다.
- `burger_cam_bag`의 바퀴 지름은 70mm, 간격은 180mm다.
- 도로판의 기존 높이 축소 설정(30mm→0.5mm)을 유지한다. `autorace_panel_surface_steps_2mm`는 별도 단차 시험용이다.

## 에셋 재생성

법선·재질·조명만 다시 적용하려면 시스템 Python의 NumPy로 실행한다. 같은 입력에 반복 실행해도 결과가 바뀌지 않는다.

```bash
python3 tools/improve_visuals.py
```

GLB에서 장치 등을 전체 재생성할 때는 `tools/build_assets.py --handoff /path/to/autorace-v014-handoff`를 사용한다. 이 도구도 마지막에 같은 시각 보정을 적용하고, 포함된 Blender 정적 표지 모델을 연결한다. Blender 실행은 필요하지 않지만 NumPy, ROS의 `ament_index_python`, `turtlebot3_gazebo` 에셋이 필요하다. 바닥과 정적 표지 모델은 이 패키지에 포함된다. 벽·장애물 등 나머지 정적 소품은 여전히 2020 모델 기반이며 전체 `.blend`를 모두 이식한 것은 아니다.

Blender의 색 관리/PBR과 Gazebo Classic의 Phong 렌더러는 다르므로 완전히 동일한 렌더 결과를 목표로 하지는 않는다. 이번 변경의 검증 범위는 빌드, 실제 GUI·카메라·센서 출력, 원본 텍스처/기하/주행 소스 보존이며 자동 주행 성공을 보장하지 않는다.
