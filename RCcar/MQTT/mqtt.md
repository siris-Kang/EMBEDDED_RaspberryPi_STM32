# MQTT

## cameraMQTT
> Raspberry pi와 USB 웹캠 연결

라즈베리파이에 연결된 USB 카메라로 주변을 주기적으로 캡처 하고,  
저장된 이미지에 접근 가능한 HTTP URL을 생성해서  
MQTT로 GUI/ROS 쪽에 알려주는 역할

### 폴더 구조
```
rpi_cam_service/
  CMakeLists.txt
  include/
    app.h
    app_config.h
    camera_worker.h
    http_server.h
    json_mini.h
    mqtt_client.h
    utils.h
  src/
    app.cpp
    camera_worker.cpp
    http_server.cpp
    json_mini.cpp
    main.cpp
    mqtt_client.cpp
    utils.cpp
```

### CMake 빌드
```
sudo apt-get update
sudo apt-get install -y libmosquitto-dev mosquitto mosquitto-clients libopencv-dev cmake g++

mkdir -p build
cd build
cmake .. # 빌드 파일 생성
make -j # 컴파일
```

### 실행: mqtt_host mqtt_port http_port
```
./rpi_cam 127.0.0.1 1883 8000
```



## mapMQTT
> robot/map_state를 MQTT를 이용해 GUI로 보냄  

GUI는 robot/map_state를 MQTT로 받으면,  
image_url로 PNG 받아서 QLabel에 그림  

### 폴더 구조
```
ros_map_mqtt_bridge/
  CMakeLists.txt
  package.xml
  include/
    http_server.h
    mqtt_pub.h
    utils.h
    map_bridge_node.h
  src/
    http_server.cpp
    mqtt_pub.cpp
    utils.cpp
    map_bridge_node.cpp
    main.cpp
```

### CMake 빌드
```
sudo apt-get update
sudo apt-get install -y libmosquitto-dev python3 opencv4 libopencv-dev

# 워크스페이스 예시
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 여기에 ros_map_mqtt_bridge 폴더를 넣기
cd ~/ros2_ws
colcon build --packages-select ros_map_mqtt_bridge
source install/setup.bash

```

### 실행
```
ros2 run ros_map_mqtt_bridge map_mqtt_bridge \
  --ros-args \
  -p mqtt_host:=127.0.0.1 \
  -p mqtt_port:=1883 \
  -p http_port:=8001 \
  -p out_dir:=~/maps/map_images \
  -p publish_hz:=2.0
```


## motorMQTT
> Raspberry pi와 USB 웹캠 연결

motor 어케 움직여야할지 구독해서 GUI로도 보내고 STM UART로도 보내는 부분 만들어야하거든
### 폴더 구조
```

```

### CMake 빌드
```

```



