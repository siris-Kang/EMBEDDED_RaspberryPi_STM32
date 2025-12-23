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

# 실행: mqtt_host mqtt_port http_port
./rpi_cam 127.0.0.1 1883 8000
```



## mapMQTT

폴더 구조




## motorMQTT

폴더 구조



