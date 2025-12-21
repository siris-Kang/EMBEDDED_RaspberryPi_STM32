#pragma once
/**
 * @file app_types.h
 * @brief MQTT 메시지(JSON)와 GUI 내부에서 공용으로 쓰는 데이터 타입 모음
 *
 * - UI(Qt Widgets)와는 완전히 분리되어야 함 (순수 데이터)
 * - 이 파일은 다른 헤더들이 서로 끌어당기며 꼬이는 걸 막는 "중앙 타입" 역할
 */

#include <QString>

struct Shot {
    QString shotId;

    double x = 0.0;     // world (meters), map frame
    double y = 0.0;
    double theta = 0.0;

    QString imageUrl;
    QString thumbUrl;
    QString timestamp;
};

struct MapMeta {
    QString mapId;

    int widthCells = 0;
    int heightCells = 0;
    double resolution = 0.05;

    double originX = 0.0;
    double originY = 0.0;
    double originYaw = 0.0;

    QString imageUrl;
    QString timestamp;
};

struct MotorCmd {
    QString source = "unknown";
    QString mode   = "unknown";

    int speed = 0;           // 0~100 (UI 기준)
    int steer = 90;          // 0~180 (UI 기준)

    bool enable = true;
    bool emergencyStop = false;

    QString timestamp;
};
