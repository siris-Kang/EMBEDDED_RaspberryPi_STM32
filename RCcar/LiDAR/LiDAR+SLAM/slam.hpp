#ifndef SLAM_HPP
#define SLAM_HPP

#include <vector>
#include <string>
#include <cstdint>
#include <opencv2/opencv.hpp>

constexpr float SLAM_PI = 3.1415926535f;

// ----------------- 기본 구조체들 -----------------
struct Pose2D {
    float x;   // m
    float y;   // m
    float yaw; // rad
};

struct LidarPoint {
    float angle_deg;
    float dist_mm;
    int   quality;
};

struct Scan {
    std::vector<LidarPoint> points;
};

struct OccupancyGrid {
    int width;
    int height;
    float res_m;        // cell size (m)
    float half_size_m;  // half of map size (m)

    // 이건 예전과 동일: -1 unknown, 0 free, 100 occupied
    std::vector<int8_t> data;

    // log-odds 형태로 확률을 저장 (새로 추가)
    //  l = log( p / (1 - p) ), 초기값 0 → p=0.5 (unknown)
    std::vector<float> log_odds;
};

// ----------------- I/O -----------------
std::vector<Scan> loadScansFromTxt(const std::string& path);

// ----------------- 맵 생성/좌표 변환 -----------------
OccupancyGrid createGrid(float map_size_m, float res_m);
bool worldToGrid(float x, float y, const OccupancyGrid& og, int& gx, int& gy);

// ----------------- occupancy 업데이트 -----------------
void updateWithScan(const Scan& scan,
                    OccupancyGrid& og,
                    const Pose2D& pose,
                    float sensor_max_m,
                    float hit_logodds = 2.0f,   // 히트시 +2
                    float miss_logodds = -0.5f  // 레이 경로 free에 -0.5
);

// log-odds → data(-1/0/100)로 다시 계산
void recomputeDiscreteOccupancy(OccupancyGrid& og,
                                float occ_thresh = 0.65f,
                                float free_thresh = 0.35f);

// ----------------- pose scoring / scan matching -----------------
float scorePose(const Scan& scan,
                const OccupancyGrid& og,
                const Pose2D& pose,
                float sensor_max_m);

// grid search로 pose refinement (초간단 multi-param 튜닝 가능)
Pose2D refinePoseGridSearch(const Scan& scan,
                            const OccupancyGrid& og,
                            const Pose2D& initial,
                            float sensor_max_m,
                            float trans_range_m,   // ±몇 m 범위에서 찾을지
                            float trans_step_m,    // m 단위 step
                            float rot_range_deg,   // ±몇 도 범위
                            float rot_step_deg);   // 도 단위 step

// ----------------- 시각화 -----------------
cv::Mat gridToImage(const OccupancyGrid& og,
                    float occ_thresh = 0.65f,
                    float free_thresh = 0.35f);

                    
// ----------------- 로컬 플래너 (방향/속도 결정) -----------------

// 로봇에게 줄 "명령"을 이 형태로 표현
struct VelocityCommand {
    float v;      // 전진 속도 [m/s]
    float omega;  // 회전 속도 [rad/s]
};

// 현재 포즈에서, 맵을 보고 "가장 뚫린 방향"을 고르는 함수
//  - max_check_dist : 이 거리까지 앞을 보며 장애물 체크
//  - fov_deg        : 좌우 몇 도 범위에서 후보 각도 볼지 (예: 90도면 -45~+45)
//  - step_deg       : 몇 도 간격으로 스캔할지 (예: 5도면 -45, -40, ... , +45)
float chooseOpenDirection(const OccupancyGrid& og,
                          const Pose2D& pose,
                          float max_check_dist,
                          float fov_deg,
                          float step_deg);

// 선택된 방향(desired_dir)을 기준으로 속도/조향 명령 계산
VelocityCommand computeVelocityCommand(const Pose2D& pose,
                                       float desired_dir,
                                       float max_speed,
                                       float max_turn_rate);

#endif // SLAM_HPP
