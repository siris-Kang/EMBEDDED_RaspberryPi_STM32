#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>

#include <opencv2/opencv.hpp>

// ====== 구조체 정의 ======
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
    float half_size_m;  // 한 변 길이의 1/2 (m)
    std::vector<int8_t> data; // -1: unknown, 0: free, 100: occupied
};

constexpr float PI = 3.1415926535f;

// ====== 1. txt에서 스캔 읽기 ======
// 형식 예시:
// #RPLIDAR SCAN DATA
// #COUNT=401
// #Angule Distance Quality
// 352.6227 1358.0 188
// 353.2324 1362.0 188
std::vector<Scan> loadScansFromTxt(const std::string& path) {
    std::ifstream fin(path);
    if (!fin) {
        std::cerr << "Failed to open file: " << path << std::endl;
        return {};
    }

    std::vector<Scan> scans;
    Scan current;
    std::string line;

    while (std::getline(fin, line)) {
        if (line.empty()) {
            continue;
        }

        // 주석/헤더 라인
        if (line[0] == '#') {
            // 만약 지금까지 쌓인 포인트가 있으면 하나의 스캔으로 마무리
            if (!current.points.empty()) {
                scans.push_back(current);
                current.points.clear();
            }
            continue;
        }

        float a, d;
        int q;
        // Angle Distance Quality
        if (std::sscanf(line.c_str(), "%f %f %d", &a, &d, &q) == 3) {
            current.points.push_back({a, d, q});
        }
    }

    // 마지막 스캔 처리
    if (!current.points.empty()) {
        scans.push_back(current);
    }

    std::cout << "Loaded " << scans.size() << " scan(s).\n";
    return scans;
}

// ====== 2. occupancy grid 생성 ======
OccupancyGrid createGrid(float map_size_m = 10.0f, float res_m = 0.05f) {
    OccupancyGrid og;
    og.res_m = res_m;
    og.half_size_m = map_size_m / 2.0f;

    og.width  = static_cast<int>(map_size_m / res_m);
    og.height = static_cast<int>(map_size_m / res_m);
    og.data.assign(og.width * og.height, -1); // unknown

    return og;
}

// world 좌표(x,y[m]) -> grid index
inline bool worldToGrid(float x, float y, const OccupancyGrid& og, int& gx, int& gy) {
    gx = static_cast<int>((x + og.half_size_m) / og.res_m);
    gy = static_cast<int>((y + og.half_size_m) / og.res_m);
    return (0 <= gx && gx < og.width && 0 <= gy && gy < og.height);
}

// ====== 3. scan을 전역 좌표에서 grid에 반영 ======
void addScanToGridGlobal(const Scan& scan,
                         OccupancyGrid& og,
                         const Pose2D& pose,
                         float sensor_max_m)
{
    for (const auto& p : scan.points) {
        float dist_m = p.dist_mm / 1000.0f;
        if (dist_m <= 0.0f || dist_m > sensor_max_m)
            continue;

        float angle_sensor = p.angle_deg * PI / 180.0f;
        float angle_world  = pose.yaw + angle_sensor;

        // 레이 따라 이동하면서 free 채우기
        float use_dist = dist_m;
        int steps = static_cast<int>(use_dist / og.res_m);
        if (steps < 1) steps = 1;

        for (int i = 0; i < steps; ++i) {
            float r  = i * og.res_m;     // 0, res, 2*res, ...
            float lx = r * std::cos(angle_world);
            float ly = r * std::sin(angle_world);

            float wx = pose.x + lx;
            float wy = pose.y + ly;

            int gx, gy;
            if (worldToGrid(wx, wy, og, gx, gy)) {
                int idx = gy * og.width + gx;
                if (og.data[idx] == -1) {
                    og.data[idx] = 0;   // unknown -> free
                }
            }
        }

        // 끝점은 occupied
        float hx = pose.x + dist_m * std::cos(angle_world);
        float hy = pose.y + dist_m * std::sin(angle_world);

        int gx_hit, gy_hit;
        if (worldToGrid(hx, hy, og, gx_hit, gy_hit)) {
            int idx = gy_hit * og.width + gx_hit;
            og.data[idx] = 100;  // 장애물
        }
    }
}

// ====== 4. 포즈 점수 계산 (scan이 기존 맵과 얼마나 잘 맞는지) ======
float scorePose(const Scan& scan,
                const OccupancyGrid& og,
                const Pose2D& pose,
                float sensor_max_m)
{
    float score = 0.0f;

    for (const auto& p : scan.points) {
        float dist_m = p.dist_mm / 1000.0f;
        if (dist_m <= 0.0f || dist_m > sensor_max_m)
            continue;

        float angle_sensor = p.angle_deg * PI / 180.0f;
        float angle_world  = pose.yaw + angle_sensor;

        float hx = pose.x + dist_m * std::cos(angle_world);
        float hy = pose.y + dist_m * std::sin(angle_world);

        int gx, gy;
        if (!worldToGrid(hx, hy, og, gx, gy))
            continue;

        int idx = gy * og.width + gx;
        int8_t cell = og.data[idx];

        if (cell == 100)      score += 3.0f;  // 장애물 위에 hit
        else if (cell == 0)   score -= 1.0f;  // free 위에 hit
        else                  score += 0.0f;  // unknown
    }

    return score;
}

// ====== 5. grid search로 포즈 미세 조정 (초간단 scan matching) ======
Pose2D refinePoseByGridSearch(const Scan& scan,
                              const OccupancyGrid& og,
                              const Pose2D& initial,
                              float sensor_max_m)
{
    Pose2D best = initial;
    float bestScore = -1e9f;

    const float trans_step = 0.02f;                 // 2cm
    const float rot_step   = 1.0f * PI / 180.0f;    // 1deg
    const int trans_range_steps = 5;  // ±5*2cm = ±10cm
    const int rot_range_steps   = 5;  // ±5deg

    for (int dx_i = -trans_range_steps; dx_i <= trans_range_steps; ++dx_i) {
        for (int dy_i = -trans_range_steps; dy_i <= trans_range_steps; ++dy_i) {
            for (int dth_i = -rot_range_steps; dth_i <= rot_range_steps; ++dth_i) {

                Pose2D cand;
                cand.x   = initial.x + dx_i * trans_step;
                cand.y   = initial.y + dy_i * trans_step;
                cand.yaw = initial.yaw + dth_i * rot_step;

                float s = scorePose(scan, og, cand, sensor_max_m);
                if (s > bestScore) {
                    bestScore = s;
                    best = cand;
                }
            }
        }
    }
    return best;
}

// ====== 6. occupancy grid -> OpenCV 이미지 ======
cv::Mat gridToImage(const OccupancyGrid& og) {
    cv::Mat img(og.height, og.width, CV_8UC1);

    for (int y = 0; y < og.height; ++y) {
        for (int x = 0; x < og.width; ++x) {
            int8_t v = og.data[y * og.width + x];
            uint8_t c;
            if (v == -1)      c = 127; // unknown -> 회색
            else if (v == 0)  c = 255; // free -> 흰색
            else              c = 0;   // occupied -> 검정

            // y 뒤집기 (이미지 위쪽이 +y 처럼 보이도록)
            img.at<uint8_t>(og.height - 1 - y, x) = c;
        }
    }
    return img;
}

// ====== 7. main ======
int main(int argc, char** argv) {
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " lidar_log.txt\n";
        return 0;
    }

    std::string path = argv[1];
    auto scans = loadScansFromTxt(path);
    if (scans.empty()) {
        std::cerr << "No scans loaded.\n";
        return 1;
    }

    // 방 전체를 커버할 정도의 글로벌 맵 설정 (필요에 따라 값 조절)
    float map_size_m = 10.0f;   // 10m x 10m
    float res_m      = 0.05f;   // 5cm resolution
    OccupancyGrid og = createGrid(map_size_m, res_m);

    float sensor_max_m = 6.0f;  // 라이다 최대 거리 (대충 값)

    // 시작 포즈
    Pose2D pose{0.0f, 0.0f, 0.0f};

    // 1) 첫 스캔: 초기 포즈로 맵에 반영
    addScanToGridGlobal(scans[0], og, pose, sensor_max_m);

    // 2) 테스트용: 스캔이 1개뿐이므로,
    //    "로봇이 앞으로 조금씩 직진하면서 같은 스캔을 반복해서 본다"고 가정
    //    (실제 SLAM에서는 각 시점마다 다른 스캔이 들어옴)
    int num_steps = 20;          // 총 20번 움직인다고 가정
    float step_dist = 0.1f;      // 한 번에 0.1m(10cm) 전진

    for (int i = 1; i <= num_steps; ++i) {
        // (1) 이전 포즈에서 "로봇이 전진했다" 가정
        Pose2D odom_guess = pose;
        odom_guess.x += step_dist;      // x 방향으로 직진 가정

        // (2) 실제 SLAM이면: odom_guess를 initial로 해서 grid search로 보정
        //     지금은 맵도, 스캔도 가짜 상황이므로 일단 그대로 쓰거나,
        //     원하면 refinePoseByGridSearch 사용 가능.
        Pose2D refined = refinePoseByGridSearch(scans[0], og, odom_guess, sensor_max_m);

        pose = refined; // 현재 포즈 업데이트

        std::cout << "Step " << i
                  << ": pose = (" << pose.x << ", "
                  << pose.y << ", " << pose.yaw << ")\n";

        // (3) 이 포즈에서 같은 스캔을 다시 관측했다고 가정하고 맵 업데이트
        addScanToGridGlobal(scans[0], og, pose, sensor_max_m);

        // (4) 중간 과정 PNG로 보고 싶으면 주석 해제
        
        cv::Mat step_img = gridToImage(og);
        std::string fname = "map_step_" + std::to_string(i) + ".png";
        cv::imwrite(fname, step_img);
        
    }

    // 최종 맵 이미지 저장
    cv::Mat img = gridToImage(og);
    cv::imwrite("map_global.png", img);
    std::cout << "Saved map_global.png\n";

    // GUI 가능하면 화면에도 띄워보기
    cv::imshow("map_global", img);
    cv::waitKey(0);

    return 0;
}
