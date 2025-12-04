#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdint>
#include <string>

#include <opencv2/opencv.hpp>

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

/// 1. txt에서 스캔 여러 개 읽기
///    - '#' 로 시작하는 줄은 헤더/구분자로 보고 스킵
///    - 데이터 라인은 "Angle Distance Quality" 형태 (float, float, int)
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
            // 빈 줄은 무시
            continue;
        }

        // 헤더 / 주석 라인 처리
        if (line[0] == '#') {
            if (!current.points.empty()) {
                scans.push_back(current);
                current.points.clear();
            }
            continue;
        }

        float a, d;
        int q;
        // 데이터 라인: "각도 거리 품질" 형식
        if (sscanf(line.c_str(), "%f %f %d", &a, &d, &q) == 3) {
            current.points.push_back({a, d, q});
        }
    }

    // 마지막 스캔 처리
    if (!current.points.empty()) {
        scans.push_back(current);
    }

    std::cout << "Loaded " << scans.size() << " scans.\n";
    return scans;
}

/// 2. 빈 occupancy grid 생성
OccupancyGrid createGrid(float map_size_m = 10.0f, float res_m = 0.05f) {
    OccupancyGrid og;
    og.res_m = res_m;
    og.half_size_m = map_size_m / 2.0f;

    og.width  = static_cast<int>(map_size_m / res_m);
    og.height = static_cast<int>(map_size_m / res_m);

    og.data.assign(og.width * og.height, -1); // unknown으로 초기화
    return og;
}

/// world 좌표(x, y, m 단위)를 grid index로 변환
inline bool worldToGrid(float x, float y, const OccupancyGrid& og, int& gx, int& gy) {
    gx = static_cast<int>((x + og.half_size_m) / og.res_m);
    gy = static_cast<int>((y + og.half_size_m) / og.res_m);
    return (0 <= gx && gx < og.width && 0 <= gy && gy < og.height);
}

/// 3. 한 스캔을 그리드에 반영 (매우 단순: 끝점만 occupied로 표시)
void addScanToGrid(const Scan& scan, OccupancyGrid& og) {
    const float max_range_m = 2.0f; // 2m까지만 사용 (원하면 조절)

    for (const auto& p : scan.points) {
        float dist_m = p.dist_mm / 1000.0f;
        if (dist_m <= 0.0f || dist_m > max_range_m) continue;

        float rad = p.angle_deg * static_cast<float>(M_PI) / 180.0f;

        // 레이 따라 몇 칸을 찍을지
        int steps = static_cast<int>(dist_m / og.res_m);
        if (steps < 1) steps = 1;

        // 1) 로봇(0,0) → hit 지점까지 free 셀 채우기
        for (int i = 0; i < steps; ++i) {
            float r = i * og.res_m;      // 0, res, 2*res, ...
            float x = r * std::cos(rad);
            float y = r * std::sin(rad);

            int gx, gy;
            if (worldToGrid(x, y, og, gx, gy)) {
                int idx = gy * og.width + gx;
                if (og.data[idx] == -1) {    // 아직 미방문이면 free 로
                    og.data[idx] = 0;        // free
                }
            }
        }

        // 2) 끝점은 occupied 로 표시
        float x_hit = dist_m * std::cos(rad);
        float y_hit = dist_m * std::sin(rad);

        int gx, gy;
        if (worldToGrid(x_hit, y_hit, og, gx, gy)) {
            int idx = gy * og.width + gx;
            og.data[idx] = 100;   // 장애물
        }
    }
}


/// 4. occupancy grid를 OpenCV 이미지로 변환
cv::Mat gridToImage(const OccupancyGrid& og) {
    cv::Mat img(og.height, og.width, CV_8UC1);

    for (int y = 0; y < og.height; ++y) {
        for (int x = 0; x < og.width; ++x) {
            int8_t v = og.data[y * og.width + x];
            uint8_t c;
            if (v == -1)       c = 127; // unknown -> 회색
            else if (v == 0)   c = 255; // free (지금은 안 쓰고 있지만 남겨둠)
            else               c = 0;   // occupied -> 검정

            // y 뒤집어서 위쪽이 +y가 되게
            img.at<uint8_t>(og.height - 1 - y, x) = c;
        }
    }

    return img;
}

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

    // 우선 1개 스캔만으로 테스트
    OccupancyGrid og = createGrid(10.0f, 0.05f);
    addScanToGrid(scans[0], og);

    cv::Mat img = gridToImage(og);
    cv::imwrite("map_test.png", img);
    std::cout << "Saved map_test.png\n";

    cv::imshow("map", img);
    cv::waitKey(0);

    return 0;
}
