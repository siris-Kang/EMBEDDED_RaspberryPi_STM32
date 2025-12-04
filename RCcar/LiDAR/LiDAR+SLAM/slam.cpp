#include "slam.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

// ---------- 내부 유틸 ----------

static inline float clampLogOdds(float l,
                                 float l_min = -4.0f,
                                 float l_max =  4.0f)
{
    return std::max(l_min, std::min(l_max, l));
}

static inline float logOddsToProb(float l) {
    // l = log(p/(1-p))  →  p = 1 / (1 + exp(-l))
    return 1.0f / (1.0f + std::exp(-l));
}

// ---------- 1. txt에서 스캔 읽기 ----------

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

        if (line[0] == '#') {
            if (!current.points.empty()) {
                scans.push_back(current);
                current.points.clear();
            }
            continue;
        }

        float a, d;
        int q;
        if (std::sscanf(line.c_str(), "%f %f %d", &a, &d, &q) == 3) {
            current.points.push_back({a, d, q});
        }
    }

    if (!current.points.empty()) {
        scans.push_back(current);
    }

    std::cout << "Loaded " << scans.size() << " scan(s).\n";
    return scans;
}

// ---------- 2. occupancy grid 생성 / 좌표 변환 ----------

OccupancyGrid createGrid(float map_size_m, float res_m) {
    OccupancyGrid og;
    og.res_m = res_m;
    og.half_size_m = map_size_m / 2.0f;

    og.width  = static_cast<int>(map_size_m / res_m);
    og.height = static_cast<int>(map_size_m / res_m);

    og.data.assign(og.width * og.height, -1);     // unknown
    og.log_odds.assign(og.width * og.height, 0); // p=0.5

    return og;
}

bool worldToGrid(float x, float y, const OccupancyGrid& og, int& gx, int& gy) {
    gx = static_cast<int>((x + og.half_size_m) / og.res_m);
    gy = static_cast<int>((y + og.half_size_m) / og.res_m);
    return (0 <= gx && gx < og.width && 0 <= gy && gy < og.height);
}

// ---------- 3. occupancy 업데이트 (log-odds) ----------

void updateWithScan(const Scan& scan,
                    OccupancyGrid& og,
                    const Pose2D& pose,
                    float sensor_max_m,
                    float hit_logodds,
                    float miss_logodds)
{
    const float l_min = -4.0f;
    const float l_max =  4.0f;

    for (const auto& p : scan.points) {
        float dist_m = p.dist_mm / 1000.0f;
        if (dist_m <= 0.0f || dist_m > sensor_max_m)
            continue;

        float angle_sensor = p.angle_deg * SLAM_PI / 180.0f;
        float angle_world  = pose.yaw + angle_sensor;

        // 센서가 최대 범위 근처를 보고 있다면 "hit"가 불확실하다고 보고
        // 끝점 occupied 업데이트는 건너뛰고 free만 업데이트 할 수도 있다.
        bool reliable_hit = (dist_m < sensor_max_m * 0.95f);

        // 1) 레이 경로 free 업데이트 (miss_logodds)
        float use_dist = std::min(dist_m, sensor_max_m);
        int steps = std::max(1, static_cast<int>(use_dist / og.res_m));

        for (int i = 0; i < steps; ++i) {
            float r  = i * og.res_m;
            float wx = pose.x + r * std::cos(angle_world);
            float wy = pose.y + r * std::sin(angle_world);

            int gx, gy;
            if (!worldToGrid(wx, wy, og, gx, gy)) continue;
            int idx = gy * og.width + gx;

            og.log_odds[idx] = clampLogOdds(og.log_odds[idx] + miss_logodds,
                                            l_min, l_max);
        }

        // 2) 끝점 occupied 업데이트 (hit_logodds)
        if (reliable_hit) {
            float hx = pose.x + dist_m * std::cos(angle_world);
            float hy = pose.y + dist_m * std::sin(angle_world);

            int gx_hit, gy_hit;
            if (worldToGrid(hx, hy, og, gx_hit, gy_hit)) {
                int idx = gy_hit * og.width + gx_hit;
                og.log_odds[idx] = clampLogOdds(og.log_odds[idx] + hit_logodds,
                                                l_min, l_max);
            }
        }
    }

    // log_odds가 업데이트 되었으니 data(-1/0/100)도 다시 계산
    recomputeDiscreteOccupancy(og);
}

// log-odds → data(-1/0/100)로 변환
void recomputeDiscreteOccupancy(OccupancyGrid& og,
                                float occ_thresh,
                                float free_thresh)
{
    for (int i = 0; i < og.width * og.height; ++i) {
        float l = og.log_odds[i];
        float p = logOddsToProb(l); // [0,1]

        if (p > occ_thresh) {
            og.data[i] = 100; // occupied
        } else if (p < free_thresh) {
            og.data[i] = 0;   // free
        } else {
            og.data[i] = -1;  // unknown / 중간
        }
    }
}

// ---------- 4. pose scoring (scan-to-map) ----------

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

        float angle_sensor = p.angle_deg * SLAM_PI / 180.0f;
        float angle_world  = pose.yaw + angle_sensor;

        float hx = pose.x + dist_m * std::cos(angle_world);
        float hy = pose.y + dist_m * std::sin(angle_world);

        int gx, gy;
        if (!worldToGrid(hx, hy, og, gx, gy)) continue;
        int idx = gy * og.width + gx;

        float l = og.log_odds[idx];
        float p_occ = logOddsToProb(l);

        if (p_occ > 0.6f) {
            score += 3.0f;           // 장애물 위에 잘 맞음
        } else if (p_occ < 0.3f) {
            score -= 1.0f;           // free인데 hit라고 주장 → 나쁨
        } else {
            // unknown이면 영향 적게
            score += 0.0f;
        }
    }
    return score;
}

// ---------- 5. grid search 기반 pose refinement ----------

Pose2D refinePoseGridSearch(const Scan& scan,
                            const OccupancyGrid& og,
                            const Pose2D& initial,
                            float sensor_max_m,
                            float trans_range_m,
                            float trans_step_m,
                            float rot_range_deg,
                            float rot_step_deg)
{
    Pose2D best = initial;
    float bestScore = -1e9f;

    int t_steps = static_cast<int>(trans_range_m / trans_step_m);
    int r_steps = static_cast<int>(rot_range_deg / rot_step_deg);

    for (int dx_i = -t_steps; dx_i <= t_steps; ++dx_i) {
        for (int dy_i = -t_steps; dy_i <= t_steps; ++dy_i) {
            for (int dth_i = -r_steps; dth_i <= r_steps; ++dth_i) {

                Pose2D cand;
                cand.x   = initial.x + dx_i * trans_step_m;
                cand.y   = initial.y + dy_i * trans_step_m;
                cand.yaw = initial.yaw +
                           dth_i * (rot_step_deg * SLAM_PI / 180.0f);

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

// ---------- 6. 맵 → OpenCV 이미지 ----------

cv::Mat gridToImage(const OccupancyGrid& og,
                    float occ_thresh,
                    float free_thresh)
{
    // og.data가 이미 recomputeDiscreteOccupancy를 통해
    // -1/0/100으로 채워졌다고 가정.
    cv::Mat img(og.height, og.width, CV_8UC1);

    for (int y = 0; y < og.height; ++y) {
        for (int x = 0; x < og.width; ++x) {
            int8_t v = og.data[y * og.width + x];
            uint8_t c;
            if (v == -1)      c = 127; // unknown -> 회색
            else if (v == 0)  c = 255; // free -> 흰색
            else              c = 0;   // occupied -> 검정

            img.at<uint8_t>(og.height - 1 - y, x) = c;
        }
    }
    return img;
}


// ---------- 7. 방향 결정 + 속도 명령 ----------
// -pi ~ pi 범위로 angle을 정규화하는 유틸
static inline float normalizeAngle(float a) {
    while (a >  SLAM_PI) a -= 2.0f * SLAM_PI;
    while (a < -SLAM_PI) a += 2.0f * SLAM_PI;
    return a;
}

float chooseOpenDirection(const OccupancyGrid& og,
                          const Pose2D& pose,
                          float max_check_dist,
                          float fov_deg,
                          float step_deg)
{
    // 기본값: 정면
    float best_dir   = pose.yaw;
    float best_score = -1.0f;

    int num_steps = static_cast<int>(fov_deg / step_deg);

    for (int i = -num_steps; i <= num_steps; ++i) {
        float offset_deg = i * step_deg;
        float dir = pose.yaw + offset_deg * SLAM_PI / 180.0f;

        float free_dist = 0.0f;
        float step = og.res_m;  // 맵 해상도만큼 앞으로 전진하며 검사

        for (float r = 0.0f; r <= max_check_dist; r += step) {
            float wx = pose.x + r * std::cos(dir);
            float wy = pose.y + r * std::sin(dir);

            int gx, gy;
            if (!worldToGrid(wx, wy, og, gx, gy)) {
                // 맵 밖으로 나가면 여기까지만
                break;
            }

            int idx = gy * og.width + gx;
            int8_t cell = og.data[idx];

            if (cell == 100) {
                // 장애물 만나면 여기까지가 한계 거리
                break;
            }

            // free 혹은 unknown이면 일단 갈 수 있다고 봄
            free_dist = r;
        }

        // free_dist가 큰 방향이 더 "뚫려있다"
        if (free_dist > best_score) {
            best_score = free_dist;
            best_dir   = dir;
        }
    }

    return best_dir;  // world frame 기준 방향 (rad)
}

VelocityCommand computeVelocityCommand(const Pose2D& pose,
                                       float desired_dir,
                                       float max_speed,
                                       float max_turn_rate)
{
    VelocityCommand cmd;

    // 현재 heading과 목표 방향의 각도 차이
    float angle_error = normalizeAngle(desired_dir - pose.yaw);

    // 회전 속도: 각도 오차에 비례하게
    const float k_turn = 2.0f; // P 게인
    float omega = k_turn * angle_error;
    if (omega >  max_turn_rate) omega =  max_turn_rate;
    if (omega < -max_turn_rate) omega = -max_turn_rate;
    cmd.omega = omega;

    // 전진 속도: 각도 오차가 클수록 줄이고, 정면이면 max_speed
    float ang_abs = std::fabs(angle_error);
    // 예: ang_abs = 0 → 1.0, 90도(=pi/2) → 거의 0 근처
    float speed_scale = std::exp(-3.0f * ang_abs);  // 값은 튜닝 포인트
    cmd.v = max_speed * speed_scale;

    return cmd;
}
