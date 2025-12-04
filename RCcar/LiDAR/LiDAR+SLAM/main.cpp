#include <iostream>
#include <string>

#include "slam.hpp"

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

    // 1. 글로벌 맵 생성 (10m x 10m, 5cm resolution)
    float map_size_m = 10.0f;
    float res_m      = 0.05f;
    OccupancyGrid og = createGrid(map_size_m, res_m);

    float sensor_max_m = 6.0f; // 라이다 최대 거리 (대충 값)

    // 2. 시작 포즈 (0,0,0) 에서 첫 스캔 반영
    Pose2D pose{0.0f, 0.0f, 0.0f};
    updateWithScan(scans[0], og, pose, sensor_max_m);

    // 3. 테스트:
    //    스캔이 1개뿐이므로, 로봇이 x방향으로 직선 이동한다고 가정
    //    (진짜 SLAM이라기보다 파이프라인 테스트용)
    int num_steps = 20;
    float step_dist = 0.1f; // 10cm씩 전진 가정

    for (int i = 1; i <= num_steps; ++i) {
        // ---- (A) 가짜 오도메트리로 전진 가정 ----
        Pose2D odom_guess = pose;
        odom_guess.x += step_dist;

        // ---- (B) scan-to-map 매칭으로 미세 보정 ----
        Pose2D refined = refinePoseGridSearch(
            scans[0], og, odom_guess, sensor_max_m,
            0.10f,  // trans_range_m : ±10cm
            0.02f,  // trans_step_m  : 2cm
            5.0f,   // rot_range_deg : ±5도
            1.0f    // rot_step_deg  : 1도
        );
        pose = refined;

        // ---- (C) 이 포즈에서 스캔 관측했다고 보고 맵 업데이트 ----
        updateWithScan(scans[0], og, pose, sensor_max_m);

        // ---- (D) 현재 맵 기준으로 "어디로 가면 좋을지" 방향 결정 ----
        float desired_dir = chooseOpenDirection(
            og,
            pose,
            2.0f,   // max_check_dist : 2m 앞까지 확인
            90.0f,  // fov_deg        : 정면 기준 ±45도
            5.0f    // step_deg       : 5도 간격
        );

        // ---- (E) 그 방향으로 가기 위한 속도/조향 명령 계산 ----
        VelocityCommand cmd = computeVelocityCommand(
            pose,
            desired_dir,
            0.3f,   // max_speed [m/s] (시뮬레이션용 상수)
            1.0f    // max_turn_rate [rad/s]
        );

        std::cout << "Step " << i
                << " pose = (" << pose.x << ", "
                << pose.y << ", " << pose.yaw << ")\n";

        std::cout << "  desired_dir(deg) = "
                << (desired_dir * 180.0f / SLAM_PI)
                << ", v = " << cmd.v
                << " m/s, omega = " << cmd.omega << " rad/s\n";

        // 중간 맵 이미지 저장하고 싶으면 그대로 두거나 주석 해제
        /*
        cv::Mat step_img = gridToImage(og);
        std::string fname = "map_step_" + std::to_string(i) + ".png";
        cv::imwrite(fname, step_img);
        */
    }

    // 4. 최종 맵 저장
    cv::Mat img = gridToImage(og);
    cv::imwrite("map_global.png", img);
    std::cout << "Saved map_global.png\n";

    // GUI 있을 때만
    cv::imshow("map_global", img);
    cv::waitKey(0);

    return 0;
}
