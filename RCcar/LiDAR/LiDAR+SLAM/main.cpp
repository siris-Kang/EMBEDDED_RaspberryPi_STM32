#include <iostream>
#include <string>
#include <vector>
#include <cstdio>   // sprintf

#include "slam.hpp"

int main(int argc, char** argv) {
    std::string base_dir = "./data/"; 
    std::string ext      = ".txt";
    int num_steps = 20;

    // 글로벌 맵 생성 (10m x 10m, 5cm 해상도)
    float map_size_m = 10.0f;
    float res_m      = 0.05f;
    OccupancyGrid og = createGrid(map_size_m, res_m);

    float sensor_max_m = 6.0f; // 라이다 최대거리 (적당히)

    // 시작 포즈
    Pose2D pose{0.0f, 0.0f, 0.0f};

    for (int step = 1; step <= num_steps; ++step) {
        char fname[256];
        std::sprintf(fname, "%s%02d%s",
                     base_dir.c_str(), step, ext.c_str());
        std::string path = fname;

        std::cout << "=== STEP " << step << " / file: " << path << " ===\n";

        auto scans = loadScansFromTxt(path);
        if (scans.empty()) {
            std::cerr << "No scan in file: " << path << "\n";
            continue;
        }
        const Scan& scan = scans[0];

        // pose 추정 (scan-to-map refinement)
        if (step == 0) {
        } else {
            // 이전 포즈 근처에서만 찾는다고 가정(약간씩만 움직였으니까)
            Pose2D initial_guess = pose;

            Pose2D refined = refinePoseGridSearch(
                scan,
                og,
                initial_guess,
                sensor_max_m,
                0.20f,  // trans_range_m : ±20cm 범위 안에서 찾기
                0.02f,  // trans_step_m  : 2cm 단위로
                10.0f,  // rot_range_deg : ±10도 안에서
                1.0f    // rot_step_deg  : 1도 단위
            );
            pose = refined;
        }

        // 맵 업데이트
        updateWithScan(scan, og, pose, sensor_max_m);

        // 방향/속도 명령 계산 (UART로 쓸 예정)
        float desired_dir = chooseOpenDirection(
            og,
            pose,
            2.0f,   // max_check_dist : 2m 앞까지 확인
            90.0f,  // fov_deg        : 정면 기준 ±45도
            5.0f    // step_deg       : 5도 간격
        );

        VelocityCommand cmd = computeVelocityCommand(
            pose,
            desired_dir,
            0.3f,   // max_speed [m/s]
            1.0f    // max_turn_rate [rad/s]
        );

        std::cout << "  pose = (" << pose.x << ", "
                  << pose.y << ", " << pose.yaw << ")\n";
        std::cout << "  desired_dir(deg) = "
                  << (desired_dir * 180.0f / SLAM_PI)
                  << ", v = " << cmd.v
                  << " m/s, omega = " << cmd.omega << " rad/s\n";

        // PNG 저장
        cv::Mat img = gridToImage(og);
        char imgname[256];
        std::sprintf(imgname, "global_%02d.png", step);
        cv::imwrite(imgname, img);
    }

    // 최종 맵 한 장 더 저장
    cv::Mat final_img = gridToImage(og);
    cv::imwrite("global_final.png", final_img);
    std::cout << "Saved global_final.png\n";

    cv::imshow("global_final", final_img);
    cv::waitKey(0);

    return 0;
}
