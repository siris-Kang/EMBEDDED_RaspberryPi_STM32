#include <mosquitto.h>
#include <opencv2/opencv.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <random>
#include <iomanip>

#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

namespace fs = std::filesystem;

static std::atomic<bool> g_exit{false};

// 캡처 제어
static std::atomic<bool> g_cam_running{false};
static std::atomic<int>  g_period_ms{1000};
static std::mutex g_cam_mtx;
static std::condition_variable g_cam_cv;

// MQTT
static std::atomic<bool> g_mqtt_connected{false};
static mosquitto* g_mosq = nullptr;

static std::string g_mqtt_host = "127.0.0.1";
static int g_mqtt_port = 1883;
static std::string g_client_id = "rpi_cam_service";

// HTTP (python http.server) 프로세스 PID
static std::atomic<pid_t> g_http_pid{-1};
static int g_http_port = 8000;

// 저장 경로: ~/camera_images
static fs::path expandUser(const std::string& p) {
    if (!p.empty() && p[0] == '~') {
        const char* home = std::getenv("HOME");
        if (!home) home = "/home/ubuntu";
        if (p.size() == 1) return fs::path(home);
        if (p[1] == '/') return fs::path(home) / p.substr(2);
    }
    return fs::path(p);
}
static fs::path g_img_dir = expandUser("~/camera/camera_images");

// shot id
static std::atomic<uint64_t> g_shot_id{0};

static std::mt19937 rng{ std::random_device{}() };

static std::string makeShotId(uint64_t id) {
    std::ostringstream oss;
    oss << "shot_" << std::setw(4) << std::setfill('0') << id; // shot_0001
    return oss.str();
}

static std::string nowIsoUtcZ() {
    using namespace std::chrono;
    auto tp = system_clock::now();
    std::time_t t = system_clock::to_time_t(tp);
    std::tm tm{};
    gmtime_r(&t, &tm);
    char buf[64];
    std::snprintf(buf, sizeof(buf),
                  "%04d-%02d-%02dT%02d:%02d:%02dZ",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    return buf;
}

// 간단 JSON 파서(의존성 없이): "action":"start"/"stop", "period_ms":1234 추출
static std::optional<std::string> jsonGetString(const std::string& s, const std::string& key) {
    // naive: "key" : "value"
    std::string pat = "\"" + key + "\"";
    auto kpos = s.find(pat);
    if (kpos == std::string::npos) return std::nullopt;
    auto colon = s.find(':', kpos + pat.size());
    if (colon == std::string::npos) return std::nullopt;
    auto q1 = s.find('"', colon + 1);
    if (q1 == std::string::npos) return std::nullopt;
    auto q2 = s.find('"', q1 + 1);
    if (q2 == std::string::npos) return std::nullopt;
    return s.substr(q1 + 1, q2 - (q1 + 1));
}
static std::optional<int> jsonGetInt(const std::string& s, const std::string& key) {
    std::string pat = "\"" + key + "\"";
    auto kpos = s.find(pat);
    if (kpos == std::string::npos) return std::nullopt;
    auto colon = s.find(':', kpos + pat.size());
    if (colon == std::string::npos) return std::nullopt;

    // skip spaces
    size_t i = colon + 1;
    while (i < s.size() && (s[i] == ' ' || s[i] == '\t' || s[i] == '\n' || s[i] == '\r')) i++;

    // read number
    bool neg = false;
    if (i < s.size() && s[i] == '-') { neg = true; i++; }

    size_t j = i;
    while (j < s.size() && (s[j] >= '0' && s[j] <= '9')) j++;
    if (j == i) return std::nullopt;

    int v = std::stoi(s.substr(i, j - i));
    return neg ? -v : v;
}

// 내 IP(대충): hostname -I의 첫 IPv4 사용
static std::string getMyIpBestEffort() {
    FILE* fp = popen("hostname -I 2>/dev/null", "r");
    if (!fp) return "127.0.0.1";
    char buf[256]{0};
    std::fgets(buf, sizeof(buf), fp);
    pclose(fp);
    std::istringstream iss(buf);
    std::string ip;
    while (iss >> ip) {
        // loopback 제외, IPv4만
        if (ip.rfind("127.", 0) == 0) continue;
        if (ip.find(':') != std::string::npos) continue;
        return ip;
    }
    return "127.0.0.1";
}

static void ensureDir(const fs::path& p) {
    std::error_code ec;
    fs::create_directories(p, ec);
    if (ec) {
        std::cerr << "[ERR] mkdir failed: " << p << " : " << ec.message() << "\n";
    }
}

static void stopHttpServer() {
    pid_t pid = g_http_pid.load();
    if (pid > 0) {
        kill(pid, SIGTERM);
        int st = 0;
        waitpid(pid, &st, 0);
        g_http_pid.store(-1);
        std::cerr << "[HTTP] stopped pid=" << pid << "\n";
    }
}

static bool startHttpServer(const fs::path& dir, int port) {
    stopHttpServer();

    pid_t pid = fork();
    if (pid < 0) {
        std::cerr << "[ERR] fork failed for http server\n";
        return false;
    }
    if (pid == 0) {
        // child
        if (chdir(dir.c_str()) != 0) _exit(127);

        // python3 -m http.server <port> --bind 0.0.0.0
        execlp("python3", "python3", "-m", "http.server",
               std::to_string(port).c_str(),
               "--bind", "0.0.0.0",
               (char*)nullptr);
        _exit(127);
    }

    g_http_pid.store(pid);
    std::cerr << "[HTTP] serving " << dir << " on :" << port << " pid=" << pid << "\n";
    return true;
}

static void mqttPublishCameraShot(uint64_t shotNum, const std::string& imageUrl) {
    if (!g_mosq || !g_mqtt_connected.load()) return;

    std::uniform_real_distribution<double> dx(-1.0, 1.0);
    std::uniform_real_distribution<double> dy(-1.0, 1.0);
    std::uniform_real_distribution<double> dtheta(-3.14159, 3.14159);

    double x = dx(rng);
    double y = dy(rng);
    double theta = dtheta(rng);


    std::string shotId = makeShotId(shotNum);

    // (선택) 썸네일은 지금 없으니 image_url 그대로 넣거나 빈 문자열로
    std::string thumbUrl = imageUrl; // TODO: 나중에 진짜 thumb 만들면 교체

    std::ostringstream oss;
    oss << "{"
        << "\"shot_id\":\"" << shotId << "\","
        << "\"x\":" << x << ","
        << "\"y\":" << y << ","
        << "\"theta\":" << theta << ","
        << "\"image_url\":\"" << imageUrl << "\","
        << "\"thumbnail_url\":\"" << thumbUrl << "\","
        << "\"timestamp\":\"" << nowIsoUtcZ() << "\""
        << "}";

    std::string payload = oss.str();
    int rc = mosquitto_publish(g_mosq, nullptr,
                              "robot/camera_shot",
                              (int)payload.size(), payload.data(),
                              0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[MQTT] publish failed rc=" << rc << "\n";
    }
}


// USB 웹캠 캡처 스레드
static void cameraThreadMain() {
    cv::VideoCapture cap;
    bool opened = false;

    std::string myIp = getMyIpBestEffort();

    while (!g_exit.load()) {
        // 대기: start 될 때까지
        {
            std::unique_lock<std::mutex> lk(g_cam_mtx);
            g_cam_cv.wait(lk, []{
                return g_exit.load() || g_cam_running.load();
            });
        }
        if (g_exit.load()) break;

        // start 들어왔음: 카메라 열기
        if (!opened) {
            // 기본 0번 장치. 필요하면 /dev/video1 같은 걸로 바꿔도 됨.
            opened = cap.open(0, cv::CAP_V4L2);
            if (!opened) {
                std::cerr << "[CAM] open failed (device 0)\n";
                // 조금 쉬었다가 재시도
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            // 옵션: 해상도/포맷
            cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
            std::cerr << "[CAM] opened\n";
        }

        // running 동안 주기 촬영
        while (!g_exit.load() && g_cam_running.load()) {
            auto period = std::chrono::milliseconds(std::max(100, g_period_ms.load()));

            cv::Mat frame;
            if (!cap.read(frame) || frame.empty()) {
                std::cerr << "[CAM] read failed\n";
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }

            uint64_t id = ++g_shot_id;
            char fname[64];
            std::snprintf(fname, sizeof(fname), "shot_%06llu.jpg",
                          (unsigned long long)id);

            fs::path out = g_img_dir / fname;
            try {
                cv::imwrite(out.string(), frame);
            } catch (const std::exception& e) {
                std::cerr << "[CAM] imwrite exception: " << e.what() << "\n";
            }

            // image_url
            std::ostringstream url;
            url << "http://" << myIp << ":" << g_http_port << "/" << fname;

            mqttPublishCameraShot(id, url.str());

            // stop이 오면 즉시 깨우려고 cv로 wait
            {
                std::unique_lock<std::mutex> lk(g_cam_mtx);
                g_cam_cv.wait_for(lk, period, []{
                    return g_exit.load() || !g_cam_running.load();
                });
            }
        }

        // stop 들어왔으면 카메라 릴리즈(원하면 유지해도 됨)
        if (opened) {
            cap.release();
            opened = false;
            std::cerr << "[CAM] released\n";
        }
    }

    if (opened) cap.release();
}

// MQTT 콜백
static void on_connect(struct mosquitto*, void*, int rc) {
    if (rc == 0) {
        g_mqtt_connected.store(true);
        std::cerr << "[MQTT] connected\n";
        mosquitto_subscribe(g_mosq, nullptr, "robot/camera_cmd", 0);
    } else {
        std::cerr << "[MQTT] connect failed rc=" << rc << "\n";
    }
}

static void on_disconnect(struct mosquitto*, void*, int) {
    g_mqtt_connected.store(false);
    std::cerr << "[MQTT] disconnected\n";
}

static void on_message(struct mosquitto*, void*, const struct mosquitto_message* msg) {
    if (!msg || !msg->topic || !msg->payload) return;

    std::string topic = msg->topic;
    std::string payload((const char*)msg->payload, (size_t)msg->payloadlen);

    if (topic == "robot/camera_cmd") {
        auto action = jsonGetString(payload, "action");
        auto period = jsonGetInt(payload, "period_ms");

        if (action && *action == "start") {
            if (period) g_period_ms.store(std::clamp(*period, 100, 60000));
            g_cam_running.store(true);
            g_cam_cv.notify_all();
            std::cerr << "[CMD] camera start period_ms=" << g_period_ms.load() << "\n";
        } else if (action && *action == "stop") {
            g_cam_running.store(false);
            g_cam_cv.notify_all();
            std::cerr << "[CMD] camera stop\n";
        } else {
            std::cerr << "[CMD] unknown camera_cmd payload=" << payload << "\n";
        }
    }
}

static void handleSig(int) {
    g_exit.store(true);
    g_cam_running.store(false);
    g_cam_cv.notify_all();
}

int main(int argc, char** argv) {
    // args: <mqtt_host> <mqtt_port> <http_port>
    if (argc >= 2) g_mqtt_host = argv[1];
    if (argc >= 3) g_mqtt_port = std::atoi(argv[2]);
    if (argc >= 4) g_http_port = std::atoi(argv[3]);

    std::signal(SIGINT, handleSig);
    std::signal(SIGTERM, handleSig);

    ensureDir(g_img_dir);
    if (!startHttpServer(g_img_dir, g_http_port)) {
        std::cerr << "[ERR] failed to start http server\n";
        return 1;
    }

    mosquitto_lib_init();
    g_mosq = mosquitto_new(g_client_id.c_str(), true, nullptr);
    if (!g_mosq) {
        std::cerr << "[ERR] mosquitto_new failed\n";
        stopHttpServer();
        mosquitto_lib_cleanup();
        return 1;
    }

    mosquitto_connect_callback_set(g_mosq, on_connect);
    mosquitto_disconnect_callback_set(g_mosq, on_disconnect);
    mosquitto_message_callback_set(g_mosq, on_message);

    // 자동 재연결
    mosquitto_reconnect_delay_set(g_mosq, 1, 10, true);

    int rc = mosquitto_connect(g_mosq, g_mqtt_host.c_str(), g_mqtt_port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERR] mosquitto_connect failed rc=" << rc << "\n";
        mosquitto_destroy(g_mosq);
        stopHttpServer();
        mosquitto_lib_cleanup();
        return 1;
    }

    // mosquitto 네트워크 루프는 내부 스레드로
    mosquitto_loop_start(g_mosq);

    // 캡처 스레드
    std::thread camThread(cameraThreadMain);

    std::cerr << "[OK] rpi_camera_service running. img_dir=" << g_img_dir << "\n";
    while (!g_exit.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // 종료
    g_cam_running.store(false);
    g_cam_cv.notify_all();

    if (camThread.joinable()) camThread.join();

    mosquitto_loop_stop(g_mosq, true);
    mosquitto_disconnect(g_mosq);
    mosquitto_destroy(g_mosq);
    mosquitto_lib_cleanup();

    stopHttpServer();

    std::cerr << "[BYE] exit\n";
    return 0;
}
