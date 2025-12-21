#pragma once
/**
 * @file mainwindow.h
 * @brief 앱 "조립" 담당: UI + MyMqtt + DrawMap + 네트워크 이미지 다운로드 연결
 *
 * ✅ 헤더 원칙:
 *  - 다른 헤더를 최대한 적게 include
 *  - 무거운 Qt 네트워크/JSON 헤더는 cpp에서만 include
 */

#include <QMainWindow>
#include <functional>

#include "app_types.h"

// forward decl (헤더에서 무거운 include 줄이기)
class QImage;
template <class K, class V> class QHash;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class GaugeWidget;
class MyMqtt;
class DrawMap;
class DrawCamera;
class QNetworkAccessManager;

class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

    bool loadRosMapFromYaml(const QString& yamlPath);

private slots:
    void onStartClicked();
    void onStopClicked();
    void onFinishClicked();

private:
    Ui::MainWindow *ui = nullptr;

    GaugeWidget *m_speedGauge = nullptr;

    MyMqtt *m_mqtt = nullptr;
    DrawMap *m_map = nullptr;
    DrawCamera *m_camera = nullptr;

    // 마지막으로 받은(또는 보낸) 조작값
    int m_lastSpeed = 0;   // 0~100
    int m_lastSteer = 90;  // 0~180

    // HTTP 이미지 다운로드
    QNetworkAccessManager *m_net = nullptr;
    // url -> image cache (동일 URL 반복 다운 방지)
    QHash<QString, QImage> *m_imageCache = nullptr; // cpp에서 생성/해제

    MapMeta m_currentMap;

    void setupUiWidgets();
    void setupConnections();
    void setupNetwork();
    void setupMqtt();

    void appendLog(const QString& line);

    void setUiStateIdle();
    void setUiStateRunning();
    void setUiStateStopped();

    // url을 받아서 QImage를 콜백으로 전달
    void fetchImage(const QString& url, std::function<void(const QImage&)> onOk);
    bool loadRosMapYamlInternal(const QString& yamlPath, MapMeta& outMeta, QImage& outImg);
};
