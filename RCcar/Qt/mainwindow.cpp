#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "gaugewidget.h"
#include "mymqtt.h"
#include "drawmap.h"
#include "drawcamera.h"

#include <QDateTime>
#include <QHBoxLayout>
#include <QMessageBox>
#include <QPlainTextEdit>

#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QUrl>
#include <QSslError>

#include <QHash>
#include <QImage>

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QDir>
#include <QCoreApplication>

static QString nowHMS() {
    return QDateTime::currentDateTime().toString("HH:mm:ss");
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("Qt GUI");

    setupNetwork();
    setupUiWidgets();
    setupMqtt();
    setupConnections();

    loadRosMapFromYaml("C:/Users/SSAFY/Downloads/my_map.yaml");

    setUiStateIdle();
}

MainWindow::~MainWindow()
{
    delete m_imageCache;
    delete ui;
}

void MainWindow::setupNetwork()
{
    m_net = new QNetworkAccessManager(this);
    m_imageCache = new QHash<QString, QImage>();
}

void MainWindow::setupUiWidgets()
{
    // --- speed gauge ---
    m_speedGauge = new GaugeWidget(ui->veloGraph);
    m_speedGauge->setTitle("SPEED");
    m_speedGauge->setUnit("%");
    m_speedGauge->setRange(0, 100);
    m_speedGauge->setAnimated(true);

    auto *l = new QHBoxLayout(ui->veloGraph);
    l->setContentsMargins(8,8,8,8);
    l->addWidget(m_speedGauge);

    // --- map/camera helpers ---
    m_map = new DrawMap(ui->mapImage, this);
    m_camera = new DrawCamera(ui->cameraImage, this);

    // 라벨 기본 정렬
    ui->mapImage->setAlignment(Qt::AlignCenter);
    ui->cameraImage->setAlignment(Qt::AlignCenter);
}

void MainWindow::setupMqtt()
{
    m_mqtt = new MyMqtt(this);
}

void MainWindow::setupConnections()
{
    connect(m_mqtt, &MyMqtt::logLine, this, &MainWindow::appendLog);
    connect(m_map,  &DrawMap::logLine, this, &MainWindow::appendLog);

    // MQTT -> UI 반영
    connect(m_mqtt, &MyMqtt::motorCmdReceived, this, [this](const MotorCmd& cmd){
        m_lastSpeed = cmd.speed;
        m_lastSteer = cmd.steer;
        if (m_speedGauge) m_speedGauge->setValue(cmd.speed);
    });

    connect(m_mqtt, &MyMqtt::mapStateReceived, this, [this](const MapMeta& meta){
        m_currentMap = meta;
        m_map->setMapMeta(meta);

        if (meta.imageUrl.isEmpty()) {
            appendLog(QString("[%1] [map] empty image_url").arg(nowHMS()));
            return;
        }

        // 맵 이미지 다운로드 -> DrawMap에 base image로 내려주고 render
        fetchImage(meta.imageUrl, [this, meta](const QImage& img){
            if (img.isNull()) return;
            m_map->setBaseMapImage(img);
            m_map->render();
            appendLog(QString("[%1] [map] loaded %2 size=%3x%4")
                      .arg(nowHMS(), meta.imageUrl)
                      .arg(img.width()).arg(img.height()));
        });
    });

    connect(m_mqtt, &MyMqtt::cameraShotReceived, this, [this](const Shot& s){
        m_map->addShot(s);
        m_map->render();
    });

    // 맵 클릭 -> shot 선택 -> camera 이미지 다운로드 후 표시
    connect(m_map, &DrawMap::shotClicked, this, [this](const Shot& s){
        const QString toShow = !s.imageUrl.isEmpty() ? s.imageUrl : s.thumbUrl;
        if (toShow.isEmpty()) {
            appendLog(QString("[%1] [camera] empty url").arg(nowHMS()));
            return;
        }

        appendLog(QString("[%1] [camera] downloading: %2").arg(nowHMS(), toShow));

        fetchImage(toShow, [this, toShow](const QImage& img){
            if (img.isNull()) {
                appendLog(QString("[%1] [camera] decode fail: %2").arg(nowHMS(), toShow));
                return;
            }
            m_camera->setImage(img);
            appendLog(QString("[%1] [camera] show: %2").arg(nowHMS(), toShow));
        });
    });
}

void MainWindow::appendLog(const QString& line)
{
    if (ui->rccarLogs) ui->rccarLogs->appendPlainText(line);
}

void MainWindow::onStartClicked()
{
    statusBar()->showMessage("MQTT START", 1500);
    setUiStateRunning();

    if (!m_mqtt)
        return;

    if (m_mqtt->isConnected()) {
        // 이미 연결이면 바로 publish
        m_mqtt->publishMotorCmd(m_lastSpeed, m_lastSteer, true, false, "remote");
        return;
    }

    // 연결 성공 직후 1회만 publish
    connect(m_mqtt, &MyMqtt::connected, this, [this]() {
        appendLog(QString("[%1] [UI] MQTT connected -> send enable").arg(nowHMS()));
        m_mqtt->publishMotorCmd(m_lastSpeed, m_lastSteer, true, false, "remote");
    }, Qt::SingleShotConnection);

    // 이제서야 연결
    m_mqtt->connectToBroker("10.252.52.29", 1883, "qt_gui");
}


void MainWindow::onStopClicked()
{
    statusBar()->showMessage("MQTT STOP", 1500);
    setUiStateStopped();

    if (!m_mqtt) return;

    // 연결되어 있으면 먼저 e-stop 한번 보내기
    if (m_mqtt->isConnected()) {
        m_mqtt->publishMotorCmd(0, m_lastSteer, true, true, "remote");
    }

    // 끊기
    m_mqtt->disconnectFromBroker();
}


void MainWindow::onFinishClicked()
{
    statusBar()->showMessage("MQTT FINISH", 1500);

    // enable off
    m_mqtt->publishMotorCmd(0, m_lastSteer, false, false, "remote");
}

void MainWindow::setUiStateIdle()
{
    ui->startBtn->setEnabled(true);
    ui->stopBtn->setEnabled(false);
    ui->finishBtn->setEnabled(true);
}

void MainWindow::setUiStateRunning()
{
    ui->startBtn->setEnabled(false);
    ui->stopBtn->setEnabled(true);
    ui->finishBtn->setEnabled(true);
}

void MainWindow::setUiStateStopped()
{
    ui->startBtn->setEnabled(true);
    ui->stopBtn->setEnabled(false);
    ui->finishBtn->setEnabled(true);
}

void MainWindow::fetchImage(const QString& url, std::function<void(const QImage&)> onOk)
{
    if (url.isEmpty()) {
        appendLog(QString("[%1] [http] empty url").arg(nowHMS()));
        return;
    }

    // 캐시 hit
    if (m_imageCache && m_imageCache->contains(url)) {
        onOk((*m_imageCache)[url]);
        return;
    }

    QUrl qurl(url);
    if (!qurl.isValid()) {
        appendLog(QString("[%1] [http] invalid url: %2").arg(nowHMS(), url));
        return;
    }

    QNetworkRequest req(qurl);
    req.setHeader(QNetworkRequest::UserAgentHeader, "qt_gui");

    // Qt6: RedirectPolicyAttribute 사용 (FollowRedirectsAttribute는 Qt5/상황에 따라 없을 수 있음)
    req.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                     QNetworkRequest::NoLessSafeRedirectPolicy);

    QNetworkReply* reply = m_net->get(req);

    connect(reply, &QNetworkReply::sslErrors, this, [this](const QList<QSslError>& errors){
        for (const auto& e : errors) {
            appendLog(QString("[%1] [http][ssl] %2").arg(nowHMS(), e.errorString()));
        }
    });

    connect(reply, &QNetworkReply::finished, this, [this, reply, url, onOk]() {
        const auto err = reply->error();
        const QByteArray data = reply->readAll();
        reply->deleteLater();

        if (err != QNetworkReply::NoError) {
            appendLog(QString("[%1] [http] GET fail url=%2 err=%3").arg(nowHMS(), url).arg(int(err)));
            return;
        }

        QImage img;
        if (!img.loadFromData(data)) {
            appendLog(QString("[%1] [http] decode fail url=%2 bytes=%3").arg(nowHMS(), url).arg(data.size()));
            return;
        }

        if (m_imageCache) m_imageCache->insert(url, img);
        onOk(img);
    });
}

static QString stripQuotes(QString s)
{
    s = s.trimmed();
    if (s.size() >= 2) {
        if ((s.front() == '"'  && s.back() == '"') ||
            (s.front() == '\'' && s.back() == '\'')) {
            s = s.mid(1, s.size() - 2);
        }
    }
    return s.trimmed();
}

static bool parseBracketList3(const QString& s, double& a, double& b, double& c)
{
    // origin: [x, y, yaw]
    QString t = s.trimmed();
    if (!t.startsWith('[') || !t.endsWith(']')) return false;
    t = t.mid(1, t.size() - 2);

    const auto parts = t.split(',', Qt::SkipEmptyParts);
    if (parts.size() < 3) return false;

    bool ok1=false, ok2=false, ok3=false;
    a = parts[0].trimmed().toDouble(&ok1);
    b = parts[1].trimmed().toDouble(&ok2);
    c = parts[2].trimmed().toDouble(&ok3);
    return ok1 && ok2 && ok3;
}

bool MainWindow::loadRosMapYamlInternal(const QString& yamlPath, MapMeta& outMeta, QImage& outImg)
{
    QFile f(yamlPath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text)) {
        appendLog(QString("[%1] [map] open fail: %2").arg(nowHMS(), yamlPath));
        return false;
    }

    QFileInfo yfi(yamlPath);
    const QString yamlDir = yfi.absolutePath();

    // 기본값
    QString imageRel;
    double resolution = 0.05;
    double ox=0.0, oy=0.0, oyaw=0.0;
    int negate = 0;

    QTextStream ts(&f);
    while (!ts.atEnd()) {
        QString line = ts.readLine();

        // 주석 제거
        const int hash = line.indexOf('#');
        if (hash >= 0) line = line.left(hash);

        line = line.trimmed();
        if (line.isEmpty()) continue;

        const int colon = line.indexOf(':');
        if (colon < 0) continue;

        const QString key = line.left(colon).trimmed();
        QString val = line.mid(colon + 1).trimmed();
        val = stripQuotes(val);

        if (key == "image") {
            imageRel = val;
        } else if (key == "resolution") {
            bool ok=false;
            const double r = val.toDouble(&ok);
            if (ok) resolution = r;
        } else if (key == "origin") {
            double a,b,c;
            if (parseBracketList3(val, a,b,c)) {
                ox=a; oy=b; oyaw=c;
            }
        } else if (key == "negate") {
            bool ok=false;
            const int n = val.toInt(&ok);
            if (ok) negate = n;
        }
    }

    if (imageRel.isEmpty()) {
        appendLog(QString("[%1] [map] yaml has no image field").arg(nowHMS()));
        return false;
    }

    // image 경로는 yaml 기준 상대경로가 대부분
    const QString imagePath = QDir(yamlDir).absoluteFilePath(imageRel);

    QImage img;
    if (!img.load(imagePath)) {
        appendLog(QString("[%1] [map] image load fail: %2").arg(nowHMS(), imagePath));
        return false;
    }

    // ✅ “gray map”로 쓰기 위해 grayscale로 통일
    img = img.convertToFormat(QImage::Format_Grayscale8);

    // negate: 1이면 색 반전
    if (negate == 1) {
        img.invertPixels();
    }

    MapMeta meta;
    meta.mapId = QFileInfo(yamlPath).baseName();
    meta.widthCells = img.width();
    meta.heightCells = img.height();
    meta.resolution = resolution;
    meta.originX = ox;
    meta.originY = oy;
    meta.originYaw = oyaw;

    // 지금은 MQTT 안 쓰니까, imageUrl에는 “참고용”으로 로컬 경로를 넣어도 됨
    meta.imageUrl = imagePath;
    meta.timestamp = QDateTime::currentDateTimeUtc().toString(Qt::ISODate) + "Z";

    outMeta = meta;
    outImg = img;
    return true;
}

bool MainWindow::loadRosMapFromYaml(const QString& yamlPath)
{
    MapMeta meta;
    QImage img;

    if (!loadRosMapYamlInternal(yamlPath, meta, img)) {
        return false;
    }

    m_currentMap = meta;
    m_map->setMapMeta(meta);
    m_map->setBaseMapImage(img);
    m_map->render();

    appendLog(QString("[%1] [map] loaded yaml=%2 img=%3 (%4x%5) res=%6 origin=(%7,%8,%9)")
                  .arg(nowHMS(), yamlPath, meta.imageUrl)
                  .arg(img.width()).arg(img.height())
                  .arg(meta.resolution, 0, 'f', 3)
                  .arg(meta.originX, 0, 'f', 3)
                  .arg(meta.originY, 0, 'f', 3)
                  .arg(meta.originYaw, 0, 'f', 3));
    return true;
}

