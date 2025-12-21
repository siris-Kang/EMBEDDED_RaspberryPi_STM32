#include "drawmap.h"

#include <QLabel>
#include <QEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPixmap>
#include <QDateTime>
#include <QMetaObject>

DrawMap::DrawMap(QLabel *mapLabel, QObject *parent)
    : QObject(parent), m_label(mapLabel)
{
    if (m_label) {
        m_label->installEventFilter(this);
        m_label->setAlignment(Qt::AlignCenter);
    }
}

void DrawMap::setMapMeta(const MapMeta& meta)
{
    m_currentMap = meta;
}

void DrawMap::setBaseMapImage(const QImage& img)
{
    m_baseMapImage = img;
}

void DrawMap::addShot(const Shot& shot)
{
    m_shots.push_back(shot);
}

void DrawMap::clearShots()
{
    m_shots.clear();
    m_hitboxes.clear();
}


QString DrawMap::nowHMS()
{
    return QDateTime::currentDateTime().toString("HH:mm:ss");
}

QPoint DrawMap::worldToPixel(double x, double y) const
{
    if (m_currentMap.resolution <= 0.0) return QPoint(-1, -1);
    if (m_baseMapImage.isNull()) return QPoint(-1, -1);

    // 1) 월드(m) -> 셀(cell)
    const double cx = (x - m_currentMap.originX) / m_currentMap.resolution;
    const double cy = (y - m_currentMap.originY) / m_currentMap.resolution;

    // 2) 셀(cell) -> 픽셀(px) 스케일 (이미지 크기와 셀 크기가 다를 때 보정)
    const int imgW = m_baseMapImage.width();
    const int imgH = m_baseMapImage.height();

    // width_cells/height_cells가 0이면 fallback (이미지=셀이라고 가정)
    const double gridW = (m_currentMap.widthCells  > 0) ? m_currentMap.widthCells  : imgW;
    const double gridH = (m_currentMap.heightCells > 0) ? m_currentMap.heightCells : imgH;

    const double scaleX = imgW / gridW;
    const double scaleY = imgH / gridH;

    const int px = int(cx * scaleX);
    const int py = int((gridH - 1.0 - cy) * scaleY);  // y 뒤집기 (맵 좌표계 ↔ 이미지 좌표계)

    return QPoint(px, py);
}

void DrawMap::render()
{
    if (!m_label)
        return;
    if (m_baseMapImage.isNull())
        return;

    QImage draw = m_baseMapImage.convertToFormat(QImage::Format_ARGB32);
    QPainter p(&draw);
    p.setRenderHint(QPainter::Antialiasing, true);

    m_hitboxes.clear();

    const int r = 4;

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(255, 0, 0, 255));

    for (int i = 0; i < m_shots.size(); ++i) {
        const Shot& s = m_shots[i];

        QPoint pt = worldToPixel(s.x, s.y);   // 기존 변환 함수 그대로 사용
        if (pt.x() < 0 || pt.y() < 0 || pt.x() >= draw.width() || pt.y() >= draw.height())
            continue;

        QRect rc(pt.x() - r, pt.y() - r, 2*r, 2*r);
        p.drawEllipse(rc);

        // hit 판정용 hitbox: "현재 화면에 그려진 것만" 저장
        m_hitboxes.push_back({rc, i});
    }

    // QLabel에 표시
    m_label->setPixmap(QPixmap::fromImage(draw).scaled(
        m_label->size(), Qt::KeepAspectRatio, Qt::FastTransformation));
}

bool DrawMap::labelPosToImagePos(const QPoint& labelPos, QPoint& outImgPos) const
{
    if (!m_label) return false;

    const QPixmap pm = m_label->pixmap(); // Qt6: 값 복사
    if (pm.isNull() || m_baseMapImage.isNull()) return false;

    const QSize lbl = m_label->size();
    const QSize pmSize = pm.size();

    const Qt::Alignment al = m_label->alignment();

    int offsetX = 0;
    int offsetY = 0;

    // 가로 정렬
    if (al & Qt::AlignHCenter) offsetX = (lbl.width() - pmSize.width()) / 2;
    else if (al & Qt::AlignRight) offsetX = (lbl.width() - pmSize.width());
    else offsetX = 0;

    // 세로 정렬
    if (al & Qt::AlignVCenter) offsetY = (lbl.height() - pmSize.height()) / 2;
    else if (al & Qt::AlignBottom) offsetY = (lbl.height() - pmSize.height());
    else offsetY = 0;

    const QRect pixRect(offsetX, offsetY, pmSize.width(), pmSize.height());
    if (!pixRect.contains(labelPos)) return false;

    const double sx = (labelPos.x() - pixRect.x()) / double(pixRect.width());
    const double sy = (labelPos.y() - pixRect.y()) / double(pixRect.height());

    const int ix = int(sx * m_baseMapImage.width());
    const int iy = int(sy * m_baseMapImage.height());

    outImgPos = QPoint(ix, iy);
    return true;
}

int DrawMap::findHitMarkerIndex(const QPoint& imgPos) const
{
    for (const auto& hb : m_hitboxes) {
        if (hb.first.contains(imgPos)) return hb.second;
    }
    return -1;
}

bool DrawMap::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == m_label && event->type() == QEvent::Resize) {
        // 레이아웃/리사이즈 직후에 다시 렌더(Queued로 안전하게)
        QMetaObject::invokeMethod(this, [this]() { this->render(); }, Qt::QueuedConnection);
        return false;
    }

    if (obj == m_label && event->type() == QEvent::MouseButtonPress) {
        auto *me = static_cast<QMouseEvent*>(event);

        QPoint imgPos;
        const bool ok = labelPosToImagePos(me->pos(), imgPos);

        emit logLine(QString("[%1] [click] label=(%2,%3) ok=%4 img=(%5,%6)")
                         .arg(nowHMS())
                         .arg(me->pos().x()).arg(me->pos().y())
                         .arg(ok ? "true" : "false")
                         .arg(imgPos.x()).arg(imgPos.y()));

        if (!ok) return true;

        const int idx = findHitMarkerIndex(imgPos);
        emit logLine(QString("[%1] [click] hit idx=%2").arg(nowHMS()).arg(idx));

        if (idx >= 0) {
            if (idx < m_shots.size()) {
                emit shotClicked(m_shots[idx]);
            } else {
                emit logLine(QString("[%1] [click] hit idx out of range: %2 (shots=%3)")
                                 .arg(nowHMS()).arg(idx).arg(m_shots.size()));
            }
        }
        return true;
    }
    return QObject::eventFilter(obj, event);
}

