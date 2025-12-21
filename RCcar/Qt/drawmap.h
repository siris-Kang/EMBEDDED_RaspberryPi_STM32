#pragma once
/**
 * @file drawmap.h
 * @brief "맵 이미지 + shot 마커" 렌더링과 클릭 히트테스트 담당 (UI 독립, QLabel만 붙이면 됨)
 *
 * - 맵 이미지(QImage)는 외부(MainWindow)가 내려준다. (HTTP 다운로드 등은 여기서 안함)
 * - QLabel(mapLabel)에 eventFilter를 걸어서 클릭 -> shotClicked 신호를 내보낸다.
 */

#include <QObject>
#include <QImage>
#include <QHash>
#include <QVector>
#include <QRect>
#include <QPair>

#include "app_types.h"

class QLabel;

class DrawMap final : public QObject
{
    Q_OBJECT
public:
    explicit DrawMap(QLabel *mapLabel, QObject *parent = nullptr);

    void setMapMeta(const MapMeta& meta);
    void setBaseMapImage(const QImage& img);

    void addShot(const Shot& shot);
    void clearShots();

    void render(); // 현재 map_meta + baseMapImage 기준으로 mapLabel에 그린다.

signals:
    void logLine(const QString& line);
    void shotClicked(const Shot& shot);

protected:
    bool eventFilter(QObject *obj, QEvent *event) override;

private:
    QLabel *m_label = nullptr;

    MapMeta m_currentMap;
    QImage  m_baseMapImage;

    QVector<Shot> m_shots;

    // 클릭 판정용: (이미지 좌표계 hitbox, shotIndex)
    QVector<QPair<QRect, int>> m_hitboxes;

    QPoint worldToPixel(double x, double y) const;

    // QLabel의 마우스 좌표 -> "원본 이미지" 좌표로 변환
    bool labelPosToImagePos(const QPoint& labelPos, QPoint& outImgPos) const;

    int findHitMarkerIndex(const QPoint& imgPos) const;

    static QString nowHMS();
};
