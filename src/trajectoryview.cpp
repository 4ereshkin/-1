#include "trajectoryview.h"
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QScrollBar>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>

TrajectoryView::TrajectoryView(QWidget *parent) : QGraphicsView(parent) {
    setRenderHint(QPainter::Antialiasing);
    setDragMode(QGraphicsView::ScrollHandDrag);
    setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    setTransformationAnchor(QGraphicsView::NoAnchor);
    setResizeAnchor(QGraphicsView::AnchorViewCenter);
    setFocusPolicy(Qt::StrongFocus);
    setViewportUpdateMode(QGraphicsView::FullViewportUpdate);
}
void TrajectoryView::setPositions(QPointF truth, QPointF estimate, QPointF filtered) {
    truth_ = truth;
    estimate_ = estimate;
    filtered_ = filtered;
    viewport()->update();
}
void TrajectoryView::ensureRoom() {
    const QRectF visible = mapToScene(viewport()->rect()).boundingRect();
    const QRectF room = visible.united(contentBounds_);
    if (sceneRect().contains(room.adjusted(-visible.width()/2, -visible.height()/2,
                                           visible.width()/2, visible.height()/2))) return;
    // Preserve scroll offsets exactly: repeatedly centerOn(mapToScene(center))
    // would accumulate pixel-rounding error and move a paused manual camera.
    const int horizontal = horizontalScrollBar()->value();
    const int vertical = verticalScrollBar()->value();
    setSceneRect(room.adjusted(-room.width(), -room.height(), room.width(), room.height()));
    horizontalScrollBar()->setValue(horizontal);
    verticalScrollBar()->setValue(vertical);
}
void TrajectoryView::setContentBounds(const QRectF &bounds) {
    contentBounds_ = bounds;
    ensureRoom();
    if (autoOverview_) overview();
}
void TrajectoryView::overview() {
    if (!scene()) return;
    fitInView(contentBounds_, Qt::KeepAspectRatio);
    centerOn(contentBounds_.center());
    viewport()->update();
}
void TrajectoryView::setAutoOverview(bool enabled) {
    const bool changed = autoOverview_ != enabled;
    autoOverview_ = enabled;
    if (enabled) overview();
    if (changed) emit autoOverviewChanged(enabled);
    viewport()->update();
}
double TrajectoryView::gridStep() const {
    const double target = 85.0 / transform().m11();
    const double power = std::pow(10.0, std::floor(std::log10(target)));
    const double normalized = target / power;
    return (normalized <= 1 ? 1 : normalized <= 2 ? 2 : normalized <= 5 ? 5 : 10) * power;
}
void TrajectoryView::drawBackground(QPainter *painter, const QRectF &rect) {
    painter->save();
    painter->fillRect(rect, QColor(249, 251, 253));
    painter->setRenderHint(QPainter::Antialiasing, false);
    const double step = gridStep();
    QPen grid(QColor(218, 226, 235), 0);
    painter->setPen(grid);
    for (double x = std::ceil(rect.left()/step)*step; x <= rect.right(); x += step)
        painter->drawLine(QPointF(x, rect.top()), QPointF(x, rect.bottom()));
    for (double y = std::ceil(rect.top()/step)*step; y <= rect.bottom(); y += step)
        painter->drawLine(QPointF(rect.left(), y), QPointF(rect.right(), y));
    painter->setPen(QPen(QColor(130, 148, 168), 0));
    painter->drawLine(QPointF(0, rect.top()), QPointF(0, rect.bottom()));
    painter->drawLine(QPointF(rect.left(), 0), QPointF(rect.right(), 0));
    painter->restore();
}
void TrajectoryView::drawForeground(QPainter *painter, const QRectF &) {
    painter->save();
    painter->resetTransform();
    const QRect screen = viewport()->rect();
    const QRectF visible = mapToScene(screen).boundingRect();
    const double step = gridStep();
    painter->setPen(QColor(74, 92, 115));
    QFont small = font();
    small.setPointSize(9);
    painter->setFont(small);
    for (double x = std::ceil(visible.left()/step)*step; x <= visible.right(); x += step) {
        const int px = mapFromScene(QPointF(x, 0)).x();
        painter->drawText(QRect(px+4, screen.height()-24, 85, 20), Qt::AlignLeft, QString::number(x == 0 ? 0 : x, 'g', 6));
    }
    for (double y = std::ceil(visible.top()/step)*step; y <= visible.bottom(); y += step) {
        const int py = mapFromScene(QPointF(0, y)).y();
        painter->drawText(QRect(5, py+2, 80, 20), Qt::AlignLeft, QString::number(y == 0 ? 0 : y, 'g', 6));
    }
    auto label = [&](QPointF position, const QString &text, QColor color, int offset) {
        const QPoint anchor = mapFromScene(position);
        if (!screen.contains(anchor)) return;
        QRect box(anchor.x()+10, anchor.y()+offset, 94, 23);
        box.moveLeft(std::clamp(box.left(), 2, std::max(2, screen.width()-box.width()-2)));
        box.moveTop(std::clamp(box.top(), 2, std::max(2, screen.height()-box.height()-2)));
        painter->setPen(Qt::NoPen);
        painter->setBrush(QColor(255, 255, 255, 215));
        painter->drawRoundedRect(box, 4, 4);
        painter->setPen(color);
        painter->drawText(box.adjusted(5, 0, -2, 0), Qt::AlignVCenter, text);
    };
    label(truth_, tr("Истина"), QColor(0, 125, 50), -49);
    label(estimate_, tr("Оценка ИНС"), QColor(30, 80, 230), 7);
    label(filtered_, tr("Фильтр"), QColor(175, 80, 0), -75);
    painter->restore();
}
void TrajectoryView::zoomAt(double factor, const QPoint &anchor) {
    setAutoOverview(false);
    const QPointF before = mapToScene(anchor);
    const double current = transform().m11();
    const double desired = std::clamp(current * factor, 0.00001, 100.0);
    scale(desired/current, desired/current);
    ensureRoom();
    const QPointF after = mapToScene(anchor);
    centerOn(mapToScene(viewport()->rect().center()) + before-after);
    viewport()->update();
}
void TrajectoryView::wheelEvent(QWheelEvent *event) {
    const double delta = event->angleDelta().y() != 0 ? event->angleDelta().y() : event->pixelDelta().y();
    if (delta != 0) zoomAt(std::pow(1.0015, delta), event->position().toPoint());
    event->accept();
}
void TrajectoryView::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton) {
        setAutoOverview(false);
        ensureRoom();
    }
    QGraphicsView::mousePressEvent(event);
}
void TrajectoryView::keyPressEvent(QKeyEvent *event) {
    if (event->key() == Qt::Key_F) setAutoOverview(true);
    else if (event->key() == Qt::Key_Plus || event->key() == Qt::Key_Equal)
        zoomAt(1.2, viewport()->rect().center());
    else if (event->key() == Qt::Key_Minus) zoomAt(1.0/1.2, viewport()->rect().center());
    else if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right ||
             event->key() == Qt::Key_Up || event->key() == Qt::Key_Down) {
        setAutoOverview(false);
        ensureRoom();
        const double distance = 40.0/transform().m11();
        QPointF offset;
        if (event->key() == Qt::Key_Left) offset.setX(-distance);
        if (event->key() == Qt::Key_Right) offset.setX(distance);
        if (event->key() == Qt::Key_Up) offset.setY(-distance);
        if (event->key() == Qt::Key_Down) offset.setY(distance);
        centerOn(mapToScene(viewport()->rect().center())+offset);
    } else { QGraphicsView::keyPressEvent(event); return; }
    event->accept();
}
void TrajectoryView::resizeEvent(QResizeEvent *event) {
    QGraphicsView::resizeEvent(event);
    if (autoOverview_) overview();
}
void TrajectoryView::showEvent(QShowEvent *event) {
    QGraphicsView::showEvent(event);
    if (autoOverview_) overview();
}
