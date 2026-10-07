#pragma once
#include <QGraphicsView>

class TrajectoryView : public QGraphicsView {
    Q_OBJECT
public:
    explicit TrajectoryView(QWidget *parent = nullptr);
    void setContentBounds(const QRectF &bounds);
    void setPositions(QPointF truth, QPointF estimate, QPointF filtered);
    bool autoOverview() const { return autoOverview_; }
    double gridStep() const;
public slots:
    void setAutoOverview(bool enabled);
signals:
    void autoOverviewChanged(bool enabled);
protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void drawForeground(QPainter *painter, const QRectF &rect) override;
    void wheelEvent(QWheelEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void showEvent(QShowEvent *event) override;
private:
    void zoomAt(double factor, const QPoint &anchor);
    void overview();
    void ensureRoom();
    QRectF contentBounds_{-20, -20, 640, 440};
    QPointF truth_{300, 200};
    QPointF estimate_{300, 200};
    QPointF filtered_{300, 200};
    bool autoOverview_ = true;
};
