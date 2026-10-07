#pragma once
#include "simulation.h"
#include <QMainWindow>
#include <QPainterPath>
#include <memory>
class QGraphicsScene;
class QGraphicsPathItem;
class QGraphicsEllipseItem;
class QTimer;
namespace Ui { class MainWindow; }
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;
private slots:
    void toggleRunning();
    void tick();
    void correct();
    void reset();
private:
    void redraw();
    void updateControls();
    void updateStatus(const QString &message = {});
    std::unique_ptr<Ui::MainWindow> ui;
    navigation::Simulation simulation;
    bool hasStarted = false;
    QGraphicsScene *scene;
    QTimer *timer;
    QGraphicsPathItem *estimatedItem;
    QGraphicsPathItem *trueItem;
    QGraphicsEllipseItem *estimatedMarker;
    QGraphicsEllipseItem *trueMarker;
    QPainterPath estimatedPath;
    QPainterPath truePath;
};
