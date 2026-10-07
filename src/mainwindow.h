#pragma once
#include "simulation.h"
#include <QMainWindow>
#include <QPainterPath>
#include <memory>
class QGraphicsScene;
class QGraphicsPathItem;
class QGraphicsEllipseItem;
class QTimer;
class QGraphicsSvgItem;
class QMediaPlayer;
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
    void updateAudio();
    std::unique_ptr<Ui::MainWindow> ui;
    navigation::Simulation simulation;
    bool hasStarted = false;
    bool audioSourceSet = false;
    QGraphicsScene *scene;
    QTimer *timer;
    QGraphicsPathItem *estimatedItem;
    QGraphicsPathItem *trueItem;
    QGraphicsPathItem *filteredItem;
    QGraphicsEllipseItem *estimatedMarker;
    QGraphicsEllipseItem *trueMarker;
    QGraphicsEllipseItem *filteredMarker;
    QGraphicsSvgItem *droneItem;
    QMediaPlayer *audioPlayer;
    QPainterPath estimatedPath;
    QPainterPath truePath;
    QPainterPath filteredPath;
};
