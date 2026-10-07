#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QTimer>
#include <cmath>

namespace {
QPointF point(navigation::Vector2 value) { return {value.x, value.y}; }
bool readAcceleration(QLineEdit *edit, double &value) {
    QString input = edit->text().trimmed();
    input.replace(',', '.');
    bool ok = false;
    value = input.toDouble(&ok);
    return ok && std::isfinite(value) && std::abs(value) <= 100.0;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(std::make_unique<Ui::MainWindow>()),
      scene(new QGraphicsScene(this)), timer(new QTimer(this)) {
    ui->setupUi(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    // Cosmetic pens keep both trajectories readable as the view zooms out.
    QPen green(QColor(0, 150, 50), 3);
    green.setCosmetic(true);
    QPen blue(QColor(30, 80, 230), 2);
    blue.setCosmetic(true);
    trueItem = scene->addPath({}, green);
    estimatedItem = scene->addPath({}, blue);
    trueMarker = scene->addEllipse(-5, -5, 10, 10, green, Qt::green);
    estimatedMarker = scene->addEllipse(-3, -3, 6, 6, blue, Qt::blue);
    trueMarker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    estimatedMarker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    timer->setObjectName("simulationTimer");
    timer->setInterval(50);
    timer->setTimerType(Qt::PreciseTimer);
    connect(timer, &QTimer::timeout, this, &MainWindow::tick);
    connect(ui->btnStart, &QPushButton::clicked, this, &MainWindow::toggleRunning);
    connect(ui->btnCorrect, &QPushButton::clicked, this, &MainWindow::correct);
    connect(ui->btnReset, &QPushButton::clicked, this, &MainWindow::reset);
    connect(ui->comboMotion, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this] { updateControls(); });
    connect(ui->checkAutoFollow, &QCheckBox::toggled, ui->graphicsView, &TrajectoryView::setAutoOverview);
    connect(ui->graphicsView, &TrajectoryView::autoOverviewChanged, ui->checkAutoFollow, &QCheckBox::setChecked);
    reset();
}
MainWindow::~MainWindow() = default;

void MainWindow::updateControls() {
    const bool circle = ui->comboMotion->currentIndex() == 1;
    const bool setup = !hasStarted;
    ui->comboMotion->setEnabled(setup);
    ui->spinRadius->setEnabled(setup);
    ui->spinPeriod->setEnabled(setup);
    ui->spinRadius->setVisible(circle);
    ui->spinPeriod->setVisible(circle);
    ui->labelRadius->setVisible(circle);
    ui->labelPeriod->setVisible(circle);
    ui->lineEditAx->setVisible(!circle);
    ui->lineEditAy->setVisible(!circle);
    ui->labelAx->setVisible(!circle);
    ui->labelAy->setVisible(!circle);
    ui->lineEditAx->setEnabled(!timer->isActive());
    ui->lineEditAy->setEnabled(!timer->isActive());
}

void MainWindow::toggleRunning() {
    if (timer->isActive()) {
        timer->stop();
        ui->btnStart->setText(tr("Продолжить"));
        updateControls();
        updateStatus(tr("Пауза"));
        return;
    }
    if (ui->comboMotion->currentIndex() == 1) {
        if (!hasStarted)
            simulation.configureCircle(ui->spinRadius->value(), ui->spinPeriod->value());
    } else {
        navigation::Vector2 acceleration;
        QLineEdit *invalid = nullptr;
        if (!readAcceleration(ui->lineEditAx, acceleration.x)) invalid = ui->lineEditAx;
        else if (!readAcceleration(ui->lineEditAy, acceleration.y)) invalid = ui->lineEditAy;
        if (invalid) {
            updateStatus(tr("Введите ускорения от −100 до 100 м/с² (точка или запятая)."));
            invalid->setFocus();
            invalid->selectAll();
            return;
        }
        simulation.start(acceleration);
    }
    hasStarted = true;
    timer->start();
    ui->btnStart->setText(tr("Пауза"));
    updateControls();
    updateStatus();
}
void MainWindow::tick() {
    if (!timer->isActive()) return;
    simulation.step();
    truePath.lineTo(point(simulation.state().truePosition));
    estimatedPath.lineTo(point(simulation.state().estimatedPosition));
    redraw();
    updateStatus();
}
void MainWindow::correct() {
    simulation.correct();
    // A correction is not physical motion: break the estimated trail instead of
    // drawing a spurious flight segment between the old and corrected positions.
    estimatedPath.moveTo(point(simulation.state().estimatedPosition));
    redraw();
    updateStatus(tr("Коррекция выполнена; оценённая скорость обнулена"));
}
void MainWindow::reset() {
    timer->stop();
    simulation.reset();
    hasStarted = false;
    truePath = QPainterPath(point(simulation.state().truePosition));
    estimatedPath = QPainterPath(point(simulation.state().estimatedPosition));
    ui->btnStart->setText(tr("Старт"));
    updateControls();
    ui->graphicsView->setAutoOverview(true);
    redraw();
    updateStatus(tr("Готово к запуску"));
}
void MainWindow::redraw() {
    trueItem->setPath(truePath);
    estimatedItem->setPath(estimatedPath);
    trueMarker->setPos(point(simulation.state().truePosition));
    estimatedMarker->setPos(point(simulation.state().estimatedPosition));
    const QRectF bounds = QRectF(0, 0, 600, 400)
        .united(truePath.boundingRect()).united(estimatedPath.boundingRect()).adjusted(-20, -20, 20, 20);
    scene->setSceneRect(bounds);
    ui->graphicsView->setContentBounds(bounds);
    ui->graphicsView->setPositions(trueMarker->pos(), estimatedMarker->pos());
}
void MainWindow::updateStatus(const QString &message) {
    const auto &s = simulation.state();
    const auto acceleration = simulation.acceleration();
    ui->labelStatus->setText(tr("%1\nВремя: %2 с\nОшибка: %3 м\nИстина: (%4; %5) м\nОценка: (%6; %7) м\na: (%8; %9) м/с²")
        .arg(message.isEmpty() ? (timer->isActive() ? tr("GPS недоступен · ИНС") : tr("Пауза")) : message)
        .arg(s.time, 0, 'f', 2).arg(simulation.error(), 0, 'f', 3)
        .arg(s.truePosition.x, 0, 'f', 2).arg(s.truePosition.y, 0, 'f', 2)
        .arg(s.estimatedPosition.x, 0, 'f', 2).arg(s.estimatedPosition.y, 0, 'f', 2)
        .arg(acceleration.x, 0, 'f', 3).arg(acceleration.y, 0, 'f', 3));
}
