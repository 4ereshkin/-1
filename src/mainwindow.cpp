#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QGraphicsEllipseItem>
#include <QGraphicsPathItem>
#include <QGraphicsScene>
#include <QTimer>
#include <QGraphicsSvgItem>
#include <QMediaPlayer>
#include <QUrl>
#include <algorithm>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QAudioOutput>
#else
#include <QMediaContent>
#endif
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
    Q_INIT_RESOURCE(navigation_assets);
    ui->setupUi(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    // Cosmetic pens keep both trajectories readable as the view zooms out.
    QPen green(QColor(0, 150, 50), 3);
    green.setCosmetic(true);
    QPen blue(QColor(30, 80, 230), 2);
    blue.setCosmetic(true);
    QPen orange(QColor(215, 105, 0), 2);
    orange.setCosmetic(true);
    trueItem = scene->addPath({}, green);
    estimatedItem = scene->addPath({}, blue);
    filteredItem = scene->addPath({}, orange);
    trueMarker = scene->addEllipse(-5, -5, 10, 10, green, Qt::green);
    estimatedMarker = scene->addEllipse(-3, -3, 6, 6, blue, Qt::blue);
    filteredMarker = scene->addEllipse(-4, -4, 8, 8, orange, orange.color());
    trueMarker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    estimatedMarker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    filteredMarker->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    droneItem = new QGraphicsSvgItem(QStringLiteral(":/assets/drone.svg"));
    droneItem->setObjectName("trueDrone");
    droneItem->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    const QRectF iconBounds = droneItem->boundingRect();
    droneItem->setTransformOriginPoint(iconBounds.center());
    const double iconScale = 48.0 / std::max(iconBounds.width(), iconBounds.height());
    QTransform iconTransform;
    iconTransform.scale(iconScale, iconScale);
    iconTransform.translate(-iconBounds.center().x(), -iconBounds.center().y());
    droneItem->setTransform(iconTransform);
    droneItem->setZValue(10);
    droneItem->setToolTip(tr("Дрон — истинная позиция"));
    scene->addItem(droneItem);
    audioPlayer = new QMediaPlayer(this);
    audioPlayer->setObjectName("flightAudio");
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
    auto *audioOutput = new QAudioOutput(this);
    audioOutput->setVolume(0.3);
    audioPlayer->setAudioOutput(audioOutput);
    connect(audioPlayer, &QMediaPlayer::errorOccurred, this, [this] {
        ui->checkSound->setText(tr("Звук недоступен"));
        ui->checkSound->setToolTip(audioPlayer->errorString());
    });
#else
    audioPlayer->setVolume(30);
    connect(audioPlayer, QOverload<QMediaPlayer::Error>::of(&QMediaPlayer::error), this, [this] {
        ui->checkSound->setText(tr("Звук недоступен"));
        ui->checkSound->setToolTip(audioPlayer->errorString());
    });
#endif
    connect(audioPlayer, &QMediaPlayer::mediaStatusChanged, this, [this](QMediaPlayer::MediaStatus status) {
        if (status != QMediaPlayer::EndOfMedia) return;
        QTimer::singleShot(0, this, [this] {
            if (!timer->isActive() || !ui->checkSound->isChecked()) return;
            audioPlayer->setPosition(0);
            audioPlayer->play();
        });
    });
    connect(ui->checkSound, &QCheckBox::toggled, this, &MainWindow::updateAudio);
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
    connect(ui->checkCorrection, &QCheckBox::toggled, this, [this](bool enabled) {
        simulation.setCorrectionEnabled(enabled);
        updateStatus();
    });
    connect(ui->checkExternalSensor, &QCheckBox::toggled, this, [this](bool available) {
        simulation.setExternalAvailable(available);
        updateStatus();
    });
    const auto updateFilterParameters = [this] {
        simulation.setFilterParameters(ui->spinSensorNoise->value(), ui->spinResponseTime->value());
    };
    connect(ui->spinSensorNoise, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, updateFilterParameters);
    connect(ui->spinResponseTime, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, updateFilterParameters);
    updateFilterParameters();
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
        updateAudio();
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
    updateAudio();
    ui->btnStart->setText(tr("Пауза"));
    updateControls();
    redraw();
    updateStatus();
}
void MainWindow::updateAudio() {
    if (ui->checkSound->isChecked() && !audioSourceSet) {
        audioSourceSet = true;
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        audioPlayer->setSource(QUrl(QStringLiteral("qrc:/assets/drone-flight.mp3")));
#else
        audioPlayer->setMedia(QMediaContent(QUrl(QStringLiteral("qrc:/assets/drone-flight.mp3"))));
#endif
    }
    if (!hasStarted) audioPlayer->stop();
    else if (timer->isActive() && ui->checkSound->isChecked()) audioPlayer->play();
    else audioPlayer->pause();
}
void MainWindow::tick() {
    if (!timer->isActive()) return;
    simulation.step();
    truePath.lineTo(point(simulation.state().truePosition));
    estimatedPath.lineTo(point(simulation.state().estimatedPosition));
    filteredPath.lineTo(point(simulation.state().filteredPosition));
    redraw();
    updateStatus();
}
void MainWindow::correct() {
    simulation.correct();
    // A correction is not physical motion: break the estimated trail instead of
    // drawing a spurious flight segment between the old and corrected positions.
    estimatedPath.moveTo(point(simulation.state().estimatedPosition));
    redraw();
    updateStatus(tr("Разовая коррекция ИНС; её скорость обнулена"));
}
void MainWindow::reset() {
    timer->stop();
    simulation.reset();
    hasStarted = false;
    updateAudio();
    truePath = QPainterPath(point(simulation.state().truePosition));
    estimatedPath = QPainterPath(point(simulation.state().estimatedPosition));
    filteredPath = QPainterPath(point(simulation.state().filteredPosition));
    ui->btnStart->setText(tr("Старт"));
    updateControls();
    ui->graphicsView->setAutoOverview(true);
    redraw();
    updateStatus(tr("Готово к запуску"));
}
void MainWindow::redraw() {
    trueItem->setPath(truePath);
    estimatedItem->setPath(estimatedPath);
    filteredItem->setPath(filteredPath);
    trueMarker->setPos(point(simulation.state().truePosition));
    estimatedMarker->setPos(point(simulation.state().estimatedPosition));
    filteredMarker->setPos(point(simulation.state().filteredPosition));
    droneItem->setPos(trueMarker->pos());
    const auto velocity = simulation.state().trueVelocity;
    droneItem->setRotation(std::hypot(velocity.x, velocity.y) > 1e-9
        ? std::atan2(velocity.y, velocity.x) * 180.0 / std::acos(-1.0) + 45.0 : 0.0);
    const QRectF bounds = QRectF(0, 0, 600, 400)
        .united(truePath.boundingRect()).united(estimatedPath.boundingRect())
        .united(filteredPath.boundingRect()).adjusted(-20, -20, 20, 20);
    scene->setSceneRect(bounds);
    ui->graphicsView->setContentBounds(bounds);
    ui->graphicsView->setPositions(trueMarker->pos(), estimatedMarker->pos(), filteredMarker->pos());
}
void MainWindow::updateStatus(const QString &message) {
    const auto &s = simulation.state();
    const auto acceleration = simulation.acceleration();
    QString filterStatus;
    if (!simulation.correctionEnabled()) filterStatus = tr("Автокоррекция выключена");
    else if (!simulation.externalAvailable()) filterStatus = tr("Нет внешних измерений");
    else if (!simulation.externalMeasurement()) filterStatus = tr("Ожидание измерения");
    else filterStatus = tr("Автокоррекция включена");
    ui->labelStatus->setText(tr("%1\nВремя: %2 с\nОшибка ИНС: %3 м\nИстина: (%4; %5) м\nОценка: (%6; %7) м\na: (%8; %9) м/с²\nОшибка фильтра: %10 м\n%11")
        .arg(message.isEmpty() ? (timer->isActive() ? tr("GPS недоступен · ИНС") : tr("Пауза")) : message)
        .arg(s.time, 0, 'f', 2).arg(simulation.error(), 0, 'f', 3)
        .arg(s.truePosition.x, 0, 'f', 2).arg(s.truePosition.y, 0, 'f', 2)
        .arg(s.estimatedPosition.x, 0, 'f', 2).arg(s.estimatedPosition.y, 0, 'f', 2)
        .arg(acceleration.x, 0, 'f', 3).arg(acceleration.y, 0, 'f', 3)
        .arg(simulation.filteredError(), 0, 'f', 3).arg(filterStatus));
}
