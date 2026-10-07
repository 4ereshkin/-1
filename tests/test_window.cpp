#include "mainwindow.h"
#include "trajectoryview.h"
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QWheelEvent>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <QGraphicsPathItem>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QTimer>
#include <QGraphicsSvgItem>
#include <QSvgRenderer>
#include <QMediaPlayer>
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
#include <QAudioOutput>
#endif
#include <QtTest>
class WindowTest : public QObject {
    Q_OBJECT
private slots:
    void renderedTrajectories() {
        MainWindow w;
        w.show();
        QTest::qWait(20);
        w.findChild<QPushButton *>("btnStart")->click();
        for (int i = 0; i < 1200; ++i) QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        w.findChild<QPushButton *>("btnStart")->click();
        auto *view = w.findChild<QGraphicsView *>("graphicsView");
        int paths = 0;
        for (auto *item : view->scene()->items()) {
            if (auto *path = qgraphicsitem_cast<QGraphicsPathItem *>(item)) {
                QVERIFY(path->path().elementCount() > 1000);
                QVERIFY(view->sceneRect().contains(path->path().boundingRect()));
                ++paths;
            }
        }
        QCOMPARE(paths, 3);
        QVERIFY(w.grab().save("window-preview.png"));
        w.resize(1200, 800);
        QTest::qWait(20);
        QVERIFY(view->mapToScene(view->viewport()->rect()).boundingRect().contains(view->scene()->sceneRect()));
    }
    void cameraInteractions() {
        MainWindow w;
        w.show();
        QTest::qWait(20);
        auto *view = w.findChild<TrajectoryView *>("graphicsView");
        auto *autoBox = w.findChild<QCheckBox *>("checkAutoFollow");
        QVERIFY(view && autoBox);
        QVERIFY(view->autoOverview());
        const double initialScale = view->transform().m11();
        const double initialGrid = view->gridStep();
        const QPoint anchor = view->viewport()->rect().center();
        const QPointF sceneAnchor = view->mapToScene(anchor);
        QWheelEvent wheel(anchor, view->viewport()->mapToGlobal(anchor), QPoint(), QPoint(0, 240),
                          Qt::NoButton, Qt::NoModifier, Qt::NoScrollPhase, false);
        QApplication::sendEvent(view->viewport(), &wheel);
        QVERIFY(view->transform().m11() > initialScale);
        QVERIFY(view->gridStep() <= initialGrid);
        QVERIFY(!view->autoOverview());
        QVERIFY(!autoBox->isChecked());
        QVERIFY(QLineF(sceneAnchor, view->mapToScene(anchor)).length() < 3/view->transform().m11());
        const double manualScale = view->transform().m11();
        const QPointF manualCenter = view->mapToScene(anchor);
        w.findChild<QPushButton *>("btnStart")->click();
        for (int i = 0; i < 2400; ++i) QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        w.findChild<QPushButton *>("btnStart")->click();
        QCOMPARE(view->transform().m11(), manualScale);
        QVERIFY2(QLineF(manualCenter, view->mapToScene(anchor)).length() < 3/manualScale,
                 "Model ticks must not move the manual camera");
        QTest::keyClick(view, Qt::Key_Right);
        QVERIFY(view->mapToScene(anchor).x() > manualCenter.x());
        const QPointF beforeDrag = view->mapToScene(anchor);
        QTest::mousePress(view->viewport(), Qt::LeftButton, Qt::NoModifier, anchor);
        QMouseEvent drag(QEvent::MouseMove, anchor+QPoint(60, 30), Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
        QApplication::sendEvent(view->viewport(), &drag);
        QTest::mouseRelease(view->viewport(), Qt::LeftButton, Qt::NoModifier, anchor+QPoint(60, 30));
        QVERIFY(QLineF(beforeDrag, view->mapToScene(anchor)).length() > 10/manualScale);
        w.resize(1100, 780);
        QTest::qWait(20);
        QCOMPARE(view->transform().m11(), manualScale);
        QTest::keyClick(view, Qt::Key_F);
        QVERIFY(view->autoOverview());
        QVERIFY(autoBox->isChecked());
        QVERIFY(view->mapToScene(view->viewport()->rect()).boundingRect().contains(view->scene()->sceneRect()));
        QTest::keyClick(view, Qt::Key_Plus);
        QVERIFY(!autoBox->isChecked());
        QTest::keyClick(view, Qt::Key_Minus);
        autoBox->setChecked(true);
        QVERIFY(view->autoOverview());
        autoBox->setChecked(false);
        w.findChild<QPushButton *>("btnReset")->click();
        QVERIFY(autoBox->isChecked());
    }
    void circlePauseBeforeFirstTick() {
        MainWindow w;
        auto *mode = w.findChild<QComboBox *>("comboMotion");
        auto *radius = w.findChild<QDoubleSpinBox *>("spinRadius");
        auto *start = w.findChild<QPushButton *>("btnStart");
        auto *status = w.findChild<QLabel *>("labelStatus");
        mode->setCurrentIndex(1);
        start->click();
        start->click(); // Pause before timer can deliver the first step.
        QVERIFY(!mode->isEnabled());
        QVERIFY(!radius->isEnabled());
        w.findChild<QPushButton *>("btnCorrect")->click();
        start->click();
        QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        start->click();
        // Correction zeroed estimated velocity; resume must not initialize it again.
        QVERIFY(status->text().contains("Оценка: (300.00; 200.00)"));
        QVERIFY(!status->text().contains("Ошибка ИНС: 0.000"));
        w.findChild<QPushButton *>("btnReset")->click();
        QVERIFY(mode->isEnabled());
    }
    void circleControls() {
        MainWindow w;
        w.show();
        QTest::qWait(20);
        auto *mode = w.findChild<QComboBox *>("comboMotion");
        auto *radius = w.findChild<QDoubleSpinBox *>("spinRadius");
        auto *period = w.findChild<QDoubleSpinBox *>("spinPeriod");
        auto *ax = w.findChild<QLineEdit *>("lineEditAx");
        auto *start = w.findChild<QPushButton *>("btnStart");
        auto *status = w.findChild<QLabel *>("labelStatus");
        QVERIFY(mode && radius && period);
        QVERIFY(!radius->isVisible());
        mode->setCurrentIndex(1);
        QTest::qWait(20);
        QVERIFY(radius->isVisible());
        QVERIFY(!ax->isVisible());
        radius->setValue(100);
        period->setValue(40);
        start->click();
        QVERIFY(!mode->isEnabled());
        QVERIFY(!radius->isEnabled());
        for (int i = 0; i < 400; ++i) QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        start->click();
        QVERIFY(status->text().contains("Истина: (100.00; 200.00)"));
        QVERIFY(!radius->isEnabled());
        QVERIFY(!mode->isEnabled());
        start->click();
        for (int i = 0; i < 400; ++i) QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        start->click();
        QVERIFY(status->text().contains("Истина: (300.00; 200.00)"));
        QVERIFY(status->text().contains("Время: 40.00"));
        QVERIFY(w.grab().save("circle-preview.png"));
        w.findChild<QPushButton *>("btnCorrect")->click();
        QVERIFY(status->text().contains("Ошибка ИНС: 0.000"));
        w.findChild<QPushButton *>("btnReset")->click();
        QVERIFY(mode->isEnabled());
        QVERIFY(radius->isEnabled());
        QCOMPARE(mode->currentIndex(), 1);
        start->click();
        QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection);
        start->click();
        QVERIFY(status->text().contains("Время: 0.05"));
        w.findChild<QPushButton *>("btnReset")->click();
        mode->setCurrentIndex(0);
        QVERIFY(ax->isVisible());
    }
    void filterControlsDuringMotion() {
        MainWindow window;
        window.show();
        QTest::qWait(20);
        auto *correction = window.findChild<QCheckBox *>("checkCorrection");
        auto *sensor = window.findChild<QCheckBox *>("checkExternalSensor");
        auto *noise = window.findChild<QDoubleSpinBox *>("spinSensorNoise");
        auto *response = window.findChild<QDoubleSpinBox *>("spinResponseTime");
        auto *timer = window.findChild<QTimer *>("simulationTimer");
        auto *status = window.findChild<QLabel *>("labelStatus");
        auto *view = window.findChild<QGraphicsView *>("graphicsView");
        auto *start = window.findChild<QPushButton *>("btnStart");
        QVERIFY(correction && sensor && noise && response && timer && status && view && start);
        noise->setValue(0.0);
        start->click();
        QVERIFY(correction->isEnabled() && sensor->isEnabled());
        QVERIFY(noise->isEnabled() && response->isEnabled());
        QGraphicsPathItem *filtered = nullptr;
        for (auto *item : view->scene()->items()) {
            auto *path = qgraphicsitem_cast<QGraphicsPathItem *>(item);
            if (path && path->pen().color() == QColor(215, 105, 0)) filtered = path;
        }
        QVERIFY(filtered);
        for (int step = 0; step < 400; ++step) QMetaObject::invokeMethod(&window, "tick", Qt::DirectConnection);
        const QPainterPath before = filtered->path();
        correction->click();
        QVERIFY(timer->isActive());
        QCOMPARE(filtered->path(), before);
        QVERIFY(status->text().contains("Время: 20.00"));
        QVERIFY(status->text().contains("Автокоррекция выключена"));
        for (int step = 0; step < 400; ++step) QMetaObject::invokeMethod(&window, "tick", Qt::DirectConnection);
        correction->click();
        QVERIFY(timer->isActive());
        QVERIFY(status->text().contains("Автокоррекция включена"));
        sensor->click();
        QVERIFY(status->text().contains("Нет внешних измерений"));
        QVERIFY(timer->isActive());
        sensor->click();
        QVERIFY(status->text().contains("Ожидание измерения"));
        noise->setValue(0.3);
        response->setValue(1.5);
        for (int step = 0; step < 600; ++step) QMetaObject::invokeMethod(&window, "tick", Qt::DirectConnection);
        QVERIFY(filtered->path().elementCount() > 1000);
        QVERIFY(status->text().contains("Ошибка фильтра:"));
        QVERIFY(status->text().contains("Автокоррекция включена"));
        start->click();
        QVERIFY(window.grab().save("filter-preview.png"));
        const QPainterPath paused = filtered->path();
        correction->click();
        QTest::qWait(120);
        QCOMPARE(filtered->path(), paused);
        QVERIFY(!timer->isActive());
        window.findChild<QPushButton *>("btnReset")->click();
        QVERIFY(!correction->isChecked());
        QCOMPARE(noise->value(), 0.3);
        QCOMPARE(response->value(), 1.5);
        QVERIFY(status->text().contains("Ошибка фильтра: 0.000"));
        QCOMPARE(filtered->path().elementCount(), 1);
    }
    void droneFollowsTruth() {
        MainWindow window;
        auto *view = window.findChild<QGraphicsView *>("graphicsView");
        QGraphicsSvgItem *drone = nullptr;
        for (auto *item : view->scene()->items()) {
            if (auto *svg = qgraphicsitem_cast<QGraphicsSvgItem *>(item)) drone = svg;
        }
        QVERIFY(drone && drone->renderer()->isValid());
        QCOMPARE(drone->pos(), QPointF(300, 200));
        QVERIFY(drone->flags().testFlag(QGraphicsItem::ItemIgnoresTransformations));
        window.findChild<QComboBox *>("comboMotion")->setCurrentIndex(1);
        window.findChild<QPushButton *>("btnStart")->click();
        for (int step = 0; step < 200; ++step) QMetaObject::invokeMethod(&window, "tick", Qt::DirectConnection);
        QVERIFY(QLineF(drone->pos(), QPointF(200, 300)).length() < 1e-8);
        const QPointF screenCenter = drone->deviceTransform(view->viewportTransform()).map(drone->boundingRect().center());
        QVERIFY(QLineF(screenCenter, view->viewportTransform().map(drone->pos())).length() < 1e-8);
        const QSizeF sizeBefore = drone->deviceTransform(view->viewportTransform()).mapRect(drone->boundingRect()).size();
        view->scale(2, 2);
        const QSizeF sizeAfter = drone->deviceTransform(view->viewportTransform()).mapRect(drone->boundingRect()).size();
        QCOMPARE(sizeAfter, sizeBefore);
        const QPointF beforeCorrection = drone->pos();
        window.findChild<QPushButton *>("btnCorrect")->click();
        QCOMPARE(drone->pos(), beforeCorrection);
        window.findChild<QPushButton *>("btnStart")->click();
        QTest::qWait(100);
        QCOMPARE(drone->pos(), beforeCorrection);
        window.findChild<QPushButton *>("btnReset")->click();
        QCOMPARE(drone->pos(), QPointF(300, 200));
        QCOMPARE(drone->rotation(), 0.0);
    }
    void flightAudioControls() {
        MainWindow window;
        auto *sound = window.findChild<QCheckBox *>("checkSound");
        auto *start = window.findChild<QPushButton *>("btnStart");
        auto *player = window.findChild<QMediaPlayer *>("flightAudio");
        QVERIFY(sound && start && player);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        QVERIFY(player->audioOutput());
        player->audioOutput()->setMuted(true);
        const auto playbackState = [player] { return player->playbackState(); };
#else
        player->setMuted(true);
        const auto playbackState = [player] { return player->state(); };
#endif
        QVERIFY(!sound->isChecked());
        sound->click();
        QTRY_VERIFY_WITH_TIMEOUT(player->duration() > 0, 5000);
        QCOMPARE(player->error(), QMediaPlayer::NoError);
        QCOMPARE(playbackState(), QMediaPlayer::StoppedState);
        start->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PlayingState);
        QTRY_VERIFY(player->position() > 0);
        start->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PausedState);
        start->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PlayingState);
        sound->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PausedState);
        QVERIFY(window.findChild<QTimer *>("simulationTimer")->isActive());
        sound->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PlayingState);
        player->setPosition(player->duration() - 100);
        QTRY_VERIFY(player->position() >= player->duration() - 1000);
        QTRY_VERIFY_WITH_TIMEOUT(player->position() < player->duration() / 2, 5000);
        QCOMPARE(playbackState(), QMediaPlayer::PlayingState);
        window.findChild<QPushButton *>("btnReset")->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::StoppedState);
        QCOMPARE(player->position(), qint64(0));
        QVERIFY(sound->isChecked());
        QCOMPARE(player->error(), QMediaPlayer::NoError);
    }
    void flightAudioNaturalLoop() {
        MainWindow window;
        auto *player = window.findChild<QMediaPlayer *>("flightAudio");
        auto *sound = window.findChild<QCheckBox *>("checkSound");
        auto *start = window.findChild<QPushButton *>("btnStart");
        QVERIFY(player && sound && start);
#if QT_VERSION >= QT_VERSION_CHECK(6, 0, 0)
        player->audioOutput()->setMuted(true);
        const auto playbackState = [player] { return player->playbackState(); };
#else
        player->setMuted(true);
        const auto playbackState = [player] { return player->state(); };
#endif
        int completedLoops = 0;
        connect(player, &QMediaPlayer::mediaStatusChanged, &window, [&](QMediaPlayer::MediaStatus status) {
            if (status == QMediaPlayer::EndOfMedia) ++completedLoops;
        });
        sound->click();
        start->click();
        QTRY_VERIFY_WITH_TIMEOUT(player->duration() > 0, 5000);
        const int loopTimeout = int(player->duration()) + 5000;
        for (int cycle = 1; cycle <= 2; ++cycle) {
            QTRY_VERIFY_WITH_TIMEOUT(completedLoops >= cycle, loopTimeout);
            QTRY_COMPARE(playbackState(), QMediaPlayer::PlayingState);
            QTRY_VERIFY_WITH_TIMEOUT(player->position() > 500 && player->position() < 5000, 5000);
        }
        start->click();
        QTRY_COMPARE(playbackState(), QMediaPlayer::PausedState);
        const int completedBeforePause = completedLoops;
        QTest::qWait(200);
        QCOMPARE(completedLoops, completedBeforePause);
        QCOMPARE(player->error(), QMediaPlayer::NoError);
    }
    void controls() {
        MainWindow w;
        auto *start = w.findChild<QPushButton *>("btnStart");
        auto *reset = w.findChild<QPushButton *>("btnReset");
        auto *correct = w.findChild<QPushButton *>("btnCorrect");
        auto *ax = w.findChild<QLineEdit *>("lineEditAx");
        auto *ay = w.findChild<QLineEdit *>("lineEditAy");
        auto *status = w.findChild<QLabel *>("labelStatus");
        auto *timer = w.findChild<QTimer *>("simulationTimer");
        auto *view = w.findChild<QGraphicsView *>("graphicsView");
        QVERIFY(start && reset && correct && ax && ay && status && timer && view);
        QCOMPARE(timer->interval(), 50);
        QCOMPARE(view->scene()->items().size(), 7);
        for (const QString &bad : {QString(), QString("nan"), QString("inf"), QString("101"), QString("1,2.3")}) {
            ax->setText(bad);
            start->click();
            QVERIFY(!timer->isActive());
            QVERIFY(status->text().contains("Введите"));
        }
        ax->setText("0,5");
        ay->setText("bad");
        start->click();
        QVERIFY(!timer->isActive());
        ay->setText("-0.3");
        start->click();
        QVERIFY(timer->isActive());
        QVERIFY(!ax->isEnabled());
        for (int i = 0; i < 100; ++i) QVERIFY(QMetaObject::invokeMethod(&w, "tick", Qt::DirectConnection));
        QVERIFY(status->text().contains("5.00"));
        QVERIFY(!status->text().contains("Ошибка ИНС: 0.000"));
        start->click();
        QVERIFY(!timer->isActive());
        const QString paused = status->text();
        QTest::qWait(120);
        QCOMPARE(status->text(), paused);
        correct->click();
        QVERIFY(status->text().contains("Ошибка ИНС: 0.000"));
        start->click();
        QVERIFY(timer->isActive());
        QTest::qWait(160);
        start->click();
        QVERIFY(!status->text().contains("Время: 5.00"));
        reset->click();
        QVERIFY(!timer->isActive());
        QVERIFY(ax->isEnabled());
        QVERIFY(status->text().contains("Время: 0.00"));
        QVERIFY(status->text().contains("Ошибка ИНС: 0.000"));
        QCOMPARE(ax->text(), QString("0,5"));
        for (auto *item : view->scene()->items()) {
            if (auto *path = qgraphicsitem_cast<QGraphicsPathItem *>(item)) QCOMPARE(path->path().elementCount(), 1);
        }
    }
};
QTEST_MAIN(WindowTest)
#include "test_window.moc"
