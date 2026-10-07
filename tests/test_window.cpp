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
        QCOMPARE(paths, 2);
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
        QVERIFY(!status->text().contains("Ошибка: 0.000"));
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
        QVERIFY(status->text().contains("Ошибка: 0.000"));
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
        QCOMPARE(view->scene()->items().size(), 4);
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
        QVERIFY(!status->text().contains("Ошибка: 0.000"));
        start->click();
        QVERIFY(!timer->isActive());
        const QString paused = status->text();
        QTest::qWait(120);
        QCOMPARE(status->text(), paused);
        correct->click();
        QVERIFY(status->text().contains("Ошибка: 0.000"));
        start->click();
        QVERIFY(timer->isActive());
        QTest::qWait(160);
        start->click();
        QVERIFY(!status->text().contains("Время: 5.00"));
        reset->click();
        QVERIFY(!timer->isActive());
        QVERIFY(ax->isEnabled());
        QVERIFY(status->text().contains("Время: 0.00"));
        QVERIFY(status->text().contains("Ошибка: 0.000"));
        QCOMPARE(ax->text(), QString("0,5"));
        for (auto *item : view->scene()->items()) {
            if (auto *path = qgraphicsitem_cast<QGraphicsPathItem *>(item)) QCOMPARE(path->path().elementCount(), 1);
        }
    }
};
QTEST_MAIN(WindowTest)
#include "test_window.moc"
