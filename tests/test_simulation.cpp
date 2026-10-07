#include "simulation.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
    }
}

void near(double actual, double expected, const std::string& message)
{
    check(std::isfinite(actual) && std::abs(actual - expected) < 1e-8, message);
}

void vectorNear(navigation::Vector2 actual, navigation::Vector2 expected,
                const std::string& message)
{
    near(actual.x, expected.x, message + " x");
    near(actual.y, expected.y, message + " y");
}

void advance(navigation::Simulation& simulation, int steps)
{
    for (int i = 0; i < steps; ++i) {
        simulation.step();
    }
}

void trajectory(navigation::Vector2 acceleration)
{
    navigation::Simulation simulation;
    simulation.start(acceleration);
    constexpr int steps = 200;
    constexpr double time = steps * navigation::Simulation::timeStep;
    advance(simulation, steps);
    const auto& state = simulation.state();
    near(state.time, time, "elapsed time");
    vectorNear(state.truePosition,
               {300.0 + 0.5 * acceleration.x * time * time,
                200.0 + 0.5 * acceleration.y * time * time}, "analytic true position");
    vectorNear(state.trueVelocity, {acceleration.x * time, acceleration.y * time},
               "analytic true velocity");
    vectorNear(state.estimatedPosition,
               {300.0 + 0.5 * (acceleration.x + simulation.drift) * time * time,
                200.0 + 0.5 * (acceleration.y + simulation.drift) * time * time},
               "analytic estimated position");
    vectorNear(state.estimatedVelocity,
               {(acceleration.x + simulation.drift) * time,
                (acceleration.y + simulation.drift) * time}, "analytic estimated velocity");
    near(simulation.error(), std::sqrt(2.0) * 0.5 * simulation.drift * time * time,
         "Euclidean quadratic drift");
}

void correctionAndReset()
{
    navigation::Simulation simulation;
    near(simulation.error(), 0.0, "initial error");
    simulation.start({2.0, -4.0});
    advance(simulation, 40);
    const auto before = simulation.state();
    simulation.correct();
    vectorNear(simulation.state().estimatedPosition, before.truePosition, "correction position");
    vectorNear(simulation.state().estimatedVelocity, {}, "correction clears estimated velocity");
    vectorNear(simulation.state().truePosition, before.truePosition, "correction preserves true position");
    vectorNear(simulation.state().trueVelocity, before.trueVelocity, "correction preserves true velocity");
    near(simulation.state().time, before.time, "correction preserves time");
    near(simulation.error(), 0.0, "correction clears error");
    simulation.correct();
    near(simulation.error(), 0.0, "correction is idempotent");
    simulation.step();
    constexpr double dt = navigation::Simulation::timeStep;
    vectorNear(simulation.state().estimatedPosition,
               {before.truePosition.x + 0.5 * 2.02 * dt * dt,
                before.truePosition.y - 0.5 * 3.98 * dt * dt},
               "integration after correction starts with zero estimated velocity");
    simulation.reset();
    vectorNear(simulation.state().truePosition, {300, 200}, "reset true position");
    vectorNear(simulation.state().estimatedPosition, {300, 200}, "reset estimated position");
    vectorNear(simulation.state().trueVelocity, {}, "reset true velocity");
    vectorNear(simulation.state().estimatedVelocity, {}, "reset estimated velocity");
    near(simulation.state().time, 0, "reset time");
    simulation.step();
    vectorNear(simulation.state().truePosition, {300, 200}, "reset clears acceleration");
    vectorNear(simulation.state().trueVelocity, {}, "reset clears acceleration velocity");
}

void invalidAcceleration()
{
    navigation::Simulation simulation;
    simulation.start({1.0, 2.0});
    const double invalidValues[] = {100.001, -100.001,
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    for (double value : invalidValues) {
        for (const auto acceleration : {navigation::Vector2{value, 0}, navigation::Vector2{0, value}}) {
            bool threw = false;
            try {
                simulation.start(acceleration);
            } catch (const std::invalid_argument&) {
                threw = true;
            }
            check(threw, "invalid acceleration rejected");
        }
    }
    simulation.step();
    vectorNear(simulation.state().trueVelocity, {0.05, 0.1},
               "invalid input preserves previously configured acceleration");
    simulation.start({-100.0, 100.0});
    near(simulation.state().time, simulation.timeStep, "start preserves ongoing state");
    simulation.step();
    vectorNear(simulation.state().trueVelocity, {-4.95, 5.1}, "inclusive bounds accepted");
}

void circleTrajectory()
{
    navigation::Simulation simulation;
    constexpr double radius = 80.0;
    constexpr double period = 10.0;
    const double omega = 2.0 * std::acos(-1.0) / period;
    simulation.configureCircle(radius, period);
    check(simulation.mode() == navigation::MotionMode::Circle, "circle mode selected");
    vectorNear(simulation.state().truePosition, {300, 200}, "circle starts at original position");
    vectorNear(simulation.state().trueVelocity, {0, radius * omega}, "circle initial tangential velocity");
    for (int i = 0; i < 1000; ++i) {
        simulation.step();
        const auto& state = simulation.state();
        const double angle = omega * state.time;
        near(std::hypot(state.truePosition.x - (300.0 - radius), state.truePosition.y - 200.0),
             radius, "circle radius remains constant over five loops");
        vectorNear(state.trueVelocity, {-radius * omega * std::sin(angle), radius * omega * std::cos(angle)},
                   "circle velocity is tangent with constant speed");
        vectorNear(simulation.acceleration(), {-radius * omega * omega * std::cos(angle),
                                               -radius * omega * omega * std::sin(angle)},
                   "circle acceleration is centripetal");
        const double bias = 0.5 * simulation.drift * state.time * state.time;
        vectorNear(state.estimatedPosition, {state.truePosition.x + bias, state.truePosition.y + bias},
                   "circle estimate integrates quadratic bias independently");
        vectorNear(state.estimatedVelocity, {state.trueVelocity.x + simulation.drift * state.time,
                                            state.trueVelocity.y + simulation.drift * state.time},
                   "circle estimate integrates velocity bias");
        if ((i + 1) % 200 == 0) {
            vectorNear(state.truePosition, {300, 200}, "circle returns after full period");
        }
    }
}

void circleCorrection()
{
    navigation::Simulation simulation;
    simulation.configureCircle(80, 10);
    advance(simulation, 65);
    const auto before = simulation.state();
    simulation.correct();
    near(simulation.error(), 0, "circle correction clears position error immediately");
    vectorNear(simulation.state().estimatedVelocity, {}, "circle correction resets estimated velocity");
    vectorNear(simulation.state().trueVelocity, before.trueVelocity, "circle correction preserves true velocity");
    constexpr int steps = 25;
    constexpr double elapsed = steps * navigation::Simulation::timeStep;
    advance(simulation, steps);
    const auto& state = simulation.state();
    vectorNear(state.estimatedPosition,
               {state.truePosition.x - before.trueVelocity.x * elapsed + 0.5 * simulation.drift * elapsed * elapsed,
                state.truePosition.y - before.trueVelocity.y * elapsed + 0.5 * simulation.drift * elapsed * elapsed},
               "circle integration preserves reset estimated velocity after correction");
    vectorNear(state.estimatedVelocity,
               {state.trueVelocity.x - before.trueVelocity.x + simulation.drift * elapsed,
                state.trueVelocity.y - before.trueVelocity.y + simulation.drift * elapsed},
               "circle velocity remains independent after correction");
    simulation.reset();
    check(simulation.mode() == navigation::MotionMode::ConstantAcceleration, "reset clears circle mode");
    vectorNear(simulation.acceleration(), {}, "reset clears circle acceleration");
    vectorNear(simulation.state().truePosition, {300, 200}, "reset circle position");
    vectorNear(simulation.state().estimatedPosition, {300, 200}, "reset circle estimated position");
    vectorNear(simulation.state().trueVelocity, {}, "reset circle velocity");
    vectorNear(simulation.state().estimatedVelocity, {}, "reset circle estimated velocity");
    near(simulation.state().time, 0, "reset circle time");
    simulation.step();
    vectorNear(simulation.state().truePosition, {300, 200}, "reset returns stationary true trajectory");
}

void circleConfiguration()
{
    navigation::Simulation simulation;
    simulation.configureCircle(50, 20);
    const auto before = simulation.state();
    const auto beforeAcceleration = simulation.acceleration();
    const double invalidValues[] = {0.0, -1.0, 10001.0,
        std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()};
    for (double value : invalidValues) {
        for (bool invalidRadius : {false, true}) {
            bool threw = false;
            try {
                simulation.configureCircle(invalidRadius ? value : 50, invalidRadius ? 20 : value);
            } catch (const std::invalid_argument&) {
                threw = true;
            }
            check(threw, "invalid circle parameters rejected");
            vectorNear(simulation.state().truePosition, before.truePosition, "invalid circle preserves position");
            vectorNear(simulation.state().trueVelocity, before.trueVelocity, "invalid circle preserves velocity");
            vectorNear(simulation.acceleration(), beforeAcceleration, "invalid circle preserves acceleration");
            check(simulation.mode() == navigation::MotionMode::Circle, "invalid circle preserves mode");
        }
    }
    bool threw = false;
    try { simulation.configureCircle(50, 3600.001); }
    catch (const std::invalid_argument&) { threw = true; }
    check(threw, "period upper bound enforced");
    simulation.configureCircle(1, 1);
    simulation.configureCircle(10000, 3600);
    simulation.step();
    const auto running = simulation.state();
    const auto runningAcceleration = simulation.acceleration();
    threw = false;
    try { simulation.configureCircle(50, 20); }
    catch (const std::logic_error&) { threw = true; }
    check(threw, "circle reconfiguration requires reset after time advances");
    near(simulation.state().time, running.time, "rejected reconfiguration preserves time");
    vectorNear(simulation.state().truePosition, running.truePosition, "rejected reconfiguration preserves position");
    vectorNear(simulation.state().estimatedPosition, running.estimatedPosition, "rejected reconfiguration preserves estimate");
    vectorNear(simulation.state().trueVelocity, running.trueVelocity, "rejected reconfiguration preserves velocity");
    vectorNear(simulation.acceleration(), runningAcceleration, "rejected reconfiguration preserves acceleration");
    simulation.start({2, 3});
    check(simulation.mode() == navigation::MotionMode::ConstantAcceleration, "start selects constant acceleration mode");
    vectorNear(simulation.acceleration(), {2, 3}, "constant acceleration accessor");
    simulation.step();
    vectorNear(simulation.state().trueVelocity, {running.trueVelocity.x + 2 * simulation.timeStep,
                                              running.trueVelocity.y + 3 * simulation.timeStep},
               "start preserves existing state while switching modes");
}

void filterSwitching(bool circle)
{
    navigation::Simulation simulation;
    navigation::Simulation baseline;
    simulation.setFilterParameters(0.0, 2.0);
    simulation.setCorrectionEnabled(false);
    baseline.setCorrectionEnabled(false);
    if (circle) {
        simulation.configureCircle(100, 40);
        baseline.configureCircle(100, 40);
    } else {
        simulation.start({0.5, -0.3});
        baseline.start({0.5, -0.3});
    }
    advance(simulation, 600);
    advance(baseline, 600);
    vectorNear(simulation.state().filteredPosition, simulation.state().estimatedPosition,
               "disabled filter follows raw INS");
    vectorNear(simulation.state().filteredVelocity, simulation.state().estimatedVelocity,
               "disabled filter retains INS velocity");
    const auto beforeEnable = simulation.state();
    simulation.setCorrectionEnabled(true);
    vectorNear(simulation.state().filteredPosition, beforeEnable.filteredPosition,
               "enabling correction does not teleport");
    vectorNear(simulation.state().filteredVelocity, beforeEnable.filteredVelocity,
               "enabling correction does not reset velocity");
    near(simulation.state().time, beforeEnable.time, "enabling preserves time");
    advance(simulation, 600);
    advance(baseline, 600);
    check(simulation.filteredError() < 0.2, "filter recovers after enabling in motion");
    vectorNear(simulation.state().truePosition, baseline.state().truePosition,
               "filter does not change truth");
    vectorNear(simulation.state().estimatedPosition, baseline.state().estimatedPosition,
               "filter does not change raw INS");
    const auto beforeDisable = simulation.state();
    simulation.setCorrectionEnabled(false);
    vectorNear(simulation.state().filteredPosition, beforeDisable.filteredPosition,
               "disabling preserves position");
    vectorNear(simulation.state().filteredVelocity, beforeDisable.filteredVelocity,
               "disabling preserves velocity");
    advance(simulation, 400);
    constexpr double elapsed = 400 * navigation::Simulation::timeStep;
    const double bias = 0.5 * simulation.drift * elapsed * elapsed;
    vectorNear(simulation.state().filteredPosition,
               {simulation.state().truePosition.x + beforeDisable.filteredPosition.x - beforeDisable.truePosition.x
                    + (beforeDisable.filteredVelocity.x - beforeDisable.trueVelocity.x) * elapsed + bias,
                simulation.state().truePosition.y + beforeDisable.filteredPosition.y - beforeDisable.truePosition.y
                    + (beforeDisable.filteredVelocity.y - beforeDisable.trueVelocity.y) * elapsed + bias},
               "disabled correction continues inertial integration from current state");
    check(simulation.filteredError() > 2.0, "drift resumes while correction is disabled");
    simulation.setCorrectionEnabled(true);
    advance(simulation, 600);
    check(simulation.filteredError() < 0.2, "filter recovers after re-enabling");
    const auto beforeManual = simulation.state();
    simulation.correct();
    vectorNear(simulation.state().filteredPosition, beforeManual.filteredPosition,
               "manual INS correction does not change filter position");
    vectorNear(simulation.state().filteredVelocity, beforeManual.filteredVelocity,
               "manual INS correction does not reset filter velocity");
}

void filterMeasurements()
{
    navigation::Simulation simulation;
    simulation.setFilterParameters(0.0, 2.0);
    simulation.start({0.5, 0.3});
    advance(simulation, 3);
    check(!simulation.externalMeasurement(), "no measurement before 0.2 seconds");
    vectorNear(simulation.state().filteredPosition, simulation.state().estimatedPosition,
               "no correction without a fresh measurement");
    simulation.step();
    check(simulation.externalMeasurement().has_value(), "measurement arrives at 5 Hz");
    vectorNear(*simulation.externalMeasurement(), simulation.state().truePosition,
               "zero-noise sensor gives current position");
    const double expectedPositionGain = 1.0 - std::exp(-0.2);
    const double expectedVelocityGain = std::pow(1.0 - std::exp(-0.1), 2) / 0.2;
    near(simulation.state().filteredPosition.x - simulation.state().truePosition.x,
         (1.0 - expectedPositionGain) * 0.0004, "position feedback at first measurement");
    near(simulation.state().filteredVelocity.x - simulation.state().trueVelocity.x,
         0.004 - expectedVelocityGain * 0.0004, "velocity feedback at first measurement");
    const auto lastMeasurement = *simulation.externalMeasurement();
    const auto before = simulation.state();
    simulation.step();
    vectorNear(*simulation.externalMeasurement(), lastMeasurement, "measurement held between sensor updates");
    near(simulation.state().filteredPosition.x,
         before.filteredPosition.x + before.filteredVelocity.x * 0.05 + 0.5 * 0.52 * 0.05 * 0.05,
         "old measurement is not reapplied between updates");
    simulation.setExternalAvailable(false);
    check(!simulation.externalMeasurement(), "disconnect clears stale sensor reading");
    advance(simulation, 600);
    check(!simulation.externalMeasurement(), "disconnected sensor produces no readings");
    check(simulation.filteredError() > 10.0, "filter drifts without external sensor");
    const auto disconnected = simulation.state();
    simulation.setExternalAvailable(true);
    vectorNear(simulation.state().filteredPosition, disconnected.filteredPosition,
               "reconnection waits for a fresh reading");
    advance(simulation, 600);
    check(simulation.filteredError() < 0.2, "filter recovers after sensor reconnects");
}

void filterNoiseAndReset()
{
    for (bool circle : {false, true}) {
        navigation::Simulation simulation;
        if (circle) simulation.configureCircle(100, 40);
        else simulation.start({0.5, 0.3});
        double squaredError = 0.0;
        constexpr int samples = 1200;
        for (int sample = 0; sample < samples; ++sample) {
            simulation.step();
            squaredError += simulation.filteredError() * simulation.filteredError();
        }
        check(std::sqrt(squaredError / samples) < 1.0,
              "noisy filter remains within one metre RMS over 60 seconds");
        check(simulation.filteredError() < simulation.error(), "filter improves on raw INS with noisy sensor");
        const auto firstRun = simulation.state();
        const auto measurement = *simulation.externalMeasurement();
        simulation.reset();
        check(!simulation.externalMeasurement(), "reset clears measurements");
        vectorNear(simulation.state().filteredPosition, {300, 200}, "reset restores filtered position");
        vectorNear(simulation.state().filteredVelocity, {}, "reset clears filtered velocity");
        if (circle) simulation.configureCircle(100, 40);
        else simulation.start({0.5, 0.3});
        advance(simulation, samples);
        vectorNear(simulation.state().filteredPosition, firstRun.filteredPosition,
                   "reset reproduces the same experiment");
        vectorNear(*simulation.externalMeasurement(), measurement, "reset reproduces sensor noise");
        simulation.setCorrectionEnabled(false);
        simulation.setExternalAvailable(false);
        simulation.reset();
        check(!simulation.correctionEnabled() && !simulation.externalAvailable(),
              "reset preserves user switch settings");
    }
}

void filterParameterValidation()
{
    navigation::Simulation simulation;
    for (double invalid : {-1.0, 10.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool threw = false;
        try { simulation.setFilterParameters(invalid, 2.0); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid sensor noise rejected");
    }
    for (double invalid : {0.0, 0.199, 20.01, std::numeric_limits<double>::infinity(),
                           std::numeric_limits<double>::quiet_NaN()}) {
        bool threw = false;
        try { simulation.setFilterParameters(0.5, invalid); }
        catch (const std::invalid_argument&) { threw = true; }
        check(threw, "invalid response time rejected");
    }
    for (double response : {0.2, 20.0}) {
        simulation.reset();
        simulation.setFilterParameters(0.0, response);
        simulation.start({-100, 100});
        advance(simulation, 12000);
        check(std::isfinite(simulation.filteredError()) && simulation.filteredError() < 12.0,
              "filter remains bounded at response-time limits");
    }
    simulation.setFilterParameters(10.0, 2.0);
    simulation.step();
    check(std::isfinite(simulation.filteredError()), "maximum sensor noise accepted");
}

} // namespace

int main()
{
    trajectory({3.0, 7.0});
    trajectory({0.0, 0.0});
    trajectory({-3.0, -7.0});
    trajectory({-100.0, 100.0});
    correctionAndReset();
    invalidAcceleration();
    circleTrajectory();
    circleCorrection();
    circleConfiguration();
    filterSwitching(false);
    filterSwitching(true);
    filterMeasurements();
    filterNoiseAndReset();
    filterParameterValidation();
    if (failures != 0) {
        std::cerr << failures << " checks failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All simulation checks passed\n";
    return EXIT_SUCCESS;
}
