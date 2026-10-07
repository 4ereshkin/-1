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
    if (failures != 0) {
        std::cerr << failures << " checks failed\n";
        return EXIT_FAILURE;
    }
    std::cout << "All simulation checks passed\n";
    return EXIT_SUCCESS;
}
