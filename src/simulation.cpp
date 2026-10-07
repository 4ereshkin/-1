#include "simulation.h"

#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace navigation {
namespace {

void integrate(Vector2& position, Vector2& velocity, Vector2 acceleration)
{
    constexpr double dt = Simulation::timeStep;
    position.x += velocity.x * dt + 0.5 * acceleration.x * dt * dt;
    position.y += velocity.y * dt + 0.5 * acceleration.y * dt * dt;
    velocity.x += acceleration.x * dt;
    velocity.y += acceleration.y * dt;
}

bool validAcceleration(double value)
{
    return std::isfinite(value) && value >= -100.0 && value <= 100.0;
}

} // namespace

const State& Simulation::state() const
{
    return state_;
}

void Simulation::start(Vector2 acceleration)
{
    if (!validAcceleration(acceleration.x) || !validAcceleration(acceleration.y)) {
        throw std::invalid_argument("Acceleration must be finite and within [-100, 100]");
    }
    acceleration_ = acceleration;
    mode_ = MotionMode::ConstantAcceleration;
}

void Simulation::configureCircle(double radius, double period)
{
    if (state_.time != 0.0) {
        throw std::logic_error("Reset the simulation before configuring a circle");
    }
    if (!std::isfinite(radius) || radius < 1.0 || radius > 10000.0 ||
        !std::isfinite(period) || period < 1.0 || period > 3600.0) {
        throw std::invalid_argument("Circle radius must be in [1, 10000] and period in [1, 3600]");
    }
    radius_ = radius;
    angularVelocity_ = 2.0 * std::acos(-1.0) / period;
    state_ = State{};
    state_.trueVelocity = {0.0, radius_ * angularVelocity_};
    state_.estimatedVelocity = state_.trueVelocity;
    mode_ = MotionMode::Circle;
}

Vector2 Simulation::acceleration() const
{
    if (mode_ == MotionMode::ConstantAcceleration) {
        return acceleration_;
    }
    const double angle = angularVelocity_ * state_.time;
    const double magnitude = radius_ * angularVelocity_ * angularVelocity_;
    return {-magnitude * std::cos(angle), -magnitude * std::sin(angle)};
}

MotionMode Simulation::mode() const
{
    return mode_;
}

void Simulation::step()
{
    if (mode_ == MotionMode::ConstantAcceleration) {
        integrate(state_.truePosition, state_.trueVelocity, acceleration_);
        integrate(state_.estimatedPosition, state_.estimatedVelocity,
                  {acceleration_.x + drift, acceleration_.y + drift});
    } else {
        const double angle = angularVelocity_ * state_.time;
        const double delta = angularVelocity_ * timeStep;
        const double halfSin = std::sin(delta / 2.0);
        const double cosMinusOne = -2.0 * halfSin * halfSin;
        const double sinMinusDelta = std::sin(delta) - delta;
        // Exact integrals of commanded centripetal acceleration over this step.
        // Each branch retains its own velocity, including after correction.
        const Vector2 positionIncrement{
            radius_ * (std::cos(angle) * cosMinusOne - std::sin(angle) * sinMinusDelta),
            radius_ * (std::sin(angle) * cosMinusOne + std::cos(angle) * sinMinusDelta)};
        const Vector2 velocityIncrement{
            -2.0 * radius_ * angularVelocity_ * halfSin * std::cos(angle + delta / 2.0),
            -2.0 * radius_ * angularVelocity_ * halfSin * std::sin(angle + delta / 2.0)};
        integrate(state_.truePosition, state_.trueVelocity, {});
        integrate(state_.estimatedPosition, state_.estimatedVelocity, {drift, drift});
        for (auto* position : {&state_.truePosition, &state_.estimatedPosition}) {
            position->x += positionIncrement.x;
            position->y += positionIncrement.y;
        }
        for (auto* velocity : {&state_.trueVelocity, &state_.estimatedVelocity}) {
            velocity->x += velocityIncrement.x;
            velocity->y += velocityIncrement.y;
        }
    }
    state_.time += timeStep;
}

void Simulation::correct()
{
    state_.estimatedPosition = state_.truePosition;
    // The assignment explicitly resets the estimated velocity, not the true velocity.
    state_.estimatedVelocity = {};
}

void Simulation::reset()
{
    state_ = State{};
    acceleration_ = {};
    mode_ = MotionMode::ConstantAcceleration;
    radius_ = 0.0;
    angularVelocity_ = 0.0;
}

double Simulation::error() const
{
    return std::hypot(state_.estimatedPosition.x - state_.truePosition.x,
                      state_.estimatedPosition.y - state_.truePosition.y);
}

} // namespace navigation
