#pragma once

namespace navigation {

struct Vector2 {
    double x = 0.0;
    double y = 0.0;
};

struct State {
    Vector2 truePosition{300.0, 200.0};
    Vector2 estimatedPosition{300.0, 200.0};
    Vector2 trueVelocity{};
    Vector2 estimatedVelocity{};
    double time = 0.0;
};

enum class MotionMode { ConstantAcceleration, Circle };

class Simulation {
public:
    static constexpr double timeStep = 0.05;
    static constexpr double drift = 0.02;

    const State& state() const;
    void start(Vector2 acceleration);
    void configureCircle(double radius, double period);
    Vector2 acceleration() const;
    MotionMode mode() const;
    void step();
    void correct();
    void reset();
    double error() const;

private:
    State state_{};
    Vector2 acceleration_{};
    MotionMode mode_ = MotionMode::ConstantAcceleration;
    double radius_ = 0.0;
    double angularVelocity_ = 0.0;
};

} // namespace navigation
