#pragma once
#include <optional>
#include <random>

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
    Vector2 filteredPosition{300.0, 200.0};
    Vector2 filteredVelocity{};
    double time = 0.0;
};

enum class MotionMode { ConstantAcceleration, Circle };

class Simulation {
public:
    static constexpr double timeStep = 0.05;
    static constexpr double drift = 0.02;
    static constexpr int measurementSteps = 4;
    static constexpr double measurementInterval = measurementSteps * timeStep;

    const State& state() const;
    void start(Vector2 acceleration);
    void configureCircle(double radius, double period);
    Vector2 acceleration() const;
    MotionMode mode() const;
    void step();
    void correct();
    void reset();
    double error() const;
    double filteredError() const;
    void setCorrectionEnabled(bool enabled);
    bool correctionEnabled() const;
    void setExternalAvailable(bool available);
    bool externalAvailable() const;
    void setFilterParameters(double noise, double responseTime);
    const std::optional<Vector2>& externalMeasurement() const;

private:
    void updateFilter();
    State state_{};
    Vector2 acceleration_{};
    MotionMode mode_ = MotionMode::ConstantAcceleration;
    double radius_ = 0.0;
    double angularVelocity_ = 0.0;
    bool correctionEnabled_ = true;
    bool externalAvailable_ = true;
    double measurementNoise_ = 0.5;
    double responseTime_ = 2.0;
    int measurementStep_ = 0;
    std::optional<Vector2> externalMeasurement_;
    std::mt19937 random_{42};
    std::normal_distribution<double> noise_{0.0, 1.0};
};

} // namespace navigation
