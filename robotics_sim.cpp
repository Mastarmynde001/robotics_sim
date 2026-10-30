#include <iostream>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <cmath>
#include <cstdint>
#include <string>
#include <random>


#ifdef PYBIND11_BUILD
#include <pybind11/pybind11.h>
namespace py = pybind11;
#endif

struct Vector3 {
    double x, y, z;
    Vector3(double x_val = 0.0, double y_val = 0.0, double z_val = 0.0) : x(x_val), y(y_val), z(z_val) {}

    double distanceToSquared(const Vector3& target) const {
        const double dx = target.x - x;
        const double dy = target.y - y;
        const double dz = target.z - z;
        return dx * dx + dy * dy + dz * dz;
    }

    double distanceTo(const Vector3& target) const {
        return std::sqrt(distanceToSquared(target));
    }
};

class Robot {
private:
    Vector3 position;
    Vector3 velocity;
    double mass;

public:
    Robot(const Vector3& pos, double m) : position(pos), velocity(Vector3()), mass(m) {}

    const Vector3& getPosition() const {
        return position;
    }

    void applyForce(const Vector3& force, double deltaTime) {
        
        Vector3 acceleration(force.x / mass, force.y / mass, force.z / mass);

        velocity.x += (acceleration.x * deltaTime);
        velocity.y += (acceleration.y * deltaTime);
        velocity.z += (acceleration.z * deltaTime);
        
        velocity.x *= 0.98;
        velocity.y *= 0.98;
        velocity.z *= 0.98;

        position.x += (velocity.x * deltaTime);
        position.y += (velocity.y * deltaTime);
        position.z += (velocity.z * deltaTime);
    }   
};

class PositionSensor {
private:
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

public:
    PositionSensor(double stddev) : distribution(0.0, stddev) {
        std::random_device rd;
        generator.seed(rd());
    }

    Vector3 readPosition(const Vector3& clean) {
        return Vector3(
            clean.x + distribution(generator),
            clean.y + distribution(generator),
            clean.z + distribution(generator)
        );
    }
};

class KalmanFilter3D {
private:
    // State arrays for each axis: [position, velocity]
    double x[2], y[2], z[2];
    // Error covariance matrices (2x2) for each axis
    double Px[2][2], Py[2][2], Pz[2][2];
    
    // Tuning parameters
    double Q_pos, Q_vel; // Process noise covariance (model uncertainty)
    double R_pos;        // Measurement noise covariance (sensor variance)
    bool initialized;

    void predict(double* state, double P[2][2], double dt) {
        // State Transition: pos = pos + vel * dt
        state[0] += state[1] * dt;
        
        // Covariance Transition: P = A * P * A^T + Q
        double p00 = P[0][0] + dt * (P[1][0] + P[0][1]) + dt * dt * P[1][1] + Q_pos;
        double p01 = P[0][1] + dt * P[1][1];
        double p10 = P[1][0] + dt * P[1][1];
        double p11 = P[1][1] + Q_vel;
        
        P[0][0] = p00; P[0][1] = p01; P[1][0] = p10; P[1][1] = p11;
    }

    void update(double* state, double P[2][2], double measurement) {
        // Innovation covariance: S = H * P * H^T + R
        double S = P[0][0] + R_pos;
        
        // Kalman Gain: K = P * H^T * S^-1
        double K[2] = {P[0][0] / S, P[1][0] / S};
        
        // Measurement residual
        double y_res = measurement - state[0];
        
        // State Update
        state[0] += K[0] * y_res;
        state[1] += K[1] * y_res;
        
        // Covariance Update: P = (I - K * H) * P
        double p00 = (1.0 - K[0]) * P[0][0];
        double p01 = (1.0 - K[0]) * P[0][1];
        double p10 = -K[1] * P[0][0] + P[1][0];
        double p11 = -K[1] * P[0][1] + P[1][1];
        
        P[0][0] = p00; P[0][1] = p01; P[1][0] = p10; P[1][1] = p11;
    }

public:
    KalmanFilter3D(double q_p, double q_v, double r_p) 
        : Q_pos(q_p), Q_vel(q_v), R_pos(r_p), initialized(false) {}

    void reset() { initialized = false; }

    Vector3 filter(const Vector3& measurement, double dt) {
        if (!initialized) {
            x[0] = measurement.x; x[1] = 0.0;
            y[0] = measurement.y; y[1] = 0.0;
            z[0] = measurement.z; z[1] = 0.0;

            for (int i = 0; i < 2; ++i) {
                for (int j = 0; j < 2; ++j) {
                    Px[i][j] = (i == j) ? 1.0 : 0.0;
                    Py[i][j] = (i == j) ? 1.0 : 0.0;
                    Pz[i][j] = (i == j) ? 1.0 : 0.0;
                }
            }
            initialized = true;
            return measurement;
        }

        // 1. Predict step (Kinematic estimation)
        predict(x, Px, dt);
        predict(y, Py, dt);
        predict(z, Pz, dt);

        // 2. Update step (Sensor correction)
        update(x, Px, measurement.x);
        update(y, Py, measurement.y);
        update(z, Pz, measurement.z);

        return Vector3(x[0], y[0], z[0]); // Return optimal position estimate
    }
};

class PIDController {
private:
    double kp, ki, kd;
    double max_integral;
    double max_force;

    Vector3 integral, prev_error;

public:
    PIDController(double kp_val, double ki_val, double kd_val, double max_i, double max_f) 
        : kp(kp_val), ki(ki_val), kd(kd_val), max_integral(max_i), max_force(max_f) {}

    void setGains(double kp_val, double ki_val, double kd_val) {
        kp = kp_val;
        ki = ki_val;
        kd = kd_val;
    }

    void setLimits(double max_i, double max_f) {
        max_integral = max_i;
        max_force = max_f;
    }

    void reset() {
        integral = Vector3(0.0, 0.0, 0.0);
        prev_error = Vector3(0.0, 0.0, 0.0);
    }

    Vector3 calculate(const Vector3& target, const Vector3& current, double dt) {
        Vector3 error(target.x - current.x, target.y - current.y, target.z - current.z);
        Vector3 proportional(error.x * kp, error.y * kp, error.z * kp);

        integral.x = std::clamp(integral.x + (error.x * dt) * ki, -max_integral, max_integral);
        integral.y = std::clamp(integral.y + (error.y * dt) * ki, -max_integral, max_integral);
        integral.z = std::clamp(integral.z + (error.z * dt) * ki, -max_integral, max_integral);

        Vector3 derivative(
            ((error.x - prev_error.x) * kd) / dt,
            ((error.y - prev_error.y) * kd) / dt,
            ((error.z - prev_error.z) * kd) / dt
        );

        prev_error = error;

        Vector3 final_force(
            std::clamp(proportional.x + integral.x + derivative.x, -max_force, max_force),
            std::clamp(proportional.y + integral.y + derivative.y, -max_force, max_force),
            std::clamp(proportional.z + integral.z + derivative.z, -max_force, max_force)
        );

        return final_force;
    }
};

#ifdef PYBIND11_BUILD
PYBIND11_MODULE(robotics_sim, m) {
    m.doc() = "C++ Robotics Math Engine with Pybind11";

    py::class_<Vector3>(m, "Vector3")
        .def(py::init<double, double, double>())
        .def_readwrite("x", &Vector3::x)
        .def_readwrite("y", &Vector3::y)
        .def_readwrite("z", &Vector3::z)
        .def("distance_to", &Vector3::distanceTo);

    py::class_<Robot>(m, "Robot")
        .def(py::init<const Vector3&, double>())
        .def("get_position", &Robot::getPosition)
        .def("apply_force", &Robot::applyForce);

    py::class_<PositionSensor>(m, "PositionSensor")
        .def(py::init<double>())
        .def("read_position", &PositionSensor::readPosition);

    py::class_<KalmanFilter3D>(m, "KalmanFilter3D")
        .def(py::init<double, double, double>())
        .def("filter", &KalmanFilter3D::filter)
        .def("reset", &KalmanFilter3D::reset);

    py::class_<PIDController>(m, "PIDController")
        .def(py::init<double, double, double, double, double>())
        .def("calculate", &PIDController::calculate)
        .def("set_gains", &PIDController::setGains) 
        .def("set_limits", &PIDController::setLimits) 
        .def("reset", &PIDController::reset); 
}
#endif