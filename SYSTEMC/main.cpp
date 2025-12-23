#include <systemc.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <ctime>

// ================== PARAMETERS ==================
const double DT = 0.02;          // 20ms
const double TARGET_ALT = 15.0;  // cm
const double KP = 22.0;
const double KI = 1.8;
const double KD = 4.0;
const double MASS = 0.15;        // kg
const double GRAVITY = 9.81;
const double THRUST_SCALE = 0.02;
const double DRAG_COEFF = 0.1;
const double NOISE_AMPLITUDE = 0.3; // cm, giống Simulink

// Pitch disturbance: sine wave 10 độ, 0.2 Hz
double get_pitch_disturbance(double time) {
    return 10.0 * sin(0.4 * M_PI * time); // freq = 0.2 Hz
}

// ================== PID MODULE ==================
SC_MODULE(PID_Controller) {
    sc_in<bool> clk;
    sc_in<double> altitude_in;
    sc_out<double> thrust_base_out;

    double integral = 0.0;
    double prev_error = 0.0;

    void process() {
        while (true) {
            wait(); // wait clock edge

            double current_alt = altitude_in.read();
            double error = TARGET_ALT - current_alt;

            integral += error * DT;
            // Simple anti-windup
            integral = std::max(-100.0, std::min(100.0, integral));

            double derivative = (error - prev_error) / DT;
            prev_error = error;

            double thrust_base = KP * error + KI * integral + KD * derivative;
            thrust_base = std::max(0.0, std::min(255.0, thrust_base)); // clamp

            thrust_base_out.write(thrust_base);
        }
    }

    SC_CTOR(PID_Controller) {
        SC_THREAD(process);
        sensitive << clk.pos();
    }
};

// ================== TILT COMPENSATION ==================
SC_MODULE(TiltCompensation) {
    sc_in<bool> clk;
    sc_in<double> thrust_base_in;
    sc_out<double> thrust_corrected_out;

    void process() {
        while (true) {
            wait();
            double t = sc_time_stamp().to_seconds();
            double pitch_deg = get_pitch_disturbance(t);
            double pitch_rad = pitch_deg * M_PI / 180.0;
            double cos_pitch = cos(pitch_rad);

            double thrust_base = thrust_base_in.read();
            double thrust_corrected = thrust_base / cos_pitch; // bù tilt

            thrust_corrected_out.write(thrust_corrected);
        }
    }

    SC_CTOR(TiltCompensation) {
        SC_THREAD(process);
        sensitive << clk.pos();
    }
};

// ================== PLANT MODULE ==================
SC_MODULE(Plant) {
    sc_in<bool> clk;
    sc_in<double> thrust_in;
    sc_out<double> altitude_out;

    double velocity = 0.0;
    double altitude = 0.0;

    void dynamics() {
        std::srand(std::time(nullptr)); // seed random
        while (true) {
            wait();

            double thrust_pwm = thrust_in.read();
            double thrust_force = thrust_pwm * THRUST_SCALE;

            double net_force = thrust_force - MASS * GRAVITY - DRAG_COEFF * velocity;
            double accel = net_force / MASS;

            velocity += accel * DT;
            altitude += velocity * DT;

            if (altitude < 0) {
                altitude = 0;
                velocity = 0;
            }

            // Band-limited white noise approximation
            double noise = NOISE_AMPLITUDE * ((double)std::rand() / RAND_MAX - 0.5) * 2.0;
            double measured_alt = altitude + noise;

            altitude_out.write(measured_alt);
        }
    }

    SC_CTOR(Plant) {
        SC_THREAD(dynamics);
        sensitive << clk.pos();
    }
};

// ================== MONITOR ==================
SC_MODULE(Monitor) {
    sc_in<bool> clk;
    sc_in<double> alt_in;
    sc_in<double> thrust_in;
    sc_in<double> thrust_corrected_in;

    void print() {
        // Header
        std::cout << "Time(s)   | Alt(cm) | Thrust_corr | Pitch(deg)" << std::endl;
        while (true) {
            wait();
            double t = sc_time_stamp().to_seconds();
            double pitch = get_pitch_disturbance(t);
            std::cout.precision(3);
            std::cout << t << " | "
                      << alt_in.read() << " | "
                      << thrust_corrected_in.read() << " | "
                      << pitch << std::endl;

            // In ít dòng hơn để dễ đọc
            if (((int)(t / DT)) % 25 == 0) { // mỗi 0.5s in 1 lần
                // in liên tục cũng được nếu muốn
            } else {
                continue;
            }
        }
    }

    SC_CTOR(Monitor) {
        SC_THREAD(print);
        sensitive << clk.pos();
    }
};

// ================== TOP ==================
int sc_main(int argc, char* argv[]) {
    sc_clock clk("clk", DT, SC_SEC);

    PID_Controller pid("pid");
    TiltCompensation tilt("tilt");
    Plant plant("plant");
    Monitor monitor("monitor");

    sc_signal<double> thrust_base_sig;
    sc_signal<double> thrust_corrected_sig;
    sc_signal<double> alt_sig;

    // Connections
    pid.clk(clk);
    pid.altitude_in(alt_sig);
    pid.thrust_base_out(thrust_base_sig);

    tilt.clk(clk);
    tilt.thrust_base_in(thrust_base_sig);
    tilt.thrust_corrected_out(thrust_corrected_sig);

    plant.clk(clk);
    plant.thrust_in(thrust_corrected_sig);
    plant.altitude_out(alt_sig);

    monitor.clk(clk);
    monitor.alt_in(alt_sig);
    monitor.thrust_in(thrust_base_sig);
    monitor.thrust_corrected_in(thrust_corrected_sig);

    std::cout << "Starting simulation for 30 seconds..." << std::endl;
    sc_start(30, SC_SEC);

    std::cout << "Simulation finished." << std::endl;
    return 0;
}
