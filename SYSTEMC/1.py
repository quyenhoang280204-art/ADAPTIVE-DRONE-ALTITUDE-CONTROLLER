import numpy as np
import matplotlib.pyplot as plt

# ================== PARAMETERS ==================
DT = 0.02                  # 20ms
TARGET_ALT = 15.0          # cm
KP = 22.0
KI = 1.8
KD = 4.0
MASS = 0.15                # kg
GRAVITY = 9.81
THRUST_SCALE = 0.02
DRAG_COEFF = 0.1
NOISE_AMPLITUDE = 0.3      # cm

def get_pitch_disturbance(t):
    return 10.0 * np.sin(0.4 * np.pi * t)  # 0.2 Hz

# ================== SIMPLE KALMAN FILTER ==================
class SimpleKalmanFilter:
    def __init__(self, mea_e=2.0, est_e=2.0, q=0.1):
        self.err_measure = mea_e
        self.err_estimate = est_e
        self.q = q
        self.current_estimate = 0.0
        self.last_estimate = 0.0
    
    def updateEstimate(self, mea):
        kg = self.err_estimate / (self.err_estimate + self.err_measure)
        self.current_estimate = self.last_estimate + kg * (mea - self.last_estimate)
        self.err_estimate = (1.0 - kg) * self.err_estimate + abs(self.last_estimate - self.current_estimate) * self.q
        self.last_estimate = self.current_estimate
        return self.current_estimate

# ================== SIMULATION ==================
sim_time = 30.0
steps = int(sim_time / DT)
time = np.arange(0, sim_time, DT)

altitude = np.zeros(steps)          # thực tế
velocity = np.zeros(steps)
thrust_base = np.zeros(steps)       # từ PID
thrust_corrected = np.zeros(steps)  # sau bù tilt
pitch = np.zeros(steps)
measured_alt = np.zeros(steps)      # sau Kalman

kalman = SimpleKalmanFilter()

integral = 0.0
prev_error = 0.0

np.random.seed(42)  # để kết quả lặp lại được

# Khởi tạo
altitude[0] = 0.0
velocity[0] = 0.0
measured_alt[0] = 0.0

for i in range(1, steps):
    t = time[i]
    
    # PID Controller
    error = TARGET_ALT - measured_alt[i-1]
    integral += error * DT
    integral = max(-100.0, min(100.0, integral))  # anti-windup
    derivative = (error - prev_error) / DT
    prev_error = error
    
    thrust_b = KP * error + KI * integral + KD * derivative
    thrust_b = max(0.0, min(255.0, thrust_b))     # clamp PWM
    thrust_base[i] = thrust_b
    
    # Tilt Compensation
    pitch_deg = get_pitch_disturbance(t)
    pitch_rad = np.deg2rad(pitch_deg)
    cos_pitch = np.cos(pitch_rad)
    thrust_c = thrust_b / cos_pitch if cos_pitch > 1e-6 else thrust_b
    thrust_corrected[i] = thrust_c
    pitch[i] = pitch_deg
    
    # Plant Dynamics
    thrust_force = thrust_c * THRUST_SCALE
    net_force = thrust_force - MASS * GRAVITY - DRAG_COEFF * velocity[i-1]
    accel = net_force / MASS
    
    velocity[i] = velocity[i-1] + accel * DT
    altitude[i] = altitude[i-1] + velocity[i] * DT
    
    if altitude[i] < 0:
        altitude[i] = 0.0
        velocity[i] = 0.0
    
    # Measurement noise + Kalman
    noise = NOISE_AMPLITUDE * (np.random.rand() - 0.5) * 2.0
    raw_meas = altitude[i] + noise
    measured_alt[i] = kalman.updateEstimate(raw_meas)

# Gán giá trị bước 0 cho các mảng khác
thrust_base[0] = thrust_base[1]
thrust_corrected[0] = thrust_corrected[1]
pitch[0] = get_pitch_disturbance(0)

# ================== PLOTTING ==================
plt.figure(figsize=(12, 10))

plt.subplot(3, 1, 1)
plt.plot(time, measured_alt, label='Độ cao đo (Kalman)', color='blue')
plt.plot(time, altitude, label='Độ cao thực tế', color='green', alpha=0.7)
plt.axhline(TARGET_ALT, color='red', linestyle='--', label='Mục tiêu 15 cm')
plt.ylabel('Độ cao (cm)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 2)
plt.plot(time, thrust_corrected, label='Thrust đã bù tilt', color='purple')
plt.plot(time, thrust_base, label='Thrust cơ bản (PID)', color='orange', alpha=0.7)
plt.ylabel('Thrust (tương đương PWM)')
plt.legend()
plt.grid(True)

plt.subplot(3, 1, 3)
plt.plot(time, pitch, label='Góc pitch nhiễu (°)', color='brown')
plt.ylabel('Pitch (°)')
plt.xlabel('Thời gian (giây)')
plt.legend()
plt.grid(True)

plt.suptitle('Mô phỏng giữ độ cao drone với PID + Bù tilt + Kalman filter')
plt.tight_layout()
plt.show()