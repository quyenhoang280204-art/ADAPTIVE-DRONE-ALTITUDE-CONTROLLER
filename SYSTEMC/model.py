import numpy as np
import matplotlib.pyplot as plt
import random
import time

# Set seed for reproducibility, similar to SystemC
random.seed(int(time.time()))

# Parameters
DT = 0.02
TARGET_ALT = 15.0  # cm
KP = 22.0
KI = 1.8
KD = 4.0
MASS = 0.15  # kg
GRAVITY = 9.81
THRUST_SCALE = 0.02
DRAG_COEFF = 0.1
NOISE_AMPLITUDE = 0.3  # cm

# Pitch disturbance function
def get_pitch_disturbance(t):
    return 10.0 * np.sin(0.4 * np.pi * t)  # freq = 0.2 Hz

# Simulation settings
total_time =45.0
num_steps = int(total_time / DT) + 1
t_array = np.arange(0, total_time + DT / 2, DT)  # To include 30.0

# Initialize variables
integral = 0.0
prev_error = 0.0
velocity = 0.0
altitude = 0.0  # true altitude
measured_alt = 0.0  # initial measured

# Lists to store data for plotting
time_list = []
alt_true_list = []
alt_measured_list = []
thrust_base_list = []
thrust_corrected_list = []
pitch_list = []

print("Starting simulation for 30 seconds...")
print("Time(s) | Alt(cm) | Thrust_corr | Pitch(deg)")

for i in range(num_steps):
    t = t_array[i]
    
    # PID Controller
    error = TARGET_ALT - measured_alt
    integral += error * DT
    # Simple anti-windup
    integral = max(-100.0, min(100.0, integral))
    derivative = (error - prev_error) / DT if i > 0 else 0.0
    prev_error = error
    thrust_base = KP * error + KI * integral + KD * derivative
    thrust_base = max(0.0, min(255.0, thrust_base))
    
    # Tilt Compensation
    pitch_deg = get_pitch_disturbance(t)
    pitch_rad = pitch_deg * np.pi / 180.0
    cos_pitch = np.cos(pitch_rad)
    thrust_corrected = thrust_base / cos_pitch if cos_pitch != 0 else thrust_base  # Avoid division by zero
    
    # Plant dynamics
    thrust_force = thrust_corrected * THRUST_SCALE
    net_force = thrust_force - MASS * GRAVITY - DRAG_COEFF * velocity
    accel = net_force / MASS
    velocity += accel * DT
    altitude += velocity * DT
    if altitude < 0:
        altitude = 0
        velocity = 0
    
    # Add noise
    noise = NOISE_AMPLITUDE * (random.random() - 0.5) * 2.0
    measured_alt = altitude + noise
    
    # Store data
    time_list.append(t)
    alt_true_list.append(altitude)
    alt_measured_list.append(measured_alt)
    thrust_base_list.append(thrust_base)
    thrust_corrected_list.append(thrust_corrected)
    pitch_list.append(pitch_deg)
    
    # Monitor print every 0.5s (25 steps)
    step_count = int(t / DT)
    if step_count % 25 == 0:
        print(f"{t:.3f} | {measured_alt:.3f} | {thrust_corrected:.3f} | {pitch_deg:.3f}")

print("Simulation finished.")

# Plot results similar to MATLAB PDF
fig, ax1 = plt.subplots(figsize=(10, 6))

ax1.plot(time_list, alt_true_list, 'b-', label='True Altitude (cm)')
ax1.plot(time_list, alt_measured_list, 'g--', label='Measured Altitude (cm)')
ax1.axhline(TARGET_ALT, color='r', linestyle='--', label='Desired Altitude')
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Altitude (cm)')
ax1.set_title('Drone Altitude Control using PID with Tilt Disturbance')
ax1.legend(loc='upper left')
ax1.grid(True)

# Second axis for pitch
ax2 = ax1.twinx()
ax2.plot(time_list, pitch_list, 'm-', label='Pitch (deg)', alpha=0.5)
ax2.set_ylabel('Pitch (deg)')
ax2.legend(loc='upper right')

plt.show()

# Display PID values like PDF
print('PID Parameters Used:')
print(f'Kp = {KP:.2f}, Ki = {KI:.2f}, Kd = {KD:.2f}')