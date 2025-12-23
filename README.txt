Ý nghĩa tổng thể của chương trình
Chương trình mô phỏng toàn bộ vòng điều khiển kín (closed-loop control) của một drone bay ở độ cao cố định (15 cm), dù có:

Nhiễu từ cảm biến đo độ cao (giống HC-SR04 siêu âm)
Góc nghiêng pitch dao động theo thời gian (mô phỏng gió thổi hoặc rung động, giống MPU6050)

Mục tiêu: Drone tự động giữ độ cao ổn định ≈ 15 cm mà không bị rơi hay dao động mạnh.
Cấu trúc chương trình gồm các module chính (tương đương các block trong Simulink)

PID_Controller (Bộ điều khiển PID)
Nhận độ cao đo được (altitude_in)
Tính lỗi = 15.0 – độ cao hiện tại
Áp dụng công thức PID (P=22, I=1.8, D=4) để tính thrust_base (lệnh đẩy cơ bản, dạng PWM 0-255)
Có clamp (giới hạn) và anti-windup đơn giản để tránh tích phân quá lớn.

TiltCompensation (Bù nghiêng – mô phỏng xử lý từ MPU6050)
Nhận thrust_base từ PID
Tính góc pitch hiện tại theo hàm sine (10° amplitude, tần số 0.2 Hz → mô phỏng drone bị gió thổi lắc nhẹ)
Tính cos(pitch)
Thrust_corrected = thrust_base / cos(pitch)
→ Khi drone nghiêng, cos(pitch) < 1 → tổng lực đẩy phải tăng lên để giữ lực thẳng đứng không đổi.
Đây chính là phần quan trọng nhất để drone không bị mất độ cao khi nghiêng!

Plant (Mô hình động lực học vật lý của drone – plant)
Nhận thrust_corrected (sau bù)
Chuyển thành lực đẩy thực tế: thrust_force = thrust_pwm × 0.02 
Tính lực net: F_net = thrust_force − trọng lực (m×g = 1.47 N) − lực cản không khí (drag = 0.1 × velocity)
Tính gia tốc = F_net / mass
Tích phân 2 lần (Euler method, bước DT=0.02s): gia tốc → vận tốc → độ cao thực (altitude)
Không cho độ cao < 0 (drone không chui xuống đất)
Thêm nhiễu ngẫu nhiên ±0.3 cm vào độ cao đo được → mô phỏng nhiễu của cảm biến HC-SR04

Monitor
In ra console theo thời gian thực: thời gian, độ cao đo được, lực đẩy đã bù , góc pitch hiện tại
Giúp quan sát hành vi hệ thống


Ban đầu: altitude ≈ 0 → lỗi lớn → thrust tăng mạnh → drone bay lên nhanh.
Khoảng 3-6 giây: altitude đạt ≈15 cm và ổn định (có rung nhẹ do nhiễu).
Khi góc pitch dao động ±10°: thrust_corrected tự động tăng/giảm nhẹ (khoảng ±1-2%) để bù → độ cao hầu như không bị ảnh hưởng.
Nếu không có phần TiltCompensation: độ cao sẽ dao động mạnh theo nhịp sine → drone "lắc lư" theo gió.
