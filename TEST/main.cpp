#include <systemc.h>
#include <iostream>

// Module GPIO đơn giản
SC_MODULE(SimpleGPIO) {
    sc_in<bool> clk;
    sc_out<bool> led;  // GPIO điều khiển LED
    sc_in<bool> button; // GPIO đọc nút nhấn

    int counter;

    SC_CTOR(SimpleGPIO) {
        counter = 0;
        SC_METHOD(update_led);
        sensitive << clk.pos();
    }

    void update_led() {
        counter++;
        
        // LED nhấp nháy mỗi 5 chu kỳ clock
        if (counter % 10 == 0) {
            led.write(!led.read());
            std::cout << "Time: " << sc_time_stamp() 
                      << " - LED: " << (led.read() ? "ON" : "OFF") 
                      << " - Button: " << (button.read() ? "PRESSED" : "RELEASED")
                      << std::endl;
        }

        // Phản ứng khi nút được nhấn
        if (button.read()) {
            std::cout << "BUTTON PRESSED DETECTED at " << sc_time_stamp() << std::endl;
        }
    }
};

// Module WiFi đơn giản
SC_MODULE(SimpleWiFi) {
    sc_in<bool> clk;
    sc_out<bool> wifi_connected;
    sc_out<bool> data_ready;

    int wifi_counter;

    SC_CTOR(SimpleWiFi) {
        wifi_counter = 0;
        SC_METHOD(wifi_process);
        sensitive << clk.pos();
    }

    void wifi_process() {
        wifi_counter++;

        // Mô phỏng kết nối WiFi sau 20 chu kỳ
        if (wifi_counter == 20) {
            wifi_connected.write(true);
            std::cout << "*** WiFi CONNECTED at " << sc_time_stamp() << " ***" << std::endl;
        }

        // Mô phỏng nhận dữ liệu mỗi 15 chu kỳ
        if (wifi_counter > 20 && wifi_counter % 15 == 0) {
            data_ready.write(true);
            std::cout << "WiFi Data Received at " << sc_time_stamp() << std::endl;
        } else {
            data_ready.write(false);
        }
    }
};

// Module chính ESP32
SC_MODULE(SimpleESP32) {
    sc_in<bool> clk;
    sc_in<bool> reset;

    // Tín hiệu nội bộ
    sc_signal<bool> led_status;
    sc_signal<bool> button_status;
    sc_signal<bool> wifi_connected;
    sc_signal<bool> wifi_data;

    // Các module con
    SimpleGPIO *gpio;
    SimpleWiFi *wifi;

    SC_CTOR(SimpleESP32) {
        // Tạo các module con
        gpio = new SimpleGPIO("GPIO");
        wifi = new SimpleWiFi("WiFi");

        // Kết nối GPIO
        gpio->clk(clk);
        gpio->led(led_status);
        gpio->button(button_status);

        // Kết nối WiFi
        wifi->clk(clk);
        wifi->wifi_connected(wifi_connected);
        wifi->data_ready(wifi_data);

        // Process chính
        SC_METHOD(main_process);
        sensitive << clk.pos();
        async_reset_signal_is(reset, true);
    }

    ~SimpleESP32() {
        delete gpio;
        delete wifi;
    }

    void main_process() {
        if (reset.read()) {
            std::cout << "ESP32 RESET at " << sc_time_stamp() << std::endl;
            button_status.write(false);
        } else {
            // Mô phỏng nút được nhấn tại thời điểm cụ thể
            if (sc_time_stamp() == sc_time(150, SC_NS)) {
                button_status.write(true);
                std::cout << ">>> Button pressed at " << sc_time_stamp() << " <<<" << std::endl;
            }
            if (sc_time_stamp() == sc_time(180, SC_NS)) {
                button_status.write(false);
            }

            // Hiển thị trạng thái hệ thống
            if (sc_time_stamp().to_double() > 0) {
                std::cout << "ESP32 Running - Time: " << sc_time_stamp();
                std::cout << " | LED: " << (led_status.read() ? "ON" : "OFF");
                std::cout << " | WiFi: " << (wifi_connected.read() ? "CONNECTED" : "DISCONNECTED");
                std::cout << " | Data: " << (wifi_data.read() ? "READY" : "NONE");
                std::cout << std::endl;
            }
        }
    }
};

int sc_main(int argc, char* argv[]) {
    std::cout << "=========================================" << std::endl;
    std::cout << "    SIMPLE ESP32 SYSTEMC SIMULATION" << std::endl;
    std::cout << "=========================================" << std::endl;

    // Tạo clock 100MHz (10ns period)
    sc_clock clk("clk", 10, SC_NS);
    sc_signal<bool> reset;

    // Tạo ESP32
    SimpleESP32 esp32("ESP32");
    esp32.clk(clk);
    esp32.reset(reset);

    // Bắt đầu simulation
    std::cout << "\nStarting Simulation..." << std::endl;
    
    // Reset sequence
    reset.write(true);
    sc_start(50, SC_NS);
    reset.write(false);
    std::cout << "Reset released at " << sc_time_stamp() << std::endl;

    // Chạy simulation trong 500ns
    sc_start(500, SC_NS);

    std::cout << "\n=========================================" << std::endl;
    std::cout << "    SIMULATION FINISHED" << std::endl;
    std::cout << "=========================================" << std::endl;

    return 0;
}