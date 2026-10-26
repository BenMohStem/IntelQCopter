// Executes the actual firmware with fake peripherals. This is NOT ESP32 emulation.
#include "arduino_stub/fake_board.hpp"
#include "../../firmware/main.cpp"
#include <iostream>

int main(int argc, char **argv)
{
    assert(argc == 2);
    const std::string test = argv[1];
    retained_boot = test == "retained";
    invalid_boot = test == "invalid_boot";
    invalid_gyro_boot = test == "invalid_gyro_boot";
    corrupt_trim_writes = test == "trim_readback_failure";
    if (test == "basis_x" || test == "basis_y" || test == "basis_z")
    {
        model_gyro_counts[0] = test == "basis_x" ? 131 : 0;
        model_gyro_counts[1] = test == "basis_y" ? 131 : 0;
        model_gyro_counts[2] = test == "basis_z" ? 131 : 0;
    }
    setup();
    const int required_input_pins[] = {1, 2, 4, 12, 14, 21};
    assert(pin_calls == 6);
    assert(pin_mode_calls.size() == 6);
    for (int pin : required_input_pins)
    {
        unsigned matches = 0;
        for (const PinModeCall &call : pin_mode_calls)
        {
            assert(call.mode == INPUT_PULLDOWN);
            if (call.pin == pin)
            {
                matches++;
            }
        }
        assert(matches == 1);
    }
    if (invalid_boot)
    {
        assert(!sensor_ready);
        assert(get_word(0x06) == -999);
        std::cerr << "PASS invalid boot rejected before trim writes\n";
        return 0;
    }
    if (invalid_gyro_boot || corrupt_trim_writes)
    {
        assert(!sensor_ready);
        assert(Serial.output.find("ERROR,") != std::string::npos);
        std::cerr << "PASS " << test << " rejected during initialization\n";
        return 0;
    }
    assert(sensor_ready);
    assert(get_word(0x06) == -2348 && get_word(0x08) == 2993 && get_word(0x0A) == 1212);
    assert(get_word(0x13) == -6 && get_word(0x15) == 9 && get_word(0x17) == 27);
    if (test == "wrap")
    {
        model_time_us = 0xFFFFFFFFull - 500000;
        next_packet_us = model_time_us + 1000;
        fifo_model.clear();
    }
    Serial.input.push_back('?');
    loop();
    reject_fifo_enable = test == "fifo_enable_failure";
    Serial.input.push_back('s');
    loop();
    if (reject_fifo_enable)
    {
        assert(!sensor_ready && !recording);
        assert(Serial.output.find("ERROR,FIFO_ENABLE_FAILED") != std::string::npos);
        std::cerr << "PASS FIFO enable failure stops acquisition\n";
        return 0;
    }
    assert(recording);
    if (test == "overflow") { fifo_model.resize(1024); overflow_model = true; }
    if (test == "i2c_failure") { fail_reads = true; }
    if (test == "fifo_packet_failure") { failed_read_address = 0x74; }
    if (test == "offset_change") { set_word(0x13, -4); }
    if (test == "queue_full") { force_queue_full = true; }
    if (test == "logger_drain_failure") { force_barrier_failure = true; }
    if (test == "slow_bus") { bus_cost_multiplier = 3; }
    if (test == "timeout") { advance_model(25000001); }
    if (test == "abort") { Serial.input.push_back('x'); }
    unsigned iterations = 0;
    while (recording && iterations++ < 200000) { loop(); }
    assert(!recording);
    assert(iterations < 200000);
    const bool success = test == "normal" || test == "wrap" || test == "retained" || test == "basis_x" || test == "basis_y" || test == "basis_z";
    if (success)
    {
        assert(sensor_ready && sequence == 20000);
        assert(Serial.output.find("END,20000,") != std::string::npos);
        assert(latest_drone_accel[2] > 8 && std::isfinite(latest_pitch_deg));
        std::cout << Serial.output;
    }
    else
    {
        assert(!sensor_ready);
        assert(Serial.output.find("ERROR,") != std::string::npos);
        assert(Serial.output.find("END,20000,") == std::string::npos);
        std::cout << Serial.output;
    }
    std::cerr << "PASS " << test << " (HOST PERIPHERAL MODEL, NOT HARDWARE TIMING)\n";
    return 0;
}
