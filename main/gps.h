#pragma once

#include <stdint.h>
#include <string>

struct gps_state_t {
    bool valid_fix = false;
    int satellites = 0;
    std::string time_utc;    // "HH:MM:SS"
    std::string date_utc;    // "YYYY-MM-DD"
    double latitude = 0.0;
    double longitude = 0.0;
    std::string grid_square; // "CM97"
    uint32_t last_rx_ms = 0; // last received decodable NMEA sentence
    int active_baud = 0;
    bool baud_locked = false;
    bool running = false;
};

// Start GPS parser on UART1 PortA pins with preload baud (9600 or 115200).
void gps_start(int preload_baud);

// Stop GPS parser and release UART1.
void gps_stop();

// Periodic housekeeping hook (lightweight; safe to call each loop).
void gps_tick();

// Current state snapshot.
gps_state_t gps_get_state();

// One-shot event: returns true once when auto-baud locks to a new baud.
bool gps_take_baud_update(int* out_baud);
