/*

Main module

# Reworked by Davide Foschi, based on modified code by Kyle T. Gabriel to fix issue with incorrect GPS data for TTNMapper

Copyright (C) 2025 by Davide Foschi, based on code by Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "configuration.h"
#include <TinyGPS++.h>
#include <Wire.h>
#include <Bounce2.h>
#include "esp_task_wdt.h"
#include "esp_sleep.h"
#include "CircularBuffer.hpp"

bool board_ready, gps_fixed, packet_queued, ttn_joined = false;
static uint32_t last_read_bat = 0;
CircularBuffer<float, BAT_NUM_READ> bat_readings;
static uint32_t last_transmission = 0;
static bool discard_was_pressed = false;
static bool send_was_pressed = false;
static bool send_by_button = false;
static uint32_t discard_min_press_ms;
static uint32_t send_min_press_ms;

// Power management variables
static bool deep_sleep_enabled = ENABLE_DEEP_SLEEP;
Bounce2::Button discard_button = Bounce2::Button();
Bounce2::Button send_button = Bounce2::Button();

static uint8_t gpsBuffer[10];
static uint8_t aliveBuffer[1];

// -----------------------------------------------------------------------------
// Application
// -----------------------------------------------------------------------------

// needed for platformio
void buildGPSPacket(uint8_t gpsBuffer[]);
void buildAlivePacket(uint8_t aliveBuffer[]);

void enter_deep_sleep(uint32_t sleep_time_ms) {
    // Configure wake-up sources
    esp_sleep_enable_timer_wakeup(sleep_time_ms * 1000ULL);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)SEND_PIN, 1);
    
    // Enter deep sleep
    esp_deep_sleep_start();
}

uint32_t calculate_next_interval() {
    if (gps_is_stationary()) {
        return DEEP_SLEEP_INTERVAL; // Use longer interval when stationary
    }
    return SEND_INTERVAL; // Normal interval when moving
}

bool trySend() {
    // Validate GPS data with proper thresholds
    if (gps_hdop() > 0 && gps_hdop() < MAX_VALID_HDOP && 
        gps_latitude() != 0 && gps_longitude() != 0 && gps_altitude() != 0 &&
        gps_latitude() >= -90.0 && gps_latitude() <= 90.0 &&
        gps_longitude() >= -180.0 && gps_longitude() <= 180.0) {
        char buffer[24];
        snprintf(buffer, sizeof(buffer), "%.4f,%.4f %.0fm", gps_latitude(), gps_longitude(), gps_altitude());
        screen_print("gps", buffer);

        buildGPSPacket(gpsBuffer);
        packet_queued = true;

        ttn_send(gpsBuffer, sizeof(gpsBuffer), LORAWAN_GPS_PORT, 0);
        return true;
    } else {
        #ifdef LORAWAN_SEND_ALIVE
            buildAlivePacket(aliveBuffer);
            packet_queued = true;

            ttn_send(aliveBuffer, sizeof(aliveBuffer), LORAWAN_ALIVE_PORT, 0);
            return true;
        #else
            return false;
        #endif
    }
}

void callback(uint8_t message) {
    if (EV_JOINED == message) {
        ttn_joined = true;
    }
    if (EV_JOINING == message) {
        screen_print("ttn", "TTN joining");
    }
    if (EV_JOIN_FAILED == message) {
        screen_print("ttn", "TTN join failed");
    }
    if (EV_REJOIN_FAILED == message) {
        screen_print("ttn", "TTN rejoin failed");
    }
    if (EV_RESET == message) {
        screen_print("ttn", "Reset TTN connection");
    }
    if (EV_LINK_DEAD == message) {
        screen_print("ttn", "TTN link dead");
    }
    if (EV_ACK == message) {
        screen_print("ttn", "ACK received");
    }
    if (EV_PENDING == message) {
        screen_print("ttn", "Message pending");
    }
    if (EV_QUEUED == message) {
        screen_print("ttn", "Message queued");
    }
    if (EV_TXCANCELED == message) {
        screen_print("ttn", "Message canceled");
    }
    if (EV_TXSTART == message) {
        screen_print("ttn", "TX Start");
        //DEBUG_PORT.println(os_getTime());  // current ticks
        //DEBUG_PORT.println(LMIC.txend);    // when TX should end
    }
    // We only want to say 'packetSent' for our packets (not packets needed for joining)
    if (EV_TXCOMPLETE == message && packet_queued) {
        screen_print("ttn", "Message sent");
        packet_queued = false;
    }

}

void battery_loop(){
    if (0 == last_read_bat || millis() - last_read_bat > BAT_INTERVAL) {

        int vref = 1100;
        uint16_t v = analogRead(BAT_PIN);
        if (v!=0) {

            float bat_voltage = ((float)v / 4095.0) * 2.0 * 3.3 * (vref / 1000.0); // Convert ADC measure to actual voltage
            //DEBUG_PORT.println(bat_voltage);
            bat_readings.push(bat_voltage);
            float bat_values = 0.0;
            for (int i = 0; i < bat_readings.size(); ++i) {
                bat_values = bat_values + bat_readings[i];
            }

            float avg_bat_voltage = bat_values / bat_readings.size();
            //DEBUG_PORT.println(avg_bat_voltage);
            float percent = (avg_bat_voltage - BAT_MIN_V) * 100.0 / (BAT_MAX_V - BAT_MIN_V);
            
            // Clamp between 0% and 100%
            if (percent < 0) percent = 0;
            if (percent > 100) percent = 100;

            char batmgmt[24] = "" ;
            sprintf (batmgmt, "Bat: %.1fV - %.0f%%", avg_bat_voltage, percent);
            screen_print("hw", batmgmt);
        } else {
            screen_print("hw", "No battery!");
        }
        last_read_bat = millis();
    }
}

void setup() {
    // Debug
    DEBUG_PORT.begin(SERIAL_BAUD);

    // Configure and enable watchdog timer
    esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);
    esp_task_wdt_add(NULL);

    // Init I2C
    Wire.begin(I2C_SDA, I2C_SCL);

    // Discard Button
    discard_button.attach(DISCARD_PIN, INPUT_PULLDOWN);
    discard_button.interval(BUTTON_DEBOUNCE_MS);
    discard_button.setPressedState(HIGH);

    // Send Button
    send_button.attach(SEND_PIN, INPUT_PULLDOWN);
    send_button.interval(BUTTON_DEBOUNCE_MS);
    send_button.setPressedState(HIGH);

    // Init Screen
    screen_setup();
    screen_print("hw", "Board not ready yet");
    screen_loop();
    
    os_init();
    LMIC_reset();
    #if defined(CFG_eu868)
    LMIC.dn2Dr = DR_SF9;  // force RX2 to SF9
    LMIC.dn2Freq = 869525000; 
    #endif
    ttn_sf(LORAWAN_SF);

    // TTN setup
    if (!ttn_setup()) {
        screen_print("hw", "Radio module not found");
        screen_loop();
        if (REQUIRE_RADIO) {
            delay(5000);
        }
    } else {
        ttn_register(callback);
        ttn_join();
        ttn_adr(LORAWAN_ADR);
    }
    screen_loop();

    // Init GPS
    screen_print("gps", "Initializing...");
    screen_loop();
    bool gps_device_ok = gps_setup();
    screen_loop();

    if (gps_device_ok) {

        #ifdef GPS_WAIT_FOR_LOCK
        if (gps_wait_for_fix(GPS_WAIT_FOR_LOCK)) {
            gps_update_last_position(); // Set initial reference position
            screen_print("gps", "GPS fix acquired");
        } else {
            screen_print("gps", "GPS fix timeout");
        }
        #endif

        screen_print("hw", "Board Ready");
        board_ready = true;
        screen_loop();
    }

}

void loop() {

    // Feed watchdog timer
    esp_task_wdt_reset();

    discard_button.update();
    send_button.update();

    // if user presses button for more than 3 secs, discard our network prefs and reboot (FIXME, use a debounce lib instead of this boilerplate)
    if (discard_button.isPressed()) {
        if (!discard_was_pressed) {
            // just started a new press
            discard_was_pressed = true;
            discard_min_press_ms = millis() + DISCARD_BUTTON_HOLD_MS;
        } else {
            char btntime[24];
            int remaining_time = int((discard_min_press_ms-millis())/1000)+1;
            if (remaining_time>=0 && remaining_time<10) {
                snprintf(btntime, sizeof(btntime), "Keep pressing for %ds", (remaining_time>=0)?remaining_time:0);
            } else {
                snprintf(btntime, sizeof(btntime), "Release the button");
            }
            screen_print("hw", btntime);
        }
    } else if (discard_was_pressed) {
        // we just did a release
        discard_was_pressed = false;
        if (millis() > discard_min_press_ms) {
            // held long enough
            #ifndef PREFS_DISCARD
                screen_print("hw", "Discarding disabled");
                delay(2000);
            #endif

            #ifdef PREFS_DISCARD
                screen_print("hw", "Resetting");
                screen_print("gps", "----");
                screen_print("ttn", "Discarding prefs");
                ttn_erase_prefs();
                screen_loop();
                delay(5000);  // Give some time to read the screen
                ESP.restart();
            #endif
        }
    } else {
        #ifdef BAT_MONITOR
            battery_loop();
        #else
            screen_print("hw", "Board Ready");
        #endif
    }

    gps_loop();
    ttn_loop();
    screen_loop();
    
    // Enter deep sleep if no pending LoRaWAN operations and stationary
    if (deep_sleep_enabled && gps_is_stationary() && ttn_joined && 
        !(LMIC.opmode & OP_TXRXPEND) && !packet_queued) {
        enter_deep_sleep(DEEP_SLEEP_INTERVAL);
    }

    // if user presses button for more than 1 secs, send message
    if (send_button.isPressed()) {
        if (!send_was_pressed) {
            // just started a new press
            send_was_pressed = true;
            send_min_press_ms = millis() + SEND_BUTTON_HOLD_MS;
        }
    } else if (send_was_pressed) {
        // we just did a release
        send_was_pressed = false;
        if (millis() > send_min_press_ms) {
            send_by_button = true;
        }
    }

    if (ttn_joined == true) {
        // Transmission logic: button press OR (interval passed AND position changed)
        if (send_by_button || 
            ((0 == last_transmission || millis() - last_transmission > SEND_INTERVAL) && gps_has_moved())) {
            
            if (send_by_button) {
                send_by_button = false;
            }

            if (trySend()) {
                last_transmission = millis();
                gps_update_last_position(); // Update reference position after transmission
                screen_print("ttn", "Message queued");
            } else {
                screen_print("ttn", "No GPS fix");
            }
        }
    }

}
