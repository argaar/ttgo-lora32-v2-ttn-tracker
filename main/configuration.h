/*

TTGO Lora32 v2.0 Tracker for The Things Network

Copyright (C) 2025 by Davide Foschi, based on code by Xose Pérez <xose dot perez at gmail dot com>

This code requires the MCCI LoRaWAN LMIC library
by IBM, Matthis Kooijman, Terry Moore, ChaeHee Won, Frank Rose
https://github.com/mcci-catena/arduino-lmic

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

#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <lmic.h>
void ttn_register(void (*callback)(uint8_t message));

// -----------------------------------------------------------------------------
// Version
// -----------------------------------------------------------------------------

#define APP_NAME                "TTGO-LORA32-V2-TTN-TRACKER"
#define APP_VERSION             "2.1.1"

// -----------------------------------------------------------------------------
// Configuration
// -----------------------------------------------------------------------------

// If you are having difficulty sending messages to TTN after the first successful send,
// uncomment the next option and experiment with values (~ 1 - 5)
//#define CLOCK_ERROR             1

#define DEBUG_PORT              Serial          // Serial debug port
#define SERIAL_BAUD             115200          // Serial debug baud rate

// Enable Battery Voltage Monitor
#define BAT_MONITOR

// Timing Constants
#define SEND_INTERVAL           (60 * 1000)     // Default transmission interval
#define DEEP_SLEEP_INTERVAL     (30 * 1000)     // Deep sleep when stationary
#define GPS_TIMEOUT             (120 * 1000)    // GPS fix timeout
#define WATCHDOG_TIMEOUT_S      (30)            // Watchdog timeout in seconds
#define BUTTON_DEBOUNCE_MS      (200)           // Button debounce time
#define COUNT_SAVE_EVERY_M      (2)             // Save frmCount to flash interval
#define BAT_INTERVAL            (30 * 1000)     // Read Battery Voltage interval

// Movement Detection
#define MOVEMENT_THRESHOLD_M    (5.0)           // Movement threshold in meters
#define STATIONARY_CHECK_MS     (5 * 60 * 1000) // Check interval for stationary state

// GPS Validation
#define MAX_VALID_HDOP          (5.0)           // Maximum acceptable HDOP
#define MIN_VALID_COORDS        (-1000.0)       // Minimum coordinate validation

// LoRawan Configuration
#define LORAWAN_SEND_ALIVE                      // Send just an "alive" message in case gps doesn't fix
#define LORAWAN_GPS_PORT        1               // Port the gps messages will be sent to
#define LORAWAN_ALIVE_PORT      2               // Port the alive messages will be sent to
#define LORAWAN_SF              DR_SF7          // Spreading factor (recommended DR_SF7 for ttn network map purposes, DR_SF10 works for slow moving trackers)
#define LORAWAN_ADR             1               // Enable ADR for better network efficiency
#define REQUIRE_RADIO           true            // If true, we will fail to start if the radio is not found
#define MAX_LEN_FRAME           64              // Increase Buffer frames

// Buttons
#define PREFS_DISCARD
#define DISCARD_BUTTON_HOLD_MS  (9 * 1000)      // How long to press button to discard prefs
#define SEND_BUTTON_HOLD_MS     (1 * 1000)      // How long to press button to send

// If not defined, we will go directly to run loop
#define GPS_WAIT_FOR_LOCK       (30 * 1000)     // Wait after every boot for GPS lock (may need longer than 5s because we turned the gps off during deep sleep)

// Custom messages
#define EV_QUEUED               100
#define EV_PENDING              101
#define EV_ACK                  102
#define EV_RESPONSE             103

// PINS
#define I2C_SDA                 21
#define I2C_SCL                 22
#define DISCARD_PIN             4
#define SEND_PIN                2
#define BAT_PIN                 35

// OLED
#define SSD1306_ADDRESS         0x3C

// GPS
#define GPS_BAUDRATE            9600
#define GPS_RX_PIN              15
#define GPS_TX_PIN              13

// LoRa SPI
#define SCK_GPIO                5
#define MISO_GPIO               19
#define MOSI_GPIO               27
#define NSS_GPIO                18
#define RESET_GPIO              LMIC_UNUSED_PIN
#define DIO0_GPIO               26
#define DIO1_GPIO               33              // Note: YOU SHOULD SOLDER A JUMP BETWEEN MCU PIN AND LORA PIN 
#define DIO2_GPIO               32              // Note: not really used on this board

// Power Management
#define BAT_NUM_READ            5               // Moving average size for battery readings
#define BAT_MAX_V               4.2             // Max Voltage of a 1s1p lipo cell
#define BAT_MIN_V               2.4             // Min voltage to prevent battery damage
#define ENABLE_DEEP_SLEEP       true            // Enable deep sleep for power savings

#endif // CONFIGURATION_H
