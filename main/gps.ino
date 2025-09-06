/*

  GPS module

  Copyright (C) 2025 Davide Foschi, based on code by Xose Pérez <xose dot perez at gmail dot com>

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

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Preferences.h>
#include <gps_commands.h>

// GPS data variables
uint32_t LatitudeBinary;
uint32_t LongitudeBinary;
uint16_t altitudeGps;
uint8_t hdopGps;
uint8_t sats;
char t[32]; // used to sprintf for Serial output

// GPS objects
TinyGPSPlus _gps;
EspSoftwareSerial::UART _serial_gps;

// GPS timeout tracking
static uint32_t gps_start_time = 0;
static uint32_t last_fix_time = 0;

// Movement detection
static double last_lat = 0.0;
static double last_lon = 0.0;
static uint32_t last_movement_check = 0;

void gps_time(char * buffer, uint8_t size) {
    snprintf(buffer, size, "%02d:%02d:%02d", _gps.time.hour(), _gps.time.minute(), _gps.time.second());
}

float gps_latitude() {
    return _gps.location.lat();
}

float gps_longitude() {
    return _gps.location.lng();
}

float gps_altitude() {
    return _gps.altitude.meters();
}

float gps_hdop() {
    return _gps.hdop.hdop();
}

uint8_t gps_sats() {
    return _gps.satellites.value();
}

void gps_send_UBX(const uint8_t *progmemBytes, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    _serial_gps.write((uint8_t)pgm_read_byte_near(progmemBytes + i));
  }
  _serial_gps.flush();
  delay(100);
}

bool check_for_gps_data() {
    bool baudOK = false;
    unsigned long start = millis();
    while (millis() - start < 2000) { // Collect a max of 2s of data
        if (_serial_gps.available()) {
            char c = _serial_gps.read();
            if (c == '$') {   // NMEA always starts with $
                baudOK = true;
                break;
            }
        }
    }
    return baudOK;
}

bool gps_setup() {

    // Start GPS serial in desired baudrate and test if it works, otherwise configure GPS device to switch to new params
    _serial_gps.begin(GPS_BAUDRATE, EspSoftwareSerial::SWSERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

    bool gps_connection = check_for_gps_data();
    if (gps_connection) {

        // Step 2: Set 5 Hz update rate
        gps_send_UBX(setRate5Hz, sizeof(setRate5Hz));

        // Step 3: Enable only RMC+GGA
        gps_send_UBX(enableRMC, sizeof(enableRMC));
        gps_send_UBX(enableGGA, sizeof(enableGGA));
        
        // Step 4: Disable other NMEA sentences
        gps_send_UBX(disableGLL, sizeof(disableGLL));
        gps_send_UBX(disableGSA, sizeof(disableGSA));
        gps_send_UBX(disableGSV, sizeof(disableGSV));
        gps_send_UBX(disableVTG, sizeof(disableVTG));

        screen_print("gps", "GPS device ok");
        gps_start_time = millis();

        return true;

    } else {
        screen_print("gps", "Can't initialize GPS");
        return false;
    }
}

bool gps_wait_for_fix(uint32_t timeout_ms) {
    uint32_t start_time = millis();
    while (millis() - start_time < timeout_ms) {
        gps_loop();
        if (gps_fixed) {
            return true;
        }
        esp_task_wdt_reset();
        yield();
        delay(50);
    }
    return false;
}

uint32_t gps_get_time_since_fix() {
    if (last_fix_time == 0) return UINT32_MAX;
    return millis() - last_fix_time;
}

float gps_calculate_distance(double lat1, double lon1, double lat2, double lon2) {
    // Haversine formula for distance calculation
    const float R = 6371000; // Earth's radius in meters
    
    float dlat = radians(lat2 - lat1);
    float dlon = radians(lon2 - lon1);
    
    float a = sin(dlat/2) * sin(dlat/2) + 
              cos(radians(lat1)) * cos(radians(lat2)) * 
              sin(dlon/2) * sin(dlon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    
    return R * c;
}

bool gps_has_moved() {
    if (!gps_fixed || last_lat == 0.0) {
        return false; // No previous position or no current fix
    }
    
    float distance = gps_calculate_distance(last_lat, last_lon, 
                                          gps_latitude(), gps_longitude());
    return distance > MOVEMENT_THRESHOLD_M;
}

void gps_update_last_position() {
    if (gps_fixed) {
        last_lat = gps_latitude();
        last_lon = gps_longitude();
        last_movement_check = millis();
    }
}

bool gps_is_stationary() {
    return !gps_has_moved() && 
           (millis() - last_movement_check) > STATIONARY_CHECK_MS;
}

static void gps_loop() {

    while (_serial_gps.available()) {
        _gps.encode(_serial_gps.read());
    }

    if (_gps.location.isValid() && _gps.hdop.hdop() < MAX_VALID_HDOP) {
        if (!gps_fixed) {
            last_fix_time = millis();
        }
        gps_fixed = true;
    } else if (gps_get_time_since_fix() > GPS_TIMEOUT) {
        gps_fixed = false;
    }
}

void buildGPSPacket(uint8_t gpsBuffer[10]) {
    LatitudeBinary = ((_gps.location.lat() + 90) / 180.0) * 16777215;
    LongitudeBinary = ((_gps.location.lng() + 180) / 360.0) * 16777215;
    altitudeGps = _gps.altitude.meters();
    hdopGps = _gps.hdop.value() / 10;
    sats = _gps.satellites.value();

    gpsBuffer[0] = ( LatitudeBinary >> 16 ) & 0xFF;
    gpsBuffer[1] = ( LatitudeBinary >> 8 ) & 0xFF;
    gpsBuffer[2] = LatitudeBinary & 0xFF;
    gpsBuffer[3] = ( LongitudeBinary >> 16 ) & 0xFF;
    gpsBuffer[4] = ( LongitudeBinary >> 8 ) & 0xFF;
    gpsBuffer[5] = LongitudeBinary & 0xFF;
    gpsBuffer[6] = ( altitudeGps >> 8 ) & 0xFF;
    gpsBuffer[7] = altitudeGps & 0xFF;
    gpsBuffer[8] = hdopGps & 0xFF;
    gpsBuffer[9] = sats & 0xFF;
}

void buildAlivePacket(uint8_t aliveBuffer[1]) {
    aliveBuffer[0] = 1 & 0xFF;
}