/*

SSD1306 - Screen module

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

#include <Wire.h>
#include "SSD1306Wire.h"
#include "OLEDDisplay.h"
#include "images.h"
#include "fonts.h"

SSD1306Wire * display;

char gps_message[24], hw_message[24], ttn_message[48];

void _screen_header() {
    if(!display) return;

    char buffer[20];

    // Message count
    snprintf(buffer, sizeof(buffer), "#%05d", ttn_get_count() % 100000);
    display->setTextAlignment(TEXT_ALIGN_LEFT);
    display->drawString(0, 2, buffer);

    // GPS Time
    gps_time(buffer, sizeof(buffer));
    display->setTextAlignment(TEXT_ALIGN_CENTER);
    display->drawString(display->getWidth()/2, 2, buffer);

    // Satellite count
    display->setTextAlignment(TEXT_ALIGN_RIGHT);
    display->drawString(display->getWidth() - SATELLITE_IMAGE_WIDTH - 4, 2, itoa(gps_sats(), buffer, 10));
    display->drawXbm(display->getWidth() - SATELLITE_IMAGE_WIDTH, 0, SATELLITE_IMAGE_WIDTH, SATELLITE_IMAGE_HEIGHT, SATELLITE_IMAGE);
}

void screen_setup() {
    // Display instance
    display = new SSD1306Wire(SSD1306_ADDRESS, I2C_SDA, I2C_SCL);
    
    display->init();
    display->flipScreenVertically();
}

void screen_print(const char type[3], const char message[50]) {
    if (strcmp(type, "hw") == 0) {
        snprintf(hw_message, sizeof(hw_message), "%s", message);
    } else if (strcmp(type, "gps") == 0) {
        snprintf(gps_message, sizeof(gps_message), "%s", message);
    } else if (strcmp(type, "ttn") == 0) {
        snprintf(ttn_message, sizeof(ttn_message), "%s", message);
    }
}

void screen_loop() {
    if (!display) return;

    display->clear();

    display->setFont(Custom_ArialMT_Plain_10);
    _screen_header();

    display->drawHorizontalLine(0, 16, display->getWidth());

    display->setTextAlignment(TEXT_ALIGN_LEFT);

    if (board_ready==true) {
        display->drawXbm(0, 18, BOARD_IMAGE_WIDTH, BOARD_IMAGE_HEIGHT, BOARD_IMAGE);
    } else {
        display->drawXbm(0, 18, NOBOARD_IMAGE_WIDTH, NOBOARD_IMAGE_HEIGHT, NOBOARD_IMAGE);
    }
    display->drawString(BOARD_IMAGE_WIDTH+1, 17, hw_message);

    if (gps_fixed==true) {
        display->drawXbm(0, 29, GPS_IMAGE_WIDTH, GPS_IMAGE_HEIGHT, GPS_IMAGE);
    } else {
        display->drawXbm(0, 29, NOGPS_IMAGE_WIDTH, NOGPS_IMAGE_HEIGHT, NOGPS_IMAGE);
    }
    display->drawString(GPS_IMAGE_WIDTH+1, 28, gps_message);

    if (ttn_joined==true) {
        display->drawXbm(0, 40, TTN_IMAGE_WIDTH, TTN_IMAGE_HEIGHT, TTN_IMAGE);
    } else {
        display->drawXbm(0, 40, NOTTN_IMAGE_WIDTH, NOTTN_IMAGE_HEIGHT, NOTTN_IMAGE);
    }
    display->drawStringMaxWidth(TTN_IMAGE_WIDTH+1, 39, (display->getWidth()-TTN_IMAGE_WIDTH-1), ttn_message);

    display->display();
}