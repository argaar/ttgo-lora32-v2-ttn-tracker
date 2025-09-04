## TTGO LoRa32 v2.0 Enhanced GPS Tracker for The Things Network

Current version: 2.0.0

Advanced GPS tracker for [TTGO LoRa32 v2.0](https://github.com/LilyGO/TTGO-LORA32/) that uploads location data to [The Things Network](https://www.thethingsnetwork.org) (TTN) and (optionally) [TTN Mapper](https://ttnmapper.org).
Features power-efficient operation with movement-based transmission and deep sleep capabilities.

#### Based on the code from [kizniche/ttgo-tbeam-ttn-tracker](https://github.com/kizniche/ttgo-tbeam-ttn-tracker/tree/master/main). License is the same.
Enhanced with power management, movement detection, improved GPS handling, battery voltage readings, and LoRaWAN timing optimizations. Partially ported the code to work with TTGO LoRa32 v2.0 board instead of TBeam.

This is a LoRaWAN node based on the [TTGO LoRa32 v2.0](https://github.com/LilyGO/TTGO-LORA32) development platform using the SSD1306 I2C OLED display.
It uses a Semtech SX1276 and the MCCI LoRaWAN LMIC stack.
This code is configured by default to connect to The Things Network using the EU 868 MHz frequency by default, but can be changed to US 915 MHz.

Two additional boards has been used for this project:
- NEO-6M GPS with active antenna
  In the project is present a file with some commands that will be passed to the GPS Unit in order to suppress some NMEA sentences and change the update rate to 5Hz

- [TP4056 board](https://www.robotstore.it/Caricabatterie-Li-Ion-TP4056-con-circuito-di-protezione) to manage the battery (if voltage drops below 2.4v it will shutdown the output)
  Even if the Lora32 board already have a charger ic, using this tiny board will provide USB-C instead of Micro-USB and some other features
  like: discarge protection, overvoltage protection and inverse polarity protection.

NOTE: At the moment of writing there are at least 4 versions of the TTGO LoRa32, this code is focused on version 2.0, but can easily been adapted to other boards changing Pins.
Please refer to the [attached image](ttgo_lora32_v2.0_pinout.png) in order to know pin mapping.

### Setup

The preferred method to install this library is via [PlatformIO](https://platformio.org/install), however the original instructions for installing with the Arduino IDE are below but YMMV.

**platformIO** users can ignore step 1 - 3, and have to set their region and radio type in ```platformio.ini```.

1. Follow the directions at [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) to install the board to the Arduino IDE and use board 'T-Beam'.

2. Install the Arduino IDE libraries:

   * [mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic) release v4.1.1
   * [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) release v1.0.3a
   * [ThingPulse/esp8266-oled-ssd1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) release 4.6.1
   * [plerup/espsoftwareserial](https://github.com/plerup/espsoftwareserial) release 8.2.0
   * [rlogiacco/CircularBuffer](https://github.com/rlogiacco/CircularBuffer) release 1.4.0
   * [thomasfredericks/Bounce2](https://github.com/thomasfredericks/Bounce2) release v2.71

1. Edit ```arduino-lmic/project_config/lmic_project_config.h``` and uncomment the proper frequency for your region.

2. Edit this project file ```main/configuration.h``` and and confirm your setup.

3. Copy this project file ```main/credentials_default.h``` as ```main/credentials.h``` and change the Keys/EUIs for your Application's Device from The Things Network.

4. Add the TTN Mapper integration to your Application (and optionally the Data Storage integration if you want to access the GPS location information yourself), then add the Decoder code:

```C
function Decoder(bytes, port) {
    var decoded = {};

    decoded.latitude = ((bytes[0]<<16)>>>0) + ((bytes[1]<<8)>>>0) + bytes[2];
    decoded.latitude = (decoded.latitude / 16777215.0 * 180) - 90;

    decoded.longitude = ((bytes[3]<<16)>>>0) + ((bytes[4]<<8)>>>0) + bytes[5];
    decoded.longitude = (decoded.longitude / 16777215.0 * 360) - 180;

    var altValue = ((bytes[6]<<8)>>>0) + bytes[7];
    var sign = bytes[6] & (1 << 7);
    if(sign) decoded.altitude = 0xFFFF0000 | altValue;
    else decoded.altitude = altValue;

    decoded.hdop = bytes[8] / 10.0;
    decoded.sats = bytes[9];

    return decoded;
}
```

7. Open this project file ```main/main.ino``` with the Arduino IDE and upload it to your TTGO LoRa32 v2.0.

8. In order to get your board working, you need to solder a cable between pin GPIO33 to Lora DI01 to let Lora lib know when it need to open the RX window
Also, if you want proper battery voltage detection you need to solder two 100KOmh resistors to form a Voltage Divider between the VCC pin coming from the PH2 connector ("+"), GND ("-") and the pin 35 (or another that you choose in [configuration.h](configuration.h))
ie: VCC (PH2) ----- 100KOmh ----- PIN35 ----- 100KOmh ----- GND

1. Turn on the device and once a GPS lock is acquired, the device will start sending data to TTN and TTN Mapper.

## v2.0.0 Improvements

### Enhanced Power Management
- **Deep Sleep Mode**: Automatically enters deep sleep when stationary to extend battery life
- **Movement-Based Transmission**: Only transmits when device moves beyond 5-meters threshold
- **Adaptive Intervals**: Uses longer intervals (30s) when stationary, normal (60s) when moving
- **LoRaWAN Timing Compliance**: Optimized for strict LoRaWAN duty cycle requirements
- **Battery Status Report**: Battery voltage (moving average) and charge percentage are shown on the display

### GPS Improvements
- **Timeout Handling**: 2-minute GPS fix timeout with automatic recovery
- **Movement Detection**: Haversine formula for accurate distance calculation
- **Position Validation**: HDOP and coordinate range validation
- **Reference Position Tracking**: Maintains last known position for movement detection

### System Reliability
- **Watchdog Timer**: 30-second hardware watchdog for system stability
- **Proper Header Guards**: All header files protected against multiple inclusion
- **Consistent Naming**: Standardized snake_case variable naming
- **Memory Management**: Initialization checks and error handling

### LoRaWAN Optimizations
- **ADR Enabled**: Adaptive Data Rate for network efficiency
- **Smart Transmission Logic**: Transmits only on button press OR (60s interval + movement)
- **Event-Driven Messaging**: Proper "Message sent" acknowledgment via TXCOMPLETE event
- **Session Persistence**: Maintains network session across deep sleep cycles

### Operation Modes
- **Active Mode**: Normal operation with 60-second transmission intervals when moving
- **Stationary Mode**: Extended 30-second deep sleep intervals when no movement detected
- **Manual Mode**: Instant transmission via send button press (bypasses all intervals)
- **Reset Mode**: Long-press discard button (9s) to clear network preferences and rejoin

### OLED Display
- **Layout**: Proper message, icons and layout has been implemented to show info [see](tracker.jpg)

### Power Consumption
- **Active Tracking**: ~80mA average (ESP32 + GPS + LoRa)
- **Deep Sleep**: ~15μA (ESP32 deep sleep, GPS remains powered)
- **Transmission**: ~120mA peak during LoRa TX (brief duration)