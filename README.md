## TTGO LoRa32 v2.0 Enhanced GPS Tracker for The Things Network

Current version: 2.1.1

Advanced GPS tracker for [TTGO LoRa32 v2.0](https://github.com/LilyGO/TTGO-LORA32/) that uploads location data to [The Things Network](https://www.thethingsnetwork.org) (TTN) and (optionally) [TTN Mapper](https://ttnmapper.org).
Features power-efficient operation with movement-based transmission and deep sleep capabilities.

#### Based on the code from [kizniche/ttgo-tbeam-ttn-tracker](https://github.com/kizniche/ttgo-tbeam-ttn-tracker/tree/master/main). License is the same.
Enhanced with power management, movement detection, improved GPS handling, battery voltage readings, and LoRaWAN timing optimizations. Partially ported the code to work with TTGO LoRa32 v2.0 board instead of TBeam.

This is a LoRaWAN node based on the [TTGO LoRa32 v2.0](https://github.com/LilyGO/TTGO-LORA32) development platform using the SSD1306 I2C OLED display.
It uses a Semtech SX1276 and the MCCI LoRaWAN LMIC stack.
This code is configured by default to connect to The Things Network using the EU 868 MHz frequency, but can be changed to US 915 MHz.

Two additional boards have been used for this project:
- NEO-6M GPS with active antenna
  The project includes a file with commands that configure the GPS unit to suppress some NMEA sentences and change the update rate to 5 Hz

- [TP4056 board](https://www.robotstore.it/Caricabatterie-Li-Ion-TP4056-con-circuito-di-protezione) to manage the battery (if voltage drops below 2.4 V it will shut down the output)
  Although the LoRa32 board already has a charger IC, using this small board provides USB-C instead of Micro-USB, as well as additional features such as discharge protection, overvoltage protection, and reverse polarity protection.

NOTE: At the time of writing, there are at least four versions of the TTGO LoRa32, this code is focused on version 2.0, but can easily be adapted to other boards changing pins.
Please refer to the [attached image](ttgo_lora32_v2.0_pinout.png) for pin mapping.

### Setup

The preferred method to install this library is via [PlatformIO](https://platformio.org/install), however the original instructions for installing with the Arduino IDE are below though your results may vary.

**PlatformIO** users can ignore step 1 - 3, and have to set their region and radio type in ```platformio.ini```.

1. Follow the directions at [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) to install the board to the Arduino IDE and use board 'T-Beam'.

2. Install the Arduino IDE libraries:

   * [mcci-catena/arduino-lmic](https://github.com/mcci-catena/arduino-lmic) release v4.1.1
   * [mikalhart/TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) release v1.0.3a
   * [ThingPulse/esp8266-oled-ssd1306](https://github.com/ThingPulse/esp8266-oled-ssd1306) release 4.6.1
   * [plerup/espsoftwareserial](https://github.com/plerup/espsoftwareserial) release 8.2.0
   * [rlogiacco/CircularBuffer](https://github.com/rlogiacco/CircularBuffer) release 1.4.0
   * [thomasfredericks/Bounce2](https://github.com/thomasfredericks/Bounce2) release v2.71

1. Edit ```arduino-lmic/project_config/lmic_project_config.h``` and uncomment the proper frequency for your region.

2. Edit this project file ```main/configuration.h``` and confirm your setup.

3. Copy this project file ```main/credentials_default.h``` as ```main/credentials.h``` and change the Keys/EUIs for your Application's Device from The Things Network.

4. Add the TTN Mapper integration to your Application (and optionally the Data Storage integration if you want to access the GPS location information yourself), then add the Decoder code:

```C
function decodeUplink(input) {

    var data = {};
    var events = {
      1: "position",
      2: "alive",
    }; 

    data.recvTime = input.recvTime.toString();
    data.event = events[input.fPort];

    if (input.fPort === 1) {

      data.latitude = ((input.bytes[0]<<16)>>>0) + ((input.bytes[1]<<8)>>>0) + input.bytes[2];
      data.latitude = (data.latitude / 16777215.0 * 180) - 90;

      data.longitude = ((input.bytes[3]<<16)>>>0) + ((input.bytes[4]<<8)>>>0) + input.bytes[5];
      data.longitude = (data.longitude / 16777215.0 * 360) - 180;

      var altValue = ((input.bytes[6]<<8)>>>0) + input.bytes[7];
      var sign = input.bytes[6] & (1 << 7);
      if(sign) data.altitude = 0xFFFF0000 | altValue;
      else data.altitude = altValue;

      data.hdop = input.bytes[8] / 10.0;
      data.sats = input.bytes[9];

    } else if (input.fPort === 2) {

      alive = input.bytes[0];
      if (alive === 1) {
        data.message = "Device is alive";
      } else {
        data.message = "Wrong data";
      }

    }

    return {
      data: data,
    };
}
```

7. Open this project file ```main/main.ino``` with the Arduino IDE and upload it to your TTGO LoRa32 v2.0.

8. To get your board working, solder a wire between GPIO33 and LoRa DIO1 so the LoRa library knows when it needs to open the RX window.
Also, for accurate battery voltage detection, solder two 100 kΩ resistors to form a voltage divider between the VCC pin from the PH2 connector ("+"), GND ("-"), and pin 35 (or another pin specified in [configuration.h](configuration.h))
Example: VCC (PH2) ----- 100 kΩ ----- PIN35 ----- 100 kΩ ----- GND

9. Turn on the device and once a GPS lock is acquired, the device will start sending data to TTN and TTN Mapper.

## Changelog

See what changed between versions [here](CHANGELOG.md)
