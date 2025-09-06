## v2.1.1

- **Fix GPS Print**: Moved function to print GPS position so you can read all location changes as soon as they're detected

## v2.1.0

- **No confirmation**: Removed confirmation of messages every N. If we don’t receive an ACK from TTN, the packet remains in the queue. Since this is a tracker, we’re supposed to be moving, so keeping and resending an old position after we’ve already moved is not useful.
- **Added alive message**: Introduced a new message on a separate LoRa port to send an "alive" event when no GPS fix is available. This confirms that the device can still communicate with the network, even if its position is unknown.
- **Fix GPS Commands**: Only RMC and GGA sentences are now enabled and parsed from the GPS device.
- **Payload Formatter**: Adapted to new TTN Uplink format

## v2.0.0

### Enhanced Power Management
- **Deep Sleep Mode**: Automatically enters deep sleep when stationary to extend battery life
- **Movement-Based Transmission**: Only transmits when the device moves beyond 5-meters threshold
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
- **Active Mode**: Transmits every 60 seconds when moving
- **Stationary Mode**: Enters deep sleep with 30-second intervals when stationary
- **Manual Mode**: Instant transmission via send button press (bypasses all intervals)
- **Reset Mode**: Long-press discard button (9s) to clear network preferences and rejoin

### OLED Display
- **Layout**: Proper messages, icons, and layout have been implemented to show info [see](tracker.jpg) for reference

### Power Consumption
- **Active Tracking**: ~80 mA average (ESP32 + GPS + LoRa)
- **Deep Sleep**: ~15 μA (ESP32 deep sleep, GPS remains powered)
- **Transmission**: ~120 mA peak during LoRa TX (brief duration)
