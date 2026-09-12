# ESP32-S3 High-Speed CAN FD Monitor (MCP2517FD)

This project turns the ESP32-S3 into a high-speed CAN FD receiver and message monitor using a Microchip **MCP2517FD** external CAN FD controller over SPI.

It is designed to handle rapid message streams (at least every 10ms and multi-message bursts) without packet loss, dynamically tracking and displaying message counts per unique CAN ID (both Standard 11-bit and Extended 29-bit IDs, CAN 2.0 and CAN FD).

---

## Hardware Connection

| ESP32-S3 Pin | MCP2517FD Pin | MCP2517FD IC Pin | Description |
| :--- | :--- | :--- | :--- |
| **GPIO 10** | **nCS** | 13 | SPI Chip Select |
| **GPIO 11** | **SDI / MOSI** | 11 | SPI Data In / Master Out |
| **GPIO 12** | **SCK** | 10 | SPI Clock |
| **GPIO 13** | **SDO / MISO** | 12 | SPI Data Out / Master In |
| **GPIO 14** | **INT** | 4 | Interrupt Line (Active Low) |
| *GPIO 15* | *INT0* | 9 | *(Optional)* |
| *GPIO 16* | *INT1* | 8 | *(Optional)* |
| **3.3V** | **VDD** | 14 | 3.3V Power Supply |
| **GND** | **VSS** | 7 | Ground |

---

## Key Features

1. **High-Speed Reception & Zero Packet Loss**:
   - Hardware SPI master clocked at 10 MHz (configurable up to 20 MHz).
   - MCP2517FD hardware FIFO 1 configured with 64-byte payload size and 24-message on-chip RAM depth.
   - GPIO 14 falling-edge interrupt triggers a high-priority FreeRTOS RX task (`priority 20`, pinned to CPU Core 1).
   - Batch-drains all queued frames in a tight loop per interrupt cycle.

2. **Per-CAN-ID Traffic Tracker**:
   - Fast hash table tracking up to 256 unique CAN IDs.
   - Records:
     - CAN ID (Hex)
     - Frame Type (Standard 11-bit / Extended 29-bit)
     - Format (Classic CAN 2.0 / CAN FD)
     - Bit Rate Switch (BRS) status
     - Payload length (DLC)
     - Total message count
     - Current frame rate (msgs/sec)
     - Payload hex preview

3. **Periodic Console Dashboard**:
   - A dedicated monitor task prints an ASCII summary table every second (`1000 ms`) to the serial console.

---

## Configuration

Run `idf.py menuconfig` -> **MCP2517FD CAN FD Configuration**:
- **SPI Pins**: GPIO 10 (CS), 11 (MOSI), 12 (SCK), 13 (MISO), 14 (INT)
- **Oscillator Frequency**: 40 MHz (default) or 20 MHz
- **Nominal Bitrate**: 500 kbps (default)
- **Data Bitrate**: 2000 kbps / 2 Mbps (default)
- **Monitor Display Interval**: 1000 ms

---

## Build, Flash & Monitor

```bash
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```
