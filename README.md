ESP32 FlexiSpot Desk Controller & Display
<img width="3000" height="4000" alt="20260526_131311" src="https://github.com/user-attachments/assets/bdc43e7d-c3cd-4f41-8d4a-c54d5f0c9ae2" />


This project uses an ESP32 to replace a dead FlexiSpot controller — Added a display to track my sitting vs standing and stats.
What it does:

Controls the desk height remotely via web interface
Moves the desk on a timer/schedule
Reports sitting vs. standing state & Steps to a display
Steps is handled from my amazfit 7 watch -> notify for andriod -> mqtt to esp32 -> display
Saves everything via Eeprom
Wifi is configurable

How it works:
The ESP32 is wired in bypass mode, taking over the desk's control bus directly. Originally, standing vs. sitting detection was handled by an ultrasonic distance sensor — this is still supported and is the simpler option. The current implementation uses the bypass wiring to determine desk position directly.
Other approaches (not used here):

Some FlexiSpot models have a second RJ45 port on the controller — there are community projects that tap into that port instead of bypassing the controller entirely. If your desk has that port, that may be an easier route.

Code:
All code was written by Claude AI. If you want to switch back to the ultrasonic sensor method or use the second-port approach instead, just ask Claude to swap it out.

Parts
HiLetgo 3pcs AM312 Mini Pyroelectric PIR Human Sensor Module PIR Infrared IR Sensor Body Manual Motion Infrared IR Detector
HC-SR04P Ultrasonic Distance Sensor Module with 3V to 5.5V Wide Voltage, 2cm–450cm Range, 4-Pin Interface Compatible with Arduino and Raspberry Pi

Displays
1/3PCS 5.5 Inch OLED LCD display green color 256x64 Drive SSD1322 interface SPI/ 8-bit Parallel Port For Arduino UNO R3
https://www.aliexpress.us/item/3256808557280461.html?spm=a2g0o.order_detail.order_detail_item.2.5fb6f19c4ly8KW&gatewayAdapt=glo2usa

or

2pcs 3.12 inch OLED Display 256x64 OLED LCD Display SSD1322 Module 16pin Parallel SPI Soldering for Arduino (Yellow)
https://www.amazon.com/dp/B0F7L9PLRM?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_3&th=1
Pin 1 - GND, Pin2 VCC, Pin 4, CLK, Pin 5 DIN, Pin 14 DC, Pin

#define PIN_CS 5       // /CS purple to blue/w
#define PIN_DC 27      // D/C orange to blue
#define PIN_RST 33     // /RES green to green/w
#define PIN_CLK 18     // SCLK grey to green
#define PIN_MOSI 23    // SDIN brown to bwron/w 
