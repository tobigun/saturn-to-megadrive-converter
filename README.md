# Saturn to Mega Drive Gamepad converter
Uses a Waveshare RP2040 Zero (similar to the Raspberry Pi Pico) to convert a Sega Saturn gamepad into a Mega Drive or USB gamepad.

Attach a DB9 female connector to the RP2040 Zero that can be plugged into the Mega Drive.

Connect a Saturn Controller directly to the RP2040 Zero via cutting the Controller's cable or a controller extension cable.

It is also possible to connect the RP2040 Zero via USB to a computer to use the Saturn gamepad as a USB HID gamepad.
If you are only interested in a Saturn to USB converter, you do not need to attach a Mega Drive connector to the RP2040 Zero.

## Pinout (Waveshare RP2040 Zero)
|Pin|Connector|Signal|Direction|
|---|---------|------|---------|
|7|Mega Drive|SELECT|IN|
|8|Mega Drive|B_A|OUT|
|9|Mega Drive|UP_Z|OUT|
|10|Mega Drive|DOWN_Y|OUT|
|11|Mega Drive|LEFT_X|OUT|
|12|Mega Drive|RIGHT_MODE|OUT|
|13|Mega Drive|C_START|OUT|
|14|Saturn|S0|OUT|
|15|Saturn|S1|OUT|
|26|Saturn|D0|IN|
|27|Saturn|D1|IN|
|28|Saturn|D2|IN|
|29|Saturn|D3|IN|
