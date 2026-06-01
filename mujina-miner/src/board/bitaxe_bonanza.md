# BitaxeBonanza Board Support

This document describes mujina-miner's support for the BitaxeBonanza board.

## Overview

The BitaxeBonanza is an open-source Bitcoin mining board with four Intel BZM2
ASICs and an ESP32-S3 running bitaxe-raw firmware. The firmware exposes two USB
CDC ACM serial ports.

## USB Identity

- VID: `0xc0de`
- PID: `0xcafe`
- Manufacturer: `OSMU`
- Product: `BitaxeBonanza`

## Serial Ports

The board presents two serial ports when connected:

- Interface 0: control channel for GPIO, I2C, ADC, and fan commands
- Interface 2: data channel for BZM2 ASIC UART traffic

On Linux, the stable by-id paths use `if00` for control and `if02` for data.
The control channel uses `115200` baud. The data channel must be opened at
`5000000` baud so the USB CDC line coding configures the ESP32 ASIC UART link.

## GPIO Map

BitaxeBonanza uses bitaxe-raw GPIO page `0x06` with these command values:

- `0x00` - ASIC reset compatibility alias (`RST_N`, active-low reset)
- `0x01` - `5V_EN` power enable
- `0x02` - `ASIC_RST` active-low reset
- `0x03` - `ASIC_TRIP` status
- `0x04` - `VR_EN` regulator enable
- `0x05` - `VR_PGOOD` regulator power-good status

## ASIC Transport

BZM2 TX frames are encoded as 9-bit byte pairs on the USB data channel. ASIC RX
responses are plain 8-bit bytes; the firmware strips the 9th bit on receive.
