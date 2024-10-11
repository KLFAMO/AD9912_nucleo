# AD9912_nucleo

# Commands

User interface is available on port `10`.

Note, that each parameter has max and min value. For instance: `F.MAX`, `F.MIN`. These values can be modified: `F.MAX 130`.

## DDS frequency setting

`F 123.14`  - set frequency 123.14 MHz

`F ?`   - get current F

`RF ?`  - get real frequency set in dds

The microcontroller operates in cyclic mode (with a period of 1ms). After receiving a new parameter value via Ethernet, it will be updated to the DDS in the next microcontroller cycle (within 1ms).

## DDS amplitude setting

`CUR 30.1`

Max value is 31.7.

## DDS dedrift setting

`DED:ON`: dedrift ON/OFF (1-ON, 0-OFF)

`DED:HZPS`: dedrift speed in Hz/s

Dedrift is applied every 1ms. Reference clock is internal oscillator of microcontroller (there will be some callibration mechanism added in the future).