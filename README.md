# AD9912_nucleo

# Commands

Note, that each parameter has max and min value. For instance: `F.MAX`, `F.MIN`. These values can be modified: `F.MAX 130`.

## DDS frequency setting

`F 123.14`  - set frequency 123.14 MHz

`F ?`   - get current F

`RF ?`  - get real frequency set in dds

## DDS amplitude setting

`CUR 30.1`

Max value is 31.7.

## DDS dedrift setting

`DED:ON`: dedrift ON/OFF (1-ON, 0-OFF)

`DED:HZPS`: dedrift speed in Hz/s