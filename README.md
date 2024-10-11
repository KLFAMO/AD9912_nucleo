# AD9912_nucleo
## Installing
### Bottom board (DDS AD9912 EVAL)
* connect 100 MHz (around 3 dBm) reference signal to `SYSCLK`
* connect output to `DUT FILTER OUT` 
![image](https://github.com/KLFAMO/AD9912_nucleo/assets/114686928/b806660c-8974-44a4-b481-348a31766cbc)

### Middle board
* connect GND and +5V to the green socket. See decription on the board. If there are two sockets, use one of them.
Power supply form this board is distributed to uC and DDS boards. Up to 1 A is usually required (typically around 0.6 A)

### Top board (Nucleo)
* connect ethernet cable to connect to your local network
Device may be configurated with DHCP or static IP. See description on the device. Port 22.

## Communication
* In case of DHCP mode, chceck assigned IP addres on your DHCP server (at the moment it is not possible to read IP address from the device)
* Use Telnet client to connect to the device (port 10).
* The device is by default set to work with 100 MHz reference signal.


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