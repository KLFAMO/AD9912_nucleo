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
* Use Telnet client to connect to the device (port 22). After connecting, welcome text with short manual in the console should appear.
* The device is by default set to work with 100 MHz reference signal.
* Type `DDS:FREQ 98.23` to set output frequency to 98.23 MHz
* The amplitude of output signal is by default set to maximum (31mA). To change amplitude use `DDS:AMPL <val>`, where <val> is from 0 to 31. However val=0 does not mean 0 amplitude. It is not possible to switch off output using `DDS:AMPL`, use `DDS:FREQ 0` to switch off dds output.
