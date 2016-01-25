# PS/2 Interface
#### Implementation of the PS/2 interface protocol

This is an implementation of the host side and the device of the PS/2 protocol for the AtMega8.
It does NOT use any Arduino library.

This projects was made stand in between a Mouse and a PC to annoy someone. It randomly switches the left and right clicks or the movement direction (up/down, left/right/ and scroll up/scroll down).

The PS/2 implementation itself works great, but it would be a great idea to build some kind of output buffer, so that the main loop would not need to lock every time it wanted to send something

#### Connections

PD0 - Data Host line
PD1 - Data Device line
PD2 - Clock Device line
PD3 - Clock Host line

#### References

http://www.computer-engineering.org/ps2protocol/

http://www.computer-engineering.org/ps2mouse/

http://extremeelectronics.co.in/avr-tutorials/ps2-keyboard-interface-with-avr-mcu/

http://wiki.osdev.org/Mouse_Input
