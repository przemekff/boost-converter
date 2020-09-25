# boost-converter
boost converter designed for ATmega microcontroller

Compile with the options to enable floating point
numbers in printf():
    -Wl,-u,vfprintf -lprintf_flt -lm
    
Pin assignment:
| Port | Pin | Use                         |
|------+-----+-----------------------------|
| A    | PA0 | Voltage at load             |
| D    | PD0 | Host connection TX (orange) |
| D    | PD1 | Host connection RX (yellow) |
| D    | PD7 | PWM out to drive MOSFET     |
| B    | PB0 | Button to control V_target  |
| B    | PB1 | Button to control V_target  |
| B    | PB2 | Output to diode 1  		     |
| B    | PB3 | Output to diode 2  		     |
