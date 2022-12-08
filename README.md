# PCB Heater


This project is an attempt to fix a small incovenience in my life.

# A bit of hystory

Whenever it's winter and I spend more than a couple hours straight sitting at my desk (you did it too, admit it) my feet get terribly cold. So cold that when I finally decide to get up they hurt during the first steps.

To fix this I decided to create a small assembly that heats up to a set temperature, on which i can comfortably rest my feet while I'm at my desk.

The assembly consists of three main parts, the heating element, a driver circuit and the wood structure on which everything mounts on.

# Hardware design

The heating element is 30x30 cm^2 pcb that features a long copper track which acts as a resistor, and thus heats up when current flows through it.

The pcb is made of three 10x30 cm^2 pcb connected in series. The reason for this is that the cost of making five 10x30 cm^2 pcbs is way less than making 5 30x30 cm^2 pcbs. Each single pcb has a resistance of X Ohm and has a track x cm long.

The heating element is powered via a MOSFET used as switch with PWM by an ESP32-S2-MINI-2 MCU. The driver board also has an OLED screen and some user buttons. The main power source is a 12V-80W AC/DC brick, which feeds the heating PCB and the MCU part of the circuit via a 3.3V switching regulator.

Check bom file for full parts list.

The driver PCB is enclosed in a 3D printed case, which is screwed on top of the wood assembly.


# Software design



# End result