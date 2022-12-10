
# PCB Heater


As part of my journey in embedded systems I try to learn new technologies with projects that fix inconveniences in my life. One such inconvenience is that during the winter time, especially in the evening, during long computer-session my feet get terribly cold. So cold that when I finally decide to get up they hurt during the first steps.

To fix this I decided to create a small assembly that houses a plate capable of heating up to a set temperature, on which I can comfortably rest my feet while I'm at my desk.

The assembly consists of three main parts, the heating element, a driver circuit and the wood structure on which everything mounts on.

# Hardware

The heating element is 30x30 cm^2 pcb that features a long copper track that acts as a resistor, and thus heats up when current flows through it.

The pcb is made of three 10x30 cm^2 aluminum pcb connected in series. The reason for using three smaller pcbs instead of a big one, given that the minimum order is 5 units, is that the cost of making five 10x30 cm^2 pcbs is way less than making 5 30x30 cm^2 pcbs. Each single pcb has a resistance of 2 Ohms and has a track 2.64 meters long.

The heating element is powered via a MOSFET used as switch with PWM by an ESP32-S2-MINI-2 MCU. The driver board has an OLED screen and some tactile buttons to let the user set the target temperature and monitor the whole assembly.

The main power source is a 12V-80W AC/DC brick, which feeds the heating PCB and the MCU part of the circuit via a 3.3V switching regulator.

Check bom file for full parts list.

The driver PCB is enclosed in a 3D printed case, which is screwed on the wood assembly.


# Firmware

The firmware for the ESP32 is written with the ESP-IDF framework and runs FreeRTOS.

