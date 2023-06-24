
# TFG.  Digital signals remote sensor.

This code is part of a Bachelor's Final Project and it is based on root's project. 

To build the application, follow the steps outlined in the tutorial: [Using the Low-Level Sub-GHz Radio Driver for the STM32WL Series](https://forum.digikey.com/t/using-the-low-level-sub-ghz-radio-driver-for-the-stm32wl-series/18253). Then, simply replace the `Inc/` and `Src/` folders in the project with the `Inc/` and `Src/` folders in this repository.

There are two main.c files; one of them is for the client side and the other is for the server side, also, the PCB proyect files.

The clien side `Client/Core/Src/main.c`, only operates as master states and above all sending data, except that it receives ACK.

The server side `Server/Core/Src/main.c`, exclusively works as slave states and mainly receiving data, except that it sends ACK.

The desing of the PCB in `PCB/SensorR.zip`
