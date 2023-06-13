
# TFG.  Digital signals remote sensor.

This code is part of a Bachelor's Final Project and it is based on root's project. 

To build the application, follow the steps outlined in the tutorial: [Using the Low-Level Sub-GHz Radio Driver for the STM32WL Series](https://forum.digikey.com/t/using-the-low-level-sub-ghz-radio-driver-for-the-stm32wl-series/18253). Then, simply replace the `main.c` in the project with the `main.c` file in this repository.

There are two main.c files; one of them is for the client side and the other is for the server side.
The clien side mainC.c, only operates as master states and above all sending data.
The server side mainS.c, exclusively works as slave states and mainly receiving data.
