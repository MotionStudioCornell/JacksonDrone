# P2 Flight Controller

Custom Pico-based flight controller/computer software.

The KiCad Schematics can be found here: https://github.com/MotionStudioCornell/P2-Schematics

## Dev Info

### MPU9250
Data Sheet: https://invensense.tdk.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf

Register Map: https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-9250-Register-Map.pdf

### Pi Pico(W)
Documents: https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html


## Build

Requirement:

Raspberry Pi Pico C/C++ SDK

`cd build`

`cmake ../src -DPICO_BOARD=pico_w`

`make`

## Flash

Find `P2.uf2` in the `build` folder, and drag it onto the Pico MSD(Or any methods you prefer)
