# SpinLED
Spinning LED display
- 8 LEDs array.
- RGB color, 3 * 8-bit hardware PWM control.
- MPU-6500 used to measure the rotation speed.
- Single ATmega328P core, optimized code, using interrupt. No Arduino.
- Read image/video from SDcard via SPI bus.
- No motor required. Attach this kit to any rotation device at reasonably RPM (fans, wheels...), the MCU will figure out the real-time rotation speed and do whatever to display the image reliably.

## Video/image data structure
A video contains a number of frames, each frame is an image. On PC (personal computer), image bitmap can be consider as a 2-D array.
```C
struct RGB bitmap[height][width];
```
This schema works quite well on PC, because PC monitor implement pixels in this structure. Image will be a rectangular in this case.

For the spinning LED display, image bitmap is constructed in different structure. The arm is rotating, and there are number of LEDs on the arm.The arm and LEDs look like a phasor, where the position of the arm is the phase, and the LEDs' distances from the center (fixed) are raduis.
```C
struct RGB bitmap[phase][radius];
```
Image will be a sphere in this case.

It is critical to use a data structure that is friendly for the MCU. In another word, the shorter time that MCU need tofetch the RBG data of any LED at any phase, the better the structure. If the MCU cannot be able to fetch the RGB data in the given time, the project fail. The following data sturcture is used:
```C
#define PHASE 32
#define LED 8
#define COLOR 4
uint8_t imageBuffer[PHASE][LED][COLOR];
```
Where:
- ```PHASE = 32```: There are 32 phases. A sphere image is devided into 32 __sections__, each section covers 360 / 32 = 11.25 degrees of area.
- ```LED = 8```: There are 8 LEDs on the arm. The gap between LEDs are same. A __section__ is divided into 8 __segments__.
- ```COLOR = 4```: Each LED has 3 colors, which are the 3 basic colors: read, green and blue. By changing the strength of each bisic color, the LED can provide light in any color. We use 4 instead of 3 because we want to make alignment, it is friendly to the hardware and software; we have tons of storage space on morden SD card.
- ```type = uint8_t```: The LED is controled by PWM signal. PWM signal is configured by generator register. Generator register is 8-bit wide (Timer0 is 8-bit. Timer1 is 16-bit, but only use 8-bit. Not worth to use 16-bit).

There are 32 * 8 = 256 __segments__ in total for each image; each __segment__ takes 4 bytes of space. That is, each image will take 1KB (1024 bytes) of space. That is 2 SD card data block. 1M storage space can hold 1024 images, or 42 seconds of video at 24 FPS.

TO DO... A picture may help to describe how this work.

## Pre-process video/image bitmap
As the above section state, a special data structure is required to render the image efficiently. Video/image need to be pre-processed on PC before upload it to SD card.

TO DO...

## MCU process
Basically, the MCU does 3 tasks:
- Measuring rotation speed. The arm of the spinning LED display is rotating, but the rendering image should not rotate. By measuring the rotation speed, the spinning LED display can be mounted on any rotation device (at reasonably rotation speed), and the rotation speed can be change at any time.
- Reading image (or a frame of video) from SD card. Video/image is stored in SD card instead of MCU ROM. To play different video, just modify the constent of SD card, no need to re-program the MCU. Adds a lot of flexibility, but the data reading overhead is considerable.
- Rendering image (or a frame of video). Use PWM signals to ignite LEDs, one LED (three colors) at a time (scanning).

The code is in C instead of Arduino. Although Arduino can be used, but it is TOOOOOO inefficient. Time is critical on this application because of high system load. We need speed, and we need to use interrupts, we need to write higly optimized code.

### Measuring rotation speed
A MPU-6500 gyrometer is connected to the MCU via SPI bus.

The MSU will query the gyrometer constantly in order to determine the real-time roattion speed. After fetch the rotation speed, the MCU can estimate the current phase of the arm; therefore, the rendering image will not spin. The purpose is that, the spinning LED display can be mounted on any rotation device (at reasonably rotation speed). The rotation speed of this device dose not need to be constant.

To measure the rotation speed, simply send the address with "read" command to the sensor and then read the data.

### Reading image (or frame of video)
The image or video is stored in a SD card, the MCU need to find someway to read the image from the SD card.

All SD card provides 2 communication interface, SPI and SD. SD requires native hardware support (and it is closured); therefore, SPI is used.

By default, data in SD card is grouped by block, each block contains 512 byates of data. To read a block, send the address of the block with "read" command to the sensor and then read the data block. Because each image/frame is 1KB, we need to read 2 block every time.

To avoid conflic between reading-while-rending (reading during rending, so half of the rendering image comes from previous frame and the other half comes from next frame), the MCU uses double-buffer. While reading date from SD card and writing to buffer A, image bitmap in buffer B  is rendering, and vice versa.

SPI bus is fast, polling is used simplfy the program and lower interrupt overhead.

### Rendering image (or a frame of video)
The MCU will ignite one of the 8 LEDs in a loop.

TO DO... A picture may help to describe how this work.

_For details about how the MCU work, refer to the comments in code_
