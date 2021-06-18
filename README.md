# Audio-Filtering-on-STM32-Using-CMSIS-DSP <img src="https://github.com/favicon.ico" width="48">

* [Introduction](#Introduction "Goto Introduction")
	* [Filter Specification](#filter-specification)
	* [Hardware](#Hardware "Goto Hardware")
	* [Software](#Software)
* [Pinout & Configuration](#pinout-and-configuration)
* [Design and Generate Filter Coefficients](#design-and-generate-filter-coefficients)
* [Code Configuration](#code-configuration)
* [Demo](#Demo)
* [Reference](#Reference)

## Introduction
<p align="center">
  <img width="460" width="500" src="https://drive.google.com/uc?export=view&id=1F5om0grdVrSmSDxNLPT0yRsJCK26UT5Z">
</p>

<div style="text-align: justify"> STM32F446RE Nucleo Board is the main microcontroller unit used to realise the implementation of the FIR filter in this project. The micro SD card act as external storage to keep the audio data file in txt format which consists of 32 bits floating-point numbers. The audio file is originally in .wav format and it is converted to text format to allow the Nucleo board to read the data. The micro SD card adaptor is utilized to allow the STM32 Nucleo board to access the micro SD card via a Serial Peripheral Interface (SPI) connection. The SPI is communicated in full-duplex mode using a master-slave architecture with a single master or microcontroller unit. The data transmitted into the board is managed by the middleware called File Allocation Table File System (FATfs) which is widely used in memory card and USB storage drive. The status of the process is updated on user monitoring via the Universal Asynchronous Receiver Transmitter (UART) interface. The output audio data file is written back to the SD card once the FIR processing on the board is done.  </div>


### Filter Specification

Declaration | Value
------------ | -------------
Cut-off Frequency | 2000 Hz
Fs  | 48000Hz
Fpass | 0 Hz - 2000 Hz
Fstop | 5000 Hz - 24000 Hz
TEST_LENGTH_SAMPLES | 19084 (number of input data)
BLOCK_SIZE | 1
NUM_TAPS | 17

### Hardware
* [STM32F446RE](https://my.element14.com/stmicroelectronics/nucleo-f446re/dev-board-arduino-mbed-nucleo/dp/2491978)
* [SD Card Module](https://my.cytron.io/p-5v-compatible-micro-sd-card-adapter?r=1&gclid=Cj0KCQjw5auGBhDEARIsAFyNm9FTleQynj9c2YMi_SlpQxx_k29wvPSq-7eCfkw4wXL21nuOy9up6-YaAr2vEALw_wcB)

### Software
* [STM32 Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html)
* [CMSIS DSP Library](https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html)
* [MATLAB / Online Filter Design Tool](http://t-filter.engineerjs.com/)
* [Putty](https://www.putty.org/)
* [Audacity](https://www.audacityteam.org/download/)

## Pinout and Configuration
In this project, we store the input and output of the FIR filter in an SD card. Therefore, we need to set the interface between the SD card module and STM32.


SD card Pin | Board Pinout
------------ | -------------
VCC | 5V
GND | GND
SCK | PA5
MISO | PA6
MOSI | PA7
CS | PB6

![image](https://drive.google.com/uc?export=view&id=1-tnJp2qxqtKqWUBYj7oHB8N2W06XAzPM)

1. First, create a new project using **STM32 cube IDE**. In the pinout & configuration, we will connect our SD card using the **SPI1** pin. After enabling the **SPI1** connection, our **SCK** pin will be auto-assigned to **PA5**, **MISO** pin to **PA6** and **MOSI** pin to **PA7**. In the **SPI1** setting, we need to set it to **Full Duplex Master** so that both master and slave can send data at the same time via the **MOSI** and **MISO** lines respectively. Then, change the clock **prescaler** to **128** to reduce the baud rate of the SD card.
![image](https://drive.google.com/uc?export=view&id=1EaPiVOsrSBZUkEBvWYULJtvqYuIwCCAV)

2. Connect the **CS pin** to **PB6** using a **GPIO** connection and label it as **SD_CS**. 
![image](https://drive.google.com/uc?export=view&id=1mWyYHbNRitWhCUE6k7VqjKCNUJzart9Y)

3. Next, communicate the SD card module with the **FAT file system** provided by **Middleware**. We changed the **maximum sector size** from **512** to **4096** so that it can support any type of memory card.
![image](https://drive.google.com/uc?export=view&id=1Ig2WFhUm7H_QoGFrYuFp4Sz9_M2MPK6V)

4. In this project, we used the serial monitor to debug our connection and check our data transfer status. We connect the board to the serial monitor using default UART, **USART2**. The UART mode is set to **asynchronous** because UART interface does not use a clock signal to synchronize the transmitter and receiver devices and the baud rate was changed to **9600 Bits/s**.
![image](https://drive.google.com/uc?export=view&id=1Eyn-ehQ9J5qlj5CWtkCStOn1dViSzqVR)

5. In the **system mode** setting, the debug state changed to **serial wire** mode. For the **RCC** setting, we used a **crystal/ceramic resonator** to generate our clock. While for the **clock configuration**, we set it to **maximum frequency,180 MHz**.
![image](https://drive.google.com/uc?export=view&id=1asDazAoJb80-9pazVPPvquZPmpzHkZRH)


## Design and Generate Filter Coefficients
Design the filter using this [website](http://t-filter.engineerjs.com/). In this project, we set the spec to:

from | to | gain | ripple/att.
------------ | ------------- | ------------- | -------------
0 Hz | 2000 Hz | 1 | 5db
5000 Hz | 24000 Hz | 0 | -30db

![image](https://drive.google.com/uc?export=view&id=1n-QvVK68c4sYVBDQeRxgq3H2zg7y4Nu5)


## Code Configuration
First, we need to import the FatFS SD card function ([fatfs_sd.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/Core/Src/fatfs_sd.c) and [fatfs_sd.h](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/Core/Inc/fatfs_sd.h)) to our project and include it in the header of [user_diskio.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/FATFS/Target/user_diskio.c) and [main.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/Core/Src/main.c). In the [user_diskio.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/FATFS/Target/user_diskio.c) Private Functions, we return all the data to the controller through SPI1 using the function we just added.

![image](https://drive.google.com/uc?export=view&id=1WeuHrJ9EqGdiCIoXrSa3NyLPUgRxS5fb)

In the [main.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/Core/Src/main.c) file, we can check our SD card connection status using the **f_mount** function and print the status to the serial monitor using the **send_uart** function. After that, we enter our main loop and start to read-write data to our SD card and monitor through the serial monitor. 
![image](https://drive.google.com/uc?export=view&id=112BfY8nDqbCqb6weL3dMoYHH_Q04gTa3)

After include and set up the path of **CMSIS DSP** library ([Tutorial](https://www.youtube.com/watch?v=MUZj4YwKVac)), we will use these two function to implement FIR in our project which are **arm_fir_f32** and **arm_fir_init_f32**. The **arm_fir_init_f32** is an associated initialization function for each data type. The initialization function has two main function. It is used to set the values of the internal structure fields and initialize the values in the state buffer. If the initialization function is used, then the instance structure cannot be placed into a constant data section. There is a total of five parameters are needed for this function. First, **S** is points to an instance of the floating-point FIR filter structure. **numTaps** is number of filter coefficients in the filter. **firCoeffs32** points to the filter coefficients buffer which are the values of the impulse response generated from the online website. **firStateF32** points to the state buffer, **blockSize** is the number of samples processed per call.
<p align="center">
  <img width="460" width="500" src="https://drive.google.com/uc?export=view&id=1PT6Tzzc9kTFGCWXDy2PPrordUn2x0eRZ">
  <img width="460" width="500" src="https://drive.google.com/uc?export=view&id=1r95EVXRNVaeYlrUdxwGbjHZHQ6BsN2WT">
</p>

This figure below shows the use of FIR function in our project [main.c](https://github.com/mkel1123project/Audio-Filtering-on-STM32-Using-CMSIS-DSP/blob/main/Core/Src/main.c) file. All the variable and parameter are first declared. Then, the fir function is called in the main function and pass all the parameter and variables accordingly for the FIR filter algorithm computation.

![image](https://drive.google.com/uc?export=view&id=1vNIKY9FaSpKQk4HOTZs3POr_hmLzwz4v)

## Demo
![image](https://drive.google.com/uc?export=view&id=1pgWNbIdR0qd9oOSIJLP_NhTVURY4pk1n)
![image](https://drive.google.com/uc?export=view&id=1V5Ugwxx4swL1DG3PDiJEJa-NEiepUN8_)
[![video](https://drive.google.com/uc?export=view&id=1LkcOgwU_gme9NritiUeSz7BUAlIBikQl)](https://www.youtube.com/watch?v=SxUf0OrASGo)



## Reference
1. <https://os.mbed.com/platforms/ST-Nucleo-F446RE/>
2. <https://controllerstech.com/sd-card-using-spi-in-stm32/>
3. <https://www.keil.com/pack/doc/CMSIS/DSP/html/index.html>
4. <https://www.youtube.com/watch?v=nAot1FCgYGo>





