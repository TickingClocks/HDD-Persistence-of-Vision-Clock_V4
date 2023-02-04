<img src="Images/banner.png">
<!--maybe add some badges here?-->

#### <p align="center">HDD converted into a persistence of vision display clock<p>

<p align="center">V4.1 Prototypte Slit Mask:<p>

<!--gif of the protytpe-->
   <p align="center" >
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Prototype%20gif3.gif" height:"190">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/transparent%20image.png" height:"200" width="30">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Prototype%20gif6.gif" height:"190">
   </p>
   <p align="center">V4.1 Prototypte Nipkow 12 Hole Mask, line and "HELLO WORLD":<p>
   <p align="center">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Nipkow%20Prototype%20gif1.gif" height:"190">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/transparent%20image.png" height:"200" width="30">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Hello%20World%20Nipkow%20Gif%20Test%202.gif" height:"190">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/transparent%20image.png" height:"200" width="30">
      <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Hello%20World%20Nipkow.jpg" height:"190" width="500">

   </p>
      
---
      
<p align="center"><a href="https://youtu.be/2cXJMgh0BA4" target="_blank" rel="noreferrer noopener"><i>V2.0 Project YouTube Video</i></a></p>

---

## Project Explanation
The front face of a HDD is removed exposing the internals. The platters are replaced with a stationary LED board and a spinning light mask. The LED board sits against the HDD frame, stationary. The light mask is mounted to the HDD motor.

There is an IR reflective sensor mounted on the LED board that keeps track of an index point on the bottom of the light mask.

The light mask spins quickly, the processor on the Driver Board monitors the sensor and tracks the period of time it takes for a full rotation. The LEDs can then be controlled with precise timing to allow light to escape the holes in the light mask when the light mask is at its desired rotational angle. Doing this quickly enough can trick the eye into seeing an image. This is called <a href="https://en.wikipedia.org/wiki/Persistence_of_vision" target="_blank" rel="noreferrer noopener">_persistence of vision_</a>.

There are 2 types of light masks designed for the clock. 
* The <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/PCB%20Light%20Masks/Slit%20Mask" target="_blank" rel="noreferrer noopener"><i>first</i></a> uses a slit going from the center of the platter to the edge. This allows me to draw lines radiating from the center of the display. I use this mask to draw an analog clock.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/slit%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/slit%20mask_back.png" width="250">
</p>

* The <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/PCB%20Light%20Masks/Nipkow%20Light%20Mask" target="_blank" rel="noreferrer noopener"><i>second</i></a> uses what is known as a <a href="https://en.wikipedia.org/wiki/Nipkow_disk" target="_blank" rel="noreferrer noopener">_Nipkow Disk_</a>. This is a disk that has holes spiraling from the center at a constant interval. Each of these holes can then be used to draw pixels on the display. Each hole corresponds to a horizontal line on the display, the vertical pixels are then placed by rotating the light mask and pulsing a backight. One light mask has 8 holes, the other has 12.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_back.png" width="250">
</p>
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_back.png" width="250">
</p>
<p align="center">*Light masks are PCBs. Actual ordered PCBs have black solder mask. V4 clock uses the same light masks as V3.</p>

---

## Hardware Breakdown
This project is a clock that uses a hard drive to create a persistence of vision display.

The project has 2 PCBs
* <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/Driver%20Board" target="_blank" rel="noreferrer noopener">_Driver Board_</a>
    * This board replaces the original PCB.
    * The Driver Board has an <a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f411.html" target="_blank" rel="noreferrer noopener">_STM32F411_</a> ARM M4 processor running at 100MHz. The driver board controls the whole system from the motor driver chip, LEDs, Temperature sensors, human interface, etc.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_top_edit.png" width="255"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_bottom_edit2.png" width="250">
</p>

* <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/LED%20Board" target="_blank" rel="noreferrer noopener">_LED Board_</a>
    * This board replaces the bottom platter in the HDD. A 3D printed spacer is used to elevate the light mask, which replaces the second platter.
    * The LED board contains 116 <a href="http://www.led-color.com/upload/201604/APA102-2020%20SMD%20LED.pdf" target="_blank" rel="noreferrer noopener">_APA102-2020_</a> LEDs in a circular pattern. It also has an <a href="https://ams-osram.com/products/sensors/position-sensors/osram-reflective-interrupter-sfh-9206" target="_blank" rel="noreferrer noopener">_IR reflective sensor_</a> which can be used to either track a shiny pad on the light mask, or the absence of one, depending on the light mask used. The LED board also has a board mounted temperature sensor.


<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_LED%20Board_Top_edit.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_LED%20Board_bottom_edit.png" width="250">
</p>

The electrical connections between the PCBs are made between spring-loaded header pins mounted on the driver board and contact points on the back of the LED board.

---

## Assembly

<!--gif of the protytpe-->
   <p align="center" >
<img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Prototype%20gif4.gif" height:"250"><img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/transparent%20image.png" height:"250" width="30"><img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Prototype%20gif5.gif" height:"250">
      </p>
      
   <p align="center">*Above shows Driver Board V4.1 and LED Board V3.1</p>
   
---
---

## Firmware Breakdown

###Update
A new version of the Firmware will be uploaded soon which includes code to run the Nipkow mask. I'm currently in final stages of testing.

### Firmware Flow Diagram
See the <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Firmware/Spinning%20Vision%20Flow%20Diagram.pdf"><i>Spinning Vision Flow Diagram</i></a> for a visiual outine of the code flow.

### Overview
<a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f411.html"><i>STM32CubeMX</i></a> was used to generate the microcontroller setup code and files. See the peripheral settings I used further down this readme for setup information. I'm not finished writing the firmware yet. The firmware I've uploaded has the following features:
* Persistence of vision
  * [x] Slit mask
  * [ ] Nipkow mask
* Devices
  * [x] RTC
  * [x] Temperature Sensors
  * [x] Motor controller
* Interface
  * [x] Momentary buttons (x3)
  * [x] Rotary encoder
* Display
  * [x] Analog clock animation
  * [ ] Nipkow animation
* Configuration 
  * [x] Manual time setting
  * [ ] Multiple clock faces

## Hardware Implemented in Firmware (so far)
* Code outine in .c and .h files was generated with <a href="https://www.st.com/en/development-tools/stm32cubemx.html"><i>STM32CubeMX</i></a>.
* Overview of components configured and used in the uploaded .c main file:

|Hardware/Component              |Configured  |Implemented   |Detail                                |
|--------------------------------|------------|--------------|--------------------------------------|
|APA102-2020 LEDs                |YES         |YES           |2 Lines at 25MHz. Display and rear LED|
|DRV1173 BLDC motor driver       |YES         |YES           |PWM frequency 10kHz (TIM2 CH1)        |
|DS1307 RTC                      |YES         |YES           |I2C, basic implementation             |
|NCT175 temperature sensor       |YES         |NO            |I2C, 3 sensors, basic implementation  |
|Right-angle momentary buttons   |YES         |YES           |X3 buttons                            |
|Right-angle rotary encoder      |YES         |YES           |24 pulses/rotation, TIM5 encoder mode |
|GPIO LED                        |YES         |NO            |not used surface mount LED            |
|USB-C                           |NO          |NO            |-                                     |

---

## Microcontroller Configuration
* <a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f411.html"><i>STM32F411RET6</i></a>
<p align="center"><img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/STM32F411%20Pins.png" width="500px"></p>
* Microcontroller is running at 100MHz using a 16MHz external crystal.

## Timers
|Timer   |Configuration                                         |Function                                      |
|--------|------------------------------------------------------|----------------------------------------------|
|TIM2    |PWM Generation CH1                                    |Motor speed signal                            |
|TIM3    |Gen Timer at 1MHz                                     |Microsecond timer used for display algorithm  |
|TIM5    |Encoder input                                         |Encoder input                                 |
|TIM11   |Gen Timer at 1MHz, interrupt overflow at 1000 pulses  |Interrupt at every 1ms for ms counter         |

## Connectivity
|Peripheral  |Configuration                |Function                            |
|------------|-----------------------------|------------------------------------|
|I2C1        |Standard speed               |Communication: RTC, Temp sensors    |
|SPI2        |Master Transmit Only, 25MHz  |Rear LED Communication              |
|SPI5        |Master Transmit Only, 25MHz  |Display LED Communication           |
|USB         |Not configured yet           |Serial communication for debugging  |

## GPIO
|Pin    |Configuration    |Given Name            |Function                                             |
|-------|:---------------:|----------------------|-----------------------------------------------------|
|PA2    |INPUT            |TempAlert1            |Temperature alert pin for Driver Board temp sensor   |
|PA3    |INPUT            |MotorFG               |Motor speed feedback signal                          |
|PA4    |INPUT            |MotorDirection        |Reads direction setting (set in hardware)            |
|PA6    |INPUT            |fastMotor/slowMotor   |Reads speed setting (set in hardware)                |
|PA7    |INPUT            |motorFault            |Reads the motor fault output                         |
|PB12   |INPUT            |button1               |Input for button 1                                   |
|PB13   |INPUT            |button2               |Input for button 2                                   |
|PB14   |INPUT            |button3               |Input for button 3                                   |
|PB15   |INPUT            |encoder_button        |Input for the encoder button                         |
|PC0    |INPUT            |TempAlert2            |Temperature alert pin for the display temp sensor    |
|PC9    |INPUT            |testLED               |Basic LED for functional testing                     |
|PC10   |INPUT            |opticalSwitchBuffOut  |Optical sensor output (sent through a buffer)        |

## DMA

* DMA is not used yet. This microcontroller has 2 DMA controllers. I'm planning to use them for:
  * I2C communication
  * SPI communication
  
---
---

## What’s Next?
I'm developing this project in my spare time. Updates will come in batches.

* Hardware
    * [ ] <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/Driver%20Board" target="_blank" rel="no referrer noopener">_Driver Board V4.1_</a>
        * [x] choose new motor driver chip
        * [x] choose new DC jack
        * [x] Fix SPI lines
        * [x] fix via placements
        * [x] fix air temp sensor routing
        * [x] fix motor connector orientation
        * [x] update IR sensor signal resistors
        * [x] order PCBs and parts
        * [x] assemble PCB
        * [x] test hardware (hardware is good)
        * [ ] update documentation (if needed)
    * [ ] <a href="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/tree/main/Electrical/LED%20Board" target="_blank" rel="no referrer noopener">_LED Board V4.0_</a>
        * [x] pick new IR reflective sensor
        * [x] order PCBs and parts
        * [ ] assmble PCB
        * [ ] test hardware
        * [ ] update documentation (if needed)

* Firmware
    * Devices
        * [x] temperature sensors
        * [x] real time clock
        * [x] LEDs
        * [x] motor driver chip (V4.1)
    * Persistence of vision (basic)
        * [x] slit light mask
        * [ ] Nipkow Disk
            * [x] Prove concept
            * [ ] Optomise 
