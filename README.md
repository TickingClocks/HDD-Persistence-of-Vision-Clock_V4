<img src="Images/banner.png">
<!--maybe add some badges here?-->

#### <p align="center">HDD converted into a persistence of vision mechanical display clock<p>

<p align="center">V4.0 Prototypte:<p>

<!--gif of the protytpe-->
   <p align="center" >
<img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/Prototype%20gif1.gif" height:"400">
      </p>

---
---

## Project Explanation
The front face of a HDD is removed exposing the internals. The platters are replaced with a stationary LED board and a spinning light mask. The LED board sits against the HDD frame, stationary. The light mask is mounted to the HDD motor.

There is an IR reflective sensor mounted on the LED board that keeps track of an index point on the bottom of the light mask.

The light mask spins quickly, the processor on the Driver Board monitors the sensor and tracks the period of time it takes for a full rotation. The LEDs can then be controlled with precise timing to allow light to escape the holes in the light mask. Doing this quickly enough can trick the eye into seeing an image. This is called _persistence of vision_.

There are 2 types of light masks that I have designed. 
* The first uses a slit going from the center of the platter to the edge. This allows me to draw lines radiating from the center of the display. I use this mask to draw an analog clock.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/slit%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/slit%20mask_back.png" width="250">
</p>

* The second uses what is known as a _Nipkow Disk_. This is a disk that has holes spiraling from the center at a constant interval. Each of these holes can then be used to draw pixels on the display. Each hole corresponds to a horizontal line on the display, the vertical pixels are then placed by rotating the light mask and pulsing a backight. One light mask has 8 holes, the other has 12.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_back.png" width="250">
</p>
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_front.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_back.png" width="250">
</p>
<p align="center">*Light masks are PCBs. Actual ordered PCBs have black solder mask. V4 clock uses the same light masks as V3.</p>
<!--add reference image-->

---

## Hardware Breakdown
This project is a clock that uses a hard drive to create a persistence of vision display.

The project has 2 PCBs
* Driver Board
    * This board replaces the original PCB.
    * The Driver Board has an STM32F411 processor running at 100MHz. The driver board controls the whole system from the motor driver chip, LEDs, Temperature sensors, human interface, etc.
<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_top_edit.png" width="255"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_bottom_edit2.png" width="250">
</p>

* LED Board
    * This board replaces the bottom platter in the HDD. A 3D printed spacer is used to elevate the light mask, which replaces the second platter.
    * The LED board contains 116 APA102-2020 LEDs in a circular pattern. It also has an IR reflective sensor which can be used to either track a shiny pad on the light mask, or the absence of one, depending on the light mask used. The LED board also has a board mounted temperature sensor.


<p align="center">
   <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_LED%20Board_Top_edit.png" width="250"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_LED%20Board_bottom_edit.png" width="250">
</p>

The electrical connections between the PCBs are made between spring-loaded header pins mounted on the driver board and contact points on the back of the LED board.

---

## What’s Next?
I'm currently developing this project in my spare time. Updates will probably come in chunks.

* Hardware
    * [ ] Driver Board V4.1
        * [x] choose new motor driver chip
        * [x] choose new DC jack
        * [x] Fix SPI lines
        * [x] fix via placements
        * [x] fix air temp sensor routing
        * [x] fix motor connector orientation
        * [x] update IR sensor signal resistors
        * [x] order PCBs and parts
        * [ ] Assemble PCB
        * [ ] test hardware
    * [ ] LED Board V4.0
        * [x] pick new IR reflective sensor
        * [x] order PCBs and parts
        * [ ] assmble PCB
        * [ ] test hardware

* Firmware
    * Devices
        * [x] temperature sensors
        * [x] real time clock
        * [x] LEDs
        * [ ] motor driver chip (V4.1)
    * Persistence of vision (basic)
        * [x] slit light mask
        * [ ] Nipkow Disk
