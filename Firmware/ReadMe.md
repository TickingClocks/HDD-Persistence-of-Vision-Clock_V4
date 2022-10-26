# NOTE
I'm not finished writing the firmware yet. The firmware I've uploaded has the following features:
* Persistence of vision
  * [x] Slit mask
  * [ ] Nipkow mask
* Devices
  * [x] RTC
  * [x] Temperature Sensors
  * [x] Motor controller
* Interface
  * [x] Momentary buttons (x3)
  * [ ] Rotary encoder
* Display
  * [x] Analog clock animation
  * [ ] Nipkow animation
* Configuration 
  * [ ] Manual time setting
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
|Right-angle rotary encoder      |YES         |NO            |24 pulses/rotation, TIM5 encoder mode |
|GPIO LED                        |YES         |NO            |not used surface mount LED            |
|USB-C                           |NO          |NO            |-                                     |

---

## Microcontroller Configuration
* <a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f411.html"><i>STM32F411RET6</i></a>
<p align="center"><img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/STM32F411%20Pins.png" width="500px"></p>

## Timers
|Timer   |Configuration                                         |Function                                      |
|--------|------------------------------------------------------|----------------------------------------------|
|TIM2    |PWM Generation CH1                                    |Motor speed signal                            |
|TIM3    |Gen Timer at 1MHz                                     |Microsecond timer used for display algorithm  |
|TIM5    |Encoder inpute                                        |Encoder input                                 |
|TIM11   |Gen Timer at 1MHz, interrupt overflow at 1000 pulses  |interrupt at every 1ms for ms counter         |

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
  * I2C communication -  blocking is taking too much time and messing with the display
  * SPI communication - I can hand off the SPI generation for the LEDs to DMA to free up the processor

