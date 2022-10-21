## <p align="center">Files for the HDD Persistence of Vision Clock V4.1 Driver Board</p>
<p align="center">
  <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_top_edit.png" width="309"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/HDDCLK_V4.0_Driver%20Board_bottom_edit2.png" width="300">
</p>

---

### <p>Improvements for next revision/Testing Notes:</p>
- Increase vias on the thermal pad of the motor driver chip (U9) from 5 to 8
- R18 change value from 3.3kohm to 6.2kohm
  - This limits the motor current to just over 1A. The motor was previously limited at 2A which was leading to excessive heat.
  - This value will likely need further adjustment/refining.
- Send PWM signal going to motor driver through a buffer. 
  - The datasheet says the HIGH level of PWM for the chip is 2.7V. I'm sending it 3.3V which should be enough. Shifting that up to 5V would be best.
- R10 change value from 0ohm_NA to 0ohm
  - Not placing this resistor results in the processor not receiving the signal.
