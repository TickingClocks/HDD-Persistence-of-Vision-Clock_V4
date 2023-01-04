## <p align="center">HDD Persistence of Vision Clock 12-Hole Nipkow Light Mask</p>
<p align="center">
  <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_front.png" width="309"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/12%20hole%20nipkow%20mask_back.png" width="300">
</p>

## <p align="center">HDD Persistence of Vision Clock 8-Hole Nipkow Light Mask</p>
<p align="center">
  <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_front.png" width="309"> <img src="https://github.com/TickingClocks/HDD-Persistence-of-Vision-Clock_V4/blob/main/Images/8%20hole%20nipkow%20mask_back.png" width="300">
</p>

### <p>Improvements for next revision/Testing Notes:</p>
- I've tested the 12 hole platter which worked successfully with 120 slices in 1 revolution which is a screen resolution of 12x120. Unless I run into a processing limit, I dont see any need to go to the 8 hole mask.
- The reflective, plated, pads of the holes are being picked up by the reflective sensor.
    - I used black paint to cover the shiny surface on the bottom side apertures. This worked to eliminate the issues I was seeing.
- The platter spins counter-clockwise because of the motor. Mirroring the aperture pattern (spiraling the opposite way) would make writing the algorithms a little easier.
- The reflective index point can be reduced to ~1.5mm wide.
    - I'll probably keep it as-is since everything is working well.
- Might be worth considering using multiple reflective points of different widths which I could use as an encoder to know the platters position more accurately - less guessing.
    - This would just be for fun, the current method is working well.
