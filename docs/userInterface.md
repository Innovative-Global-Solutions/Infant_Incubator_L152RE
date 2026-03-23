# User Interface (UI)

## Overview
The UI consists of the rotary encoder, 5 buttons, and an LCD screen. The five display menus are: Home, Humidity, Incubator Temperature, Infant Temperature, and Heart Rate. Excluding Home, all screens display a live sensor value, minimum bound, and maximum bound. If either the minimum or maximum bound are exceeded, the alarm system will sound. 

![UI Block Diagram](https://gcdnb.pbrd.co/images/cmTBl1sH7x2K.png?o=1)

**Block Diagram of UI System**

As illustrated in the diagram, all components are linked through the STM32 MCU. The rotary dial operates in two directions when any button is pressed: counterclockwise or clockwise. Turning the dial counterclockwise decreases the currently selected bound (Min. or Max.), while turning it clockwise increases the bound value.

Then, all buttons operate under the same logic. When no button is pressed, the LCD displays the Home menu by default. Pressing a button causes a selection icon to appear on the minimum bound initially. Pressing the button again moves the icon to the maximum bound. If the button is not pressed at any step, the screen remains unchanged.


