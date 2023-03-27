# **Sphere Grinding Machine**

## **Overview**
The Arduino sketch was developed for a self-built grinding machine to automate the grinding process. It provides a menu structure on an OLED display and enables the user to control the grinding machine with 3 stepper motors and 4 DC motors. It includes functions for initializing the grinding machine as well as creating and storing grinding programs.

The stepper motors are part of the lowering device. Rotation causes the support plane (on which the upper grinding wheel rests) to be raised or lowered. In this way, the distance between the upper and lower grinding wheel can be changed. To ensure that the grinding wheels are parallel to each other, the stepper motors are controlled simultaneously - except during initialization, when the upper grinding wheel is leveled by individual control of the stepper motors. Each of the two grinding wheels is driven by a DC motor. Another two DC motors drive the rotors of two peristaltic pumps.  

## **Hardware**
![](https://github.com/sven-johannsen/Sphere-Grinding-Machine/blob/main/img/hardware.png)

Hardware Components:  

- PJRC Teensy 4.0 microcontroller 
- 3 x Allegro A4988 stepper driver   
- 2 x Toshiba TB6612FNG DC driver (each controls 2 DC motors)  
- JOY-IT 0,96" I²C OLED Display (128x64, SSD1306)
- 3 x MFA/Como Drills 919D series DC motor
- 3 x 11HS20-0674S-PG14 stepper motor
- 4 x OFF-(ON) push button  

## **Menu Structure**
![](https://github.com/sven-johannsen/Sphere-Grinding-Machine/blob/main/img/menuStructure.png)

The user can navigate through the menu via four push buttons and make the desired settings. The menu consists of three levels. The submenus are accessible via the main menu. The "DC Motor Programs" submenu contains a further submenu with eight grinding programs for the DC motors. Each grinding program includes five consecutive time intervals for each DC motor. The intervals consist of a power value (duty cycle) and a corresponding time value during which the power is applied. The resolution of the values is 1% or 1 second, respectively.  

## **Menu Elements**
![](https://github.com/sven-johannsen/Sphere-Grinding-Machine/blob/main/img/menuItem.png)

The display can show up to four lines. If necessary, the user can scroll up and down accordingly. A distinction is made between the following types of menu elements: submenu, function call and grinding parameter. If the menu element represents a grinding parameter, the corresponding value is right-aligned next to the element name. In case of a function call, the abbreviation "EXEC" is displayed instead of a value. In the above example the cursor (">") points to the menu item related to the speed of the stepper motors ("Micrometer/h"). Pressing the enter button moves the cursor to the value on the right side and allows for adjustments of that value.
