# Line-Following and Maze-Solving Robot

## Fully functional line-following and maze-solving programs written in C for the [Pololu 3pi Robot](https://www.pololu.com/docs/0J21/all).

These two projects were created for a university microcontroller course. They were developed and tested using Microchip Studio and deployed and debugged on a Pololu 3pi Robot with an AVR microcontroller through an [ATMEL-ICE](https://www.microchip.com/en-us/development-tool/atatmel-ice). The program uses the 5 reflectance sensors to determine its position on a black line and adjusts its motors accordingly.

### Line Following Project

#### Algorithm

1. Configure the robot's initial state (i.e. turn motors off, set clock frequency, etc).
2. Determine if the 5 reflectance sensors are sensing light or dark with pin-change interrupts.
3. Adjust the left and right motor speeds so that only the middle sensor senses dark.
4. Repeat steps 2-3

#### Watch the Demo

<a href="https://youtu.be/tLwPTCkFpbY" target="_blank">
<img src="https://img.youtube.com/vi/tLwPTCkFpbY/0.jpg" alt="Watch the demo">
</a>

### Maze Solving Project

#### Algorithm

This algorithm extends the line following algorithm to add logic to detect and store turns decisions made at intersections. The robot's approach involves a two-step process: first, it explores the maze, always opting for left turns and going straight over right turns when possible and u-turning at dead ends. Then, during the second traversal, the robot relies on the turn sequence recorded from the initial traversal to navigate the maze without making any incorrect turns. To achieve this, a reduction algorithm is applied to the recorded sequence of turns from the first traversal to derive the correct turn sequence for the second traversal as outlined by this table:

| Turn Sequence          | Correct Decision |
| ---------------------- | ---------------- |
| Left, U-turn, Left     | Straight         |
| Straight, U-turn, Left | Right            |
| Left, U-turn, Straight | Right            |

#### Watch the Demo

<a href="https://youtu.be/NrizJmnjBWc" target="_blank">
<img src="https://img.youtube.com/vi/NrizJmnjBWc/0.jpg" alt="Watch the demo">
</a>

#### Notes

- This algorithm that there are no loops in the maze.
- Blinking the red and green LEDs on the robot were used as a way of real-time debugging during the development of the program.
- The robot's awareness of intersections (from reading from reflectance sensors) is sensitive to the changes of brightness in the environment. Further calibration of the might be needed.
