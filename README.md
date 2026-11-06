# ECE 528/L - Lab 3: Line Follower

**CSU Northridge**

**Department of Electrical and Computer Engineering**

## Overview

This lab uses the MSP432 LaunchPad and the QTRX reflectance sensor array to
follow black tape. The sensor readings tell the robot when to drive straight,
adjust its steering, or turn.

## Components Used

- TI MSP432P401R LaunchPad
- TI-RSLK MAX chassis and DC motors
- Eight-channel QTRX reflectance sensor array
- AA batteries
- Black tape and a white track surface
- USB cable and Code Composer Studio

## Pre-Lab Assignment

#### 1. Configure P8.0, P8.5, P8.6, and P8.7 as GPIO outputs and initialize them to zero.

```c
void Chassis_Board_LEDs_Init(void)
{
    P8->SEL0 &= ~0xE1;
    P8->SEL1 &= ~0xE1;
    P8->DIR |= 0xE1;
    P8->OUT &= ~0xE1;
}
```

The mask `0xE1` selects the four chassis LED pins. Clearing both select
registers sets them to GPIO, and clearing `OUT` starts with the LEDs off.

#### 2. Print every tenth index and the index after it from 10 to 51.

```c
void Print_Tenth(void)
{
    for (int i = 10; i <= 51; i++)
    {
        int n = i % 10;
        if (n == 0)
            printf("Index %d: Every 10th iteration.\n", i);
        else if (n == 1)
            printf("Index %d: Following the 10th iteration.\n", i);
    }
}
```

The full program is in [print_tenth.c](print_tenth.c).

## Analysis and Results

#### Line Following

Timer A1 reads the sensor array every 2 ms. When the robot is centered,
both motors get the same PWM. When it drifts, the code slows one motor
to steer back toward the tape. The main loop is empty.

#### Turn Detection

The code checks new sensor readings for 12 ms before choosing a turn.
Two readings in a row must agree. If all eight sensors read black twice
in a row, the T-intersection takes priority over a regular corner. This
gave the sensor array time to reach the full width of the T.

#### Demo Video

[Watch the robot follow the course and return to the starting end](line_follower_demo.mp4).

## Known Issues or Limitations

The turns are timed, so battery condition affects how far the robot rotates.
As the batteries ran down, I used about 4 seconds for the U-turn and 2 seconds
for the corners. After replacing them, the motors turned faster, and I reduced
the times to 2 seconds for the U-turn and 1 second for the corners.

## Author Contribution

I worked on the pre-lab program, steering and turn logic, and track testing.
I recorded the demo and prepared the README.

## References

- ECE 528/L Lab 3: Line Follower
- [Pololu QTRX Sensor Array](https://www.pololu.com/product/3672)
- [Pololu Gearmotor with Encoder](https://www.pololu.com/product/3675)

