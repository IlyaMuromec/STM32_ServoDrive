# STM32_ServoDrive
Vector control of PMSM

Project goals:
1) more complete study of the microcontroller STM32G4 with Cortex-M4 core 
2) development of motor control algorithms with feedback sensors of various types: halls, encoders, resolvers and without sensors
3) finding the best way to decompose modules and organize data structures for typical embedded project   
4) development of a graphical interface for Windows for configuring, diagnosing and managing hardware

Hardware
Board nucleo-g474 is used as MCU, board x-nucleo-ihm07m1 is used as invertor and ACM601V36-T2500 is used as Permanent magnet synchronous motor.

Completed stages:
1) development control system with liniar model of PMSM in Matlab/Simulink
2) development control system with SimPowerSystem model of PMSM in Matlab/Simulink
3) devolopment control system with real PMSM and incremental encoder

Next stage:
4) development of a graphical interface for Windows for configuring, diagnosing and managing hardware
5) devolopment control system with real PMSM and senser different type...

