# STM32_ServoDrive
Educational project in the field of motor control

Project goals:
1) more complete study of the microcontroller STM32G4 with Cortex-M4 core 
2) development of motor control algorithms with feedback sensors of various types: halls, encoders, resolvers and without sensors
3) finding the best way to decompose modules and organize data structures for typical embedded project   
4) development of a graphical interface for Windows for configuring, diagnosing and managing hardware

Completed stages:
1) development control system with liniar model of PMSM in Matlab/Simulink (MathACM601.slx and MathACM601.m)
2) development control system with SimPowerSystem model of PMSM in Matlab/Simulink (SimACM601.slx and MathACM601.m)
3) development control system base on STM32G4 with real PMSM and incremental encoder 

Next stage:
4) development of a graphical interface for Windows for configuring, diagnosing and managing hardware
5) devolopment control system base on STM32G4 with real PMSM and senser different type...


Software implement vector control system for PMSM.
Using hardware: 
 PMSM ACM601V36-T2500 with bield-in encoder 2000PPR, 
 bord with inventer x-nucleo-ihm07m1,
 bord with MCU nucleo-g474.

Program execution sequence
1) Program starts with declaration and definition nessesary vars and constants.
2) Then initiation of nessesary peripheral units is executed. These are
 TIM1 for generating PWM signals in up/down mode with 20 kHz frequency and calling ADC1
 TIM2 for geting signal of encoder 
 TIM3 for generating interruption with 2 kHz frequency for handling no importent vars
 ADC1 for measuring phase currents, supply voltage, temperature and voltage of target of speed
 DMA1 for transfering data from ADC1 regulag register to array dataADC1[8] and calling interruption
 USART for communicating with PC in future
 DAC1 for output analog signals of speed and general current
 GPIO for getting and setting signals enable and fault
3) Then encoder is colibrated
4) After initialization forever loop starts and all work implements in interruptions
 current loop is solved in interruption of DMA with frequency 20 kHz
 speed loop is solved also in	interruption of DMA but with frequency 2 kHz
 handling other analog signal and prepering analog output are complited in interruption of TIM3 with frequency 2 kHz
 interface with PC will be implement in interruption of UART 




