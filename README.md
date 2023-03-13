# PWM Signal Generator and Monitor

The goal of this project was to develop a system that monitors and controls a generated pulse-width modulated (PWM) signal using the `STM32F0`.

It operates by driving a `4N35` optocoupler to control the frequency output of a `NE555` timer. The desired frequency to be output is controlled using a potentiometer on the `PBMCULSK` board and read by the microcontroller which in turn controls the output to the optocoupler. The output is measured by the controller and displayed on the `Hitachi HD44780` Liquid Crystal Display (LCD).

Therefore this project has three sections to it.

## The Analog to Digital Converter (ADC)

An Analog to Digital Converter (ADC) was be used to measure the voltage levels output by the Potentiometer. The digital value was then converted back to an analog value using the Digital to Analog Converter in the microcontroller (DAC).

## The Digital to Analog Converter (DAC)

The DAC output drives the Optocoupler which controls the frequency output of the Timer that is fed back to the microcontroller and read as an edge triggered external interrupt to accurately determine the signal frequency.

## The Liquid Crystal Display

The LCD displays the values of the resistance and the corresponding frequency.
