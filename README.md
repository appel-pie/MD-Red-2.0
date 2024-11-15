Rust Firmware for UTSME VCU MD-4.0

Pinning: subject to change

Physical Pin   Pin Name   Hardware Connection   Chip Connection   Exterior Connection   NOTES
---------------------------------------------------------------------------------------------
41             PE10       ADCPIN3               ADC3_IN14         60
42             PE11       ADCPIN4               ADC3_IN15         53
43             PE12       ADCPIN5               ADC3_IN16         46
40             PE9        ADCPIN2               ADC3_IN2          41
44             PE13       ADCPIN6               ADC3_IN3          40
45             PE14       ADCPIN7               ADC4_IN1          52
46             PE15       ADCPIN8               ADC4_IN2          59
39             PE8        ADCPIN1               ADC4_IN6          47
82             PD1        CAN RX                CAN_RX            5    note: abstract connection to 4&5
81             PD0        CAN TX                CAN_TX            4    note: abstract connection to 4&5
96             PA9        DIN6                  GPIO INPUT        30
95             PB9        DIN5                  GPIO INPUT        29    start button
1              PE2        DIN1                  GPIO INPUT        20
2              PE3        DIN2                  GPIO INPUT        12
3              PE4        DIN3                  GPIO INPUT        13
4              PE5        DIN4                  GPIO INPUT        21
7              PB8        DOUT5                 GPIO OUTPUT       6     rtdsound
9              PC14       DOUT3                 GPIO OUTPUT       23
8              PC15       DOUT4                 GPIO OUTPUT       22
5              PE6        DOUT6                 GPIO OUTPUT       14
11             PF10       DOUT1                 GPIO OUTPUT       7
10             PF9        DOUT2                 GPIO OUTPUT       15
86             PD5        SD DETECT             GPIO_INPUT        NC
91             PB5        I2C_SMBAI             NC                TEST HEADERS
92             PB6        I2C_SCL               NC                TEST HEADERS
93             PB7        I2C_SDA               NC                TEST HEADERS
78             PC10       USART TX              NC                TEST HEADERS
79             PC11       USART RX              NC                TEST HEADERS
80             PC12       USART CK              NC                TEST HEADERS
84             PD3        SPI_MISO              NC                NC
85             PD4        SPI_MOSI              NC                NC
87             PD6        SPI_NSS               NC                NC
88             PD7        SPI_SCK               NC                NC
12             PF0        OSC_IN                RCC_OSC_IN        
13             PF1        OSC_OUT               RCC_OSC_OUT       
72             PA13       SWIO                  SYS               PROGRAMMING
76             PA14       SWCLK                 SYS               PROGRAMMING
77             PA15       JTDI                  SYS               PROGRAMMING
89             PB3        JTDO                  SYS               PROGRAMMING
90             PB4        NJTRST                SYS               PROGRAMMING
68             PA11       PWM_OUT1              TIM1_CH2          34    RTDS
69             PA12       PWM_OUT2              TIM1_CH3          25
65             PC9        TMR_IN2               TIM3_CH3          17    FRWHEEL
66             PC13       TMR_IN1               TIM3_CH4          9     FLWHEELSPEED: Not pwm: frequency measurement
70             PA10       TMR_IN4               TIM4_CH1          16    spare timerinput
71             PC8        TMR_IN3               TIM4_CH2          8     spare timerinput
