extern volatile unsigned char tempState; // state0=OK state1=WARM state2=HOT
extern volatile unsigned char oldTempState;
extern volatile unsigned char ledBlinkState; // state0=off state1=on
extern volatile unsigned char ledTimerFlag;
extern volatile unsigned char local,remote;
extern volatile int _PWM_width;
extern volatile int tempInFahrenheit;
extern volatile int tempInCelsius;
extern volatile unsigned char decimal;
extern volatile unsigned char modeState;//local state0=local, state1=remote
extern volatile unsigned short freq;

//timer for buzzer
void timer0_init(unsigned short m);

//timer for LEDs
void timer1_init(unsigned short m);

//timer for servo motor
void timer2_init(void);
