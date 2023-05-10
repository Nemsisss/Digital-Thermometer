extern volatile unsigned char changed ;  // Flag for state change
extern volatile int count ;        // Count to display
extern volatile unsigned char new_state, old_state; //states of the rotary encoder
extern volatile unsigned char a, b;

void encoder_init();
