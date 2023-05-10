extern char remoteData[7];
extern volatile int indexCharsReceived;
extern volatile unsigned char remoteValidDataFlag ;
extern volatile unsigned char dataReceiveStarted;

void serial_init();
void tx_char(char next_character);
void transmitString(char* str);


