#ifndef USART_H 
#define USART_H



void setup_usart(void);
void sendchar(char input);
void sendstring(char* string);
char mygetchar(void);

#endif
