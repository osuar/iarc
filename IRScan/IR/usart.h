#ifndef USART_H 
#define USART_H

#include "all.h"

void setup_usart(void);
void sendchar(char input);
void sendstring(char* string);

#endif
