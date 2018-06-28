#include "Utitils.h"

uint8_t PWM_duty(uint8_t value, uint8_t max)
{	
	int result;
	result = value*8399/10000;	
	return result;
}

char* stringcat(char *dich, const char *nguon)
{
  int i=0,j=0;
  while (dich[i]!='\0') i++;
 
  while (nguon[j]!='\0') dich[i++]=nguon[j++];
  dich[i]='\0';
  return dich;
}
