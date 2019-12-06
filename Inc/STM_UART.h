#include "main.h"

#define BUFSIZE 256

#define Serial1_available() Serial_available(0)
#define Serial2_available() Serial_available(1)
#define Serial3_available() Serial_available(2)
//#define Serial4_available() Serial_available(3)

#define Serial1_read() Serial_read(0)
#define Serial2_read() Serial_read(1)
#define Serial3_read() Serial_read(2)
//#define Serial4_read() Serial_read(3)

#define ClearBufferUart1() ClearBufferUart(0)
#define ClearBufferUart2() ClearBufferUart(1)
#define ClearBufferUart3() ClearBufferUart(2)
//#define ClearBufferUart4() ClearBufferUart(3)

#define Serial1_readByte(ptr, time) Serial_readBytes(0 ,ptr, time)
#define Serial2_readByte(ptr, time) Serial_readBytes(1 ,ptr, time)
#define Serial3_readByte(ptr, time) Serial_readBytes(2 ,ptr, time)
//#define Serial4_readByte(ptr, time) Serial_readBytes(3 ,ptr, time)

#define Serial1_readByte_String(ptr, time) Serial_readByte_String(0 ,ptr, time)
#define Serial2_readByte_String(ptr, time) Serial_readByte_String(1 ,ptr, time)
#define Serial3_readByte_String(ptr, time) Serial_readByte_String(2 ,ptr, time)
//#define Serial4_readByte_String(ptr, time) Serial_readByte_String(3 ,ptr, time)

//extern buf_st rbuf_uart[];

void Serial1_Write(char str);
void Serial2_Write(char str);
void Serial3_Write(char str);
//void Serial4_Write(char str);
void Serial1_WriteByte(char * str, int length);
void Serial2_WriteByte(char * str, int length);
void Serial3_WriteByte(char * str, int length);
//void Serial4_WriteByte(char * str, int length);
void Serial1_Print(char * str);
void Serial2_Print(char * str);
void Serial3_Print(char * str);
//void Serial4_Print(char * str);
void Serial1_Println(char * str);
void Serial2_Println(char * str);
void Serial3_Println(char * str);
//void Serial4_Println(char * str);
	
unsigned char kbhit_Uart(int uartCH);
int getbyteUart(int uartCH);
void ClearBufferUart(int uartCH);
unsigned int Serial_available(int uartCH);
char Serial_read(int uartCH);
unsigned int Serial_readByte_String(int uartCH, volatile unsigned char *Byte , int timeOut);
unsigned int Serial_readBytes(int uartCH, volatile unsigned char *Byte , int timeOut);