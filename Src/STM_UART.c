#include "STM_UART.h"
//#include "LED.h"

#if BUFSIZE==256
typedef struct {
	unsigned short in : 8;
	unsigned short out : 8;
  unsigned char buf [BUFSIZE];
}buf_st;
#elif BUFSIZE==1024
typedef struct {
	unsigned short in : 10;
	unsigned short out : 10;
  unsigned char buf [BUFSIZE];
}buf_st;
#elif BUFSIZE==2048
typedef struct {
	unsigned short in : 12;
	unsigned short out : 12;
  unsigned char buf [BUFSIZE];
}buf_st;
#endif

buf_st rbuf_uart[4];


extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
//extern UART_HandleTypeDef huart4;
extern uint8_t rbr1 , rbr2, rbr3;//, rbr4;

uint8_t stat1,stat2, stat3;//, stat4; 

//uint8_t rbr;

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart){
// if(huart->Instance == USART1){
//	 //LED2_onBlinlk(10);  //LED2 Blink triger signal 10ms 
// }
// if(huart->Instance == USART2){
//	 //LED2_onBlinlk(10);  //LED2 Blink triger signal 10ms 
// }
//// if(huart->Instance == USART3){
////	// LED2_onBlinlk(10);  //LED2 Blink triger signal 10ms 
////		if(stat3==HAL_OK){
////		 Serial1_Println("UART3 : HAL_OK");
////	 }
////		else if(stat3==HAL_BUSY){      
////			Serial1_Println("UART3 : HAL_BUSY");
////		}
////		else if(stat3==HAL_ERROR){			
////			Serial1_Println("UART3 : HAL_ERROR");	
////		}			
//// }
// if(huart->Instance == USART4){
//	 //LED2_onBlinlk(10);  //LED2 Blink triger signal 10ms 
// }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	
	if(huart->Instance == USART1){
		stat1 = HAL_UART_Receive_DMA(&huart1,&rbr1,1);
		 if ((rbuf_uart[0].in + 1) != rbuf_uart[0].out) {
				rbuf_uart[0].buf [rbuf_uart[0].in++] = (unsigned char)rbr1;
		 }
		  __HAL_UART_FLUSH_DRREGISTER (& huart1);
  }
	
	if(huart->Instance == USART2){
		stat2 = HAL_UART_Receive_DMA(&huart2,&rbr2,1);
		 if ((rbuf_uart[1].in + 1) != rbuf_uart[1].out) {
				rbuf_uart[1].buf [rbuf_uart[1].in++] = (unsigned char)rbr2;
		 }
		 __HAL_UART_FLUSH_DRREGISTER (& huart2);
	 } 
	
	 
	if(huart->Instance == USART3){
		stat3 = HAL_UART_Receive_DMA(&huart3,&rbr3,1);
			if ((rbuf_uart[2].in + 1) != rbuf_uart[2].out) {
						rbuf_uart[2].buf [rbuf_uart[2].in++] = (unsigned char)rbr3;
				 }
		__HAL_UART_FLUSH_DRREGISTER (& huart3);
	 }
	 
	 
//	 if(huart->Instance == USART4){
//		stat4 = HAL_UART_Receive_DMA(&huart4,&rbr4,1);
//		 if ((rbuf_uart[3].in + 1) != rbuf_uart[3].out) {
//				rbuf_uart[3].buf [rbuf_uart[3].in++] = (unsigned char)rbr4;
//		 }
//		 __HAL_UART_FLUSH_DRREGISTER (& huart4);
//  }

  
	
}

void Serial1_Write(char str){
	char ch[2];
	ch[1] = str;
	HAL_UART_Transmit(&huart1,(unsigned char*)ch,1, 2);
}

void Serial2_Write(char str){
	char ch[2];
	ch[1] = str;
	HAL_UART_Transmit(&huart2,(unsigned char*)ch,1, 2);
}

void Serial3_Write(char str){
	char ch[2];
	ch[1] = str;
	HAL_UART_Transmit(&huart3,(unsigned char*)ch,1, 2);
}

//void Serial4_Write(char str){
//	char ch[2];
//	ch[1] = str;
//	HAL_UART_Transmit(&huart4,(unsigned char*)ch,1, 2);
//}


void Serial1_WriteByte(char * str, int length){
	HAL_UART_Transmit(&huart1,(unsigned char*)str,length, 10);
}
void Serial2_WriteByte(char * str, int length){
	HAL_UART_Transmit(&huart2,(unsigned char*)str,length, 10);
}
void Serial3_WriteByte(char * str, int length){
	HAL_UART_Transmit(&huart3,(unsigned char*)str,length, 10);
}
//void Serial4_WriteByte(char * str, int length){
//	HAL_UART_Transmit(&huart4,(unsigned char*)str,length, 10);
//}



void Serial1_Print(char * str){
	HAL_UART_Transmit(&huart1,(unsigned char*)str,strlen(str), 20);
}

void Serial2_Print(char * str){
	HAL_UART_Transmit(&huart2,(unsigned char*)str,strlen(str), 20);
}

void Serial3_Print(char * str){
	HAL_UART_Transmit(&huart3,(unsigned char*)str,strlen(str), 20);
}

//void Serial4_Print(char * str){
//	HAL_UART_Transmit(&huart4,(unsigned char*)str,strlen(str), 20);
//}

void Serial1_Println(char * str){
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart1,(unsigned char*)str,strlen(str), 30);
	HAL_UART_Transmit(&huart1,(unsigned char*)newline,2, 10);
}

void Serial2_Println(char * str){
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart2,(unsigned char*)str,strlen(str), 20);
	HAL_UART_Transmit(&huart2,(unsigned char*)newline,2, 10);
}

void Serial3_Println(char * str){
	char newline[2] = "\r\n";
	HAL_UART_Transmit(&huart3,(unsigned char*)str,strlen(str), 20);
	HAL_UART_Transmit(&huart3,(unsigned char*)newline,2, 10);
}

//void Serial4_Println(char * str){
//	char newline[2] = "\r\n";
//	HAL_UART_Transmit(&huart4,(unsigned char*)str,strlen(str), 20);
//	HAL_UART_Transmit(&huart4,(unsigned char*)newline,2, 10);
//}


unsigned char kbhit_Uart(int uartCH)
{
   /* Read a byte from serial interface */
   //struct buf_st *p = &rbuf_uart[uartCH];
  
   if (rbuf_uart[uartCH].in == rbuf_uart[uartCH].out)  
     return (false);      /* Serial receive buffer is empty. */
    else
	 return(true);  
}

//****************************

int getbyteUart(int uartCH)  // Read character from Virtual Serail
{
   /* Read a byte from serial interface */
   //struct buf_st  *p = &rbuf_uart[uartCH];
  
   if (rbuf_uart[uartCH].in == rbuf_uart[uartCH].out) {
      /* Serial receive buffer is empty. */
	  return (-1);
   } 
   return ((unsigned char)rbuf_uart[uartCH].buf[rbuf_uart[uartCH].out++]);    
}

//***********************************

void ClearBufferUart(int uartCH)
{
  unsigned int i;
	
  for(i=0;i<=BUFSIZE;i++)
   {
		 rbuf_uart[uartCH].buf[i] = NULL;
//     if(getbyteUart(uartCH)== -1){
//			  rbuf_uart[uartCH].in = 0;
//			  rbuf_uart[uartCH].out = 0;
//	     break;
//		 }
   }	
   rbuf_uart[uartCH].in = 0;
   rbuf_uart[uartCH].out = 0;	 
}

//***********************************

unsigned int Serial_available(int uartCH){  
	int indexS;
   if(rbuf_uart[uartCH].in == rbuf_uart[uartCH].out){
		 //ClearBufferUart(uartCH);
     return (false);      /* Serial receive buffer is empty. */
	 }
   else{
		 indexS = rbuf_uart[uartCH].in - rbuf_uart[uartCH].out;
	   return(indexS); 
		}			
}
 
//************************************

char Serial_read(int uartCH){
	return getbyteUart(uartCH);
}

unsigned int Serial_readBytes(int uartCH, volatile unsigned char *Byte , int timeOut){	
		unsigned long currentMillis;
		unsigned long MillisCnt;
		unsigned long MillisTimeout;
		unsigned long Timeout;
		int Length = 0 , i;
		char byte;
//	  char sbuf[50];
		MillisTimeout = timeOut; 
    MillisCnt = HAL_GetTick();       //get systick timer millisecond (for STM32CUBE HAL)	
//		MillisCnt = millis();
    while(1){
			currentMillis = HAL_GetTick();				
			Timeout = (unsigned long)(currentMillis - MillisCnt);
			if(Timeout>= MillisTimeout){
//				sprintf(sbuf,"Timeout:%lu ms , len[%d]",Timeout,Length);	
//			  Serial1_Print(sbuf);
				//UART0_Printf("\r\n-> Serial2 timeout: %d ms\r\n",Timeout);
				//return Length;
				break;
				}
			if(Serial_available(uartCH) > 0){
				byte = Serial_read(uartCH);
				Byte[Length++] = byte;
				}					
      }
		ClearBufferUart(uartCH);
	  return Length;
}

unsigned int Serial_readByte_String(int uartCH, volatile unsigned char *Byte , int timeOut){	
		unsigned long currentMillis;
		unsigned long MillisCnt;
		unsigned long MillisTimeout;
		unsigned long Timeout;
		int Length = 0 , i;
		char byte;
		MillisTimeout = timeOut;  
		MillisCnt = HAL_GetTick();       //get systick timer millisecond (for STM32CUBE HAL)
	  //MillisCnt = millis();          //get systick timer millisecond
    while(1){
			currentMillis = HAL_GetTick();				
			Timeout = (unsigned long)(currentMillis - MillisCnt);
			if(Timeout>= MillisTimeout){
				Byte[Length++] = '\0';
				ClearBufferUart(uartCH);
				return Length;
				break;
				}
			if(Serial_available(uartCH) > 0){
				byte = Serial_read(uartCH);
				if((byte == '\r') || (byte == '\n')){
					Byte[Length++] = '\0';
					ClearBufferUart(uartCH);	  
					return Length;
				}
				else{
					Byte[Length++] = byte;
				}						
			}					
    }		
}