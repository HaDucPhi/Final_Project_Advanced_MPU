#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "Systime.h"
#define BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 30
// -------- TX ----------
volatile char TX_Buffer[TX_BUFFER_SIZE];
volatile int TX_Head = 0;
volatile int TX_Tail = 0;

// -------- RX ----------
volatile char RX_Buffer[RX_BUFFER_SIZE];
volatile int RX_Head = 0;
volatile int RX_Tail = 0;
volatile int RX_address;
volatile int RX_num;
volatile bool RX_Available = false;
// -------- Chuoi nhan ve ----------
char S[BUFFER_SIZE];
char S_sum[BUFFER_SIZE];
int S_index = 0;
char c;
volatile int next;
int k = 0;
char msg[BUFFER_SIZE];
char msgo[BUFFER_SIZE];
int i;
char CtrlZ[] = {0x1A, 0x00};
bool Stanby,Ready,Send_Done;
char Cmd;
//-------------------------------------------Khai bao ham--------------------------------------------------------
void A7670C_Start(void);
void A7670C_Send_Msg(void);
void A7670C_Message(void);
void Handle_Message(void);
void SIM_Init(void);
void Send_Message(void);
//-------------------------------------------Xu ly truyen nhan bang FIFO-----------------------------------------
void USART1_Init(void){
    RCC->APB2ENR |= (1 << 14);// Clock from ABP2 to USART1
    USART1->BRR = 625;// Baud rate 115200 bps
		USART1->CR1 |= (1 << 3) | (1 << 2); // TE + RE
		USART1->CR1 |= (1 << 13);           // UE
		USART1->CR3 |= (1 << 6)|(1 << 7); // Cho phep DMA truyen va nhan.
		USART1->CR1 |= (1 << 4);            // IDLEIE
    NVIC_EnableIRQ(USART1_IRQn);// Enable NVIC
}
void DMA1_Channel4_USART1_Config(void){
		RCC->AHBENR |= (1<<0); // Kich hoat DMA
		DMA1_Channel4->CCR  = 0; // Xoa cau hinh cu
		DMA1_Channel4->CPAR = (uint32_t)&USART1->DR; // Dia chi ngoai vi den thanh ghi du lieu USART
		DMA1_Channel4->CCR |= (1 << 12)|(1 << 7)| ( 0 << 6) | ( 1 << 4); // Priority = Medium, MSIZE = PSIZE = 8, MINC = 1, PINC = 0M, DIR = 1 
		DMA1_Channel4->CCR |= (1 << 1)|(1 << 2); // Cho phep ngat Half transfer va Transfer complete
		DMA1_Channel4->CCR |= (1 << 1); // Enable Transfer Complete Interrupt
	  NVIC_EnableIRQ(DMA1_Channel4_IRQn); // Enable NVIC cho DMA1 Channel4
	}
void DMA1_Channel5_USART1_Config(void){
		RCC->AHBENR |= (1<<0); // Kich hoat DMA
		DMA1_Channel5->CCR  = 0; // Xoa cau hinh cu
		DMA1_Channel5->CPAR = (uint32_t)&USART1->DR; // Dia chi ngoai vi den thanh ghi du lieu USART
		DMA1_Channel5->CMAR = (uint32_t)&RX_Buffer; // Dia chi bo nho nhan du lieu tu thanh ghi DR
		DMA1_Channel5->CNDTR = RX_BUFFER_SIZE; // So lan truyen du lieu DMA
		DMA1_Channel5->CCR |= (1 << 13)|(1 << 12)|(1 << 7)|(1<<5); // Priority = Very High, MSIZE = PSIZE = 8, MINC = 1, PINC = 0,CIRC = 1, DIR = 0
		DMA1_Channel5->CCR |= (1 << 1)|(1 << 2); // Cho phep ngat Half transfer va Transfer complete
		DMA1_Channel5->CCR |= (1 << 0); // Enable DMA1_Channel5
	  NVIC_SetPriority(DMA1_Channel5_IRQn, 1); //  Uu tien muc 1
		NVIC_EnableIRQ(DMA1_Channel5_IRQn); // Enable NVIC cho DMA1 Channel5
	}

void USART1_DMA_Send(const char* data) {
	  uint16_t len = strlen(data); // Chieu dai cua du lieu
    DMA1_Channel4->CCR &= ~( 1<< 0); // Tam thoi tat cau hinh DMA1_Channel4
	  DMA1->IFCR = (1 << 12); //  Xoa co trang thai
		DMA1_Channel4->CMAR = (uint32_t)data; // Truyen dia chi du lieu vao CMAR
    DMA1_Channel4->CNDTR = len; // Truyen do dai du lieu can gui
    DMA1_Channel4->CCR |= (1 << 0); // Enable DMA1_Channel4
}
void DMA1_Channel4_IRQHandler(void) {
    if (DMA1->ISR & (1 << 13)) { // Kiem tra Transfer Complete
        DMA1->IFCR |= (1 << 13); // Xoa co interrupt TC		
			  DMA1_Channel4->CCR &= ~( 1<< 0); // Tam thoi tat cau hinh DMA1_Channel4
    }
}
// -------- IRQ Handler ----------
void USART1_IRQHandler(void) {
	if (USART1->SR & (1 << 4)) {  // Kiem tra co IDLE
        volatile uint32_t tmp;
				tmp = USART1->SR;
				tmp = USART1->DR;
				(void)tmp;
        // Dung nhan du lieu
        DMA1_Channel5->CCR &= ~(1 << 0);
        // So byte da nhan
        RX_Head = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;
				// Gan du lieu nhan vao RX_Buffer
				if(RX_Head >= RX_Tail ){		
						RX_num = RX_Head - RX_Tail;
					for( i=0; i<RX_num; i++) {
            S[i] = RX_Buffer[RX_Tail];
						RX_Tail = ( RX_Tail + 1)%RX_BUFFER_SIZE;
					}
						strcat(S_sum, S);
						memcpy(msg, S_sum, sizeof(S_sum));
				    	memset(S, 0, sizeof(S));
						memset(S_sum, 0, sizeof(S_sum));
				}
				else if( RX_Head < RX_Tail){
						RX_num = RX_Head + RX_BUFFER_SIZE - RX_Tail;
					for( i=0; i<RX_num; i++) {
            S[i] = RX_Buffer[RX_Tail];
						RX_Tail = ( RX_Tail + 1)%RX_BUFFER_SIZE;
					}
						strcat(S_sum, S);
						memcpy(msg, S_sum, sizeof(S_sum));
				    memset(S, 0, sizeof(S));	
						memset(S_sum, 0, sizeof(S_sum));					
				}
        // Reset DMA de nhan tiep
        DMA1_Channel5->CCR |= (1 << 0);
				RX_Available = true;	
								c++;
    }
}
//-------- IRQ Handler ----------
void DMA1_Channel5_IRQHandler(void)
{
    // Half-transfer interrupt
    if (DMA1->ISR & (1 << 16)) {
        DMA1->IFCR |= (1 << 16);// Clear flag
				RX_Head = RX_BUFFER_SIZE - DMA1_Channel5->CNDTR;
				c++;
				if(RX_Head > RX_Tail ){		
						RX_num = RX_Head - RX_Tail;
					for( i=0; i<RX_num; i++) {
            S[i] = RX_Buffer[RX_Tail];
						RX_Tail = ( RX_Tail + 1)%RX_BUFFER_SIZE;
					}
						strcat(S_sum, S);
				    memset(S, 0, sizeof(S));
				}
				else if( RX_Head < RX_Tail){
						RX_num = RX_Head + RX_BUFFER_SIZE - RX_Tail;
					for( i=0; i<RX_num; i++) {
            S[i] = RX_Buffer[RX_Tail];
						RX_Tail = ( RX_Tail + 1)%RX_BUFFER_SIZE;
					}
						strcat(S_sum, S);
				    memset(S, 0, sizeof(S));					
				}					
    }
    // Transfer complete interrupt
    if (DMA1->ISR & (1 << 17)) {
        DMA1->IFCR |= (1 << 17);// Clear flag
				RX_Head = 30;			
				if(RX_Head > RX_Tail ){		
						RX_num = RX_Head - RX_Tail;
					for( i=0; i<RX_num; i++) {
            S[i] = RX_Buffer[RX_Tail];
						RX_Tail = ( RX_Tail + 1)%RX_BUFFER_SIZE;
					}
						strcat(S_sum, S);
				    memset(S, 0, sizeof(S));
				}				
    }
}
// -------- Main ----------
int main(void) {
    // -------- Cau hình RCC và GPIO ----------
    RCC->APB2ENR |= (1 << 2);   // GPIOA
    // PA9 TX - Alternate function push-pull
    GPIOA->CRH &= ~(0xF << ((9-8)*4));
    GPIOA->CRH |=  (0xB << ((9-8)*4));
    // PA10 RX - Input floating
    GPIOA->CRH &= ~(0xF << ((10-8)*4));
    GPIOA->CRH |=  (0x4 << ((10-8)*4));
    SysTick_Init();
	USART1_Init();
	DMA1_Channel4_USART1_Config();
	DMA1_Channel5_USART1_Config();
	A7670C_Send_Msg();

    // -------- Main loop ----------
    while(1) {
		Handle_Message();
		A7670C_Message();
}
void SIM_Init(void){
	Cmd = 'A';
	Start_SIM = true;
	Send_message = false;}
void Send_Message(void){
	Cmd = 'A';
	Start_SIM = false;
	Send_message = true;}
//-------------------------------------------------Ham xu ly nhan du lieu ----------------------------------------------------------
void Handle_Message(void){
    if(RX_Available){
        memcpy(msgo, msg, sizeof(msg));
		memset(msg, 0, sizeof(msg));
        RX_Available = false;
		// Neu  Start SIM = 1 thi kiem tra thong diep luc cau hinh.
        if(Start_SIM){
            switch(Cmd){
                case 'A':
                    // Nhay sang State B neu thong diep nhan co OK.
                    if (strstr(msgo, "OK") != NULL) {
                        Cmd = 'B';
                        Standby = 0;
                    }
                    break;
                case 'B':
                    // Nhay sang State C neu thong diep nhan co "+CPIN: READY"
                    if (strstr(msgo, "+CPIN: READY") != NULL) {
                        Cmd = 'C';
                        Standby = 0;
                    }
                    break;
                case 'C':
                    // Nhay sang State G ket thuc neu nhan OK.			
                    if (strstr(msgo, "OK") != NULL) {
                        Cmd = 'G';
                        Standby = 0;
                    }
                    break;
                default:
                    break;
            }
		// Neu  Send message = 1 thi kiem tra thong diep luc dang gui tin nhan
		if (Send_message) {
		    switch (Cmd) {
		        case 'D':
		            if (strstr(msgo, ">") != NULL) {
		                Cmd = 'E';
		                Standby = 0;
		            }
		            break;
		
		        case 'E':
		            if (strstr(msgo, "OK") != NULL) {
		                Cmd = 'F';
		                Standby = 1;
		            }
		            break;
		
		        default:
		            break;
		    }
		}
        }
    }
}

//---------------------------------------Ham chua cac tin nhan co the gui di -------------------------------------------------------
void A7670C_Message(void)
{
    switch (Cmd) {
        case 'A':
            if (!Standby) {
                USART1_DMA_Send("AT\r\n");
                Standby = 1;
            }
            break;
        case 'B':
            if (!Standby) {
                USART1_DMA_Send("AT+CPIN?\r\n");
                Standby = 1;
            }
            break;
        case 'C':
            if (!Standby) {
                USART1_DMA_Send("AT+CMGF=1\r\n");
                Standby = 1;
            }
            break;
        case 'D':
            if (!Standby) {
                USART1_DMA_Send("AT+CMGS=\"0765378801\"\r\n");
                Standby = 1;
            }
            break;
        case 'E':
            if (!Standby) {
                USART1_DMA_Send("Chao thay va cac ban <3 \r\n");
                Delay_ms(10);
                USART1_DMA_Send(CtrlZ);  // Ctrl+Z
                Standby = 1;
            }
            break;
        case 'F':
            if (!Standby) {
                Send_Done = 1;
                Standby = 1;
            }
            break;
        case 'G':
            if (!Standby) {
                Ready = 1;
                Standby = 1;
            }
            break;

        default:
            break;
    }
}
