extern "C"
{
	#include <project.h>
	int main();
	volatile uint32 time;
}
#include <stdio.h>
#include "RF24.h"

RF24 nrf(0,0);

static void SysTick_isr()
{
	time++;
}

static uint16 num_responses=0;
static uint8 nd_responses[256][8];
static void nrf_isr()
{
	while(nrf.available())
	{
		uint8 n=nrf.getDynamicPayloadSize();
		uint8 packet[32];
		
		if(num_responses>0xff) num_responses=0xff;
		
		if(n!=8) nrf.read(packet,n);
		else nrf.read(nd_responses[num_responses++],8);
	}
}

int main()
{	
	VDAC_Start();
	Opamp_Start();
	
    CyIntSetSysVector(SysTick_IRQn+16,SysTick_isr);
	SysTick_Config(BCLK__BUS_CLK__HZ/1000); //1ms SysTick
	
	CyGlobalIntEnable;
	USB_Start(0,USB_DWR_VDDD_OPERATION);
	while(!USB_GetConfiguration());
	
	CyDelay(100);
	nrf.begin();
	nrf.setAddressWidth(3);
	nrf.setRetries(1,15);
	nrf.setChannel(0);
	nrf.enableAckPayload();
	nrf.enableDynamicPayloads();
	nrf.enableDynamicAck();
	nrf.setDataRate(RF24_2MBPS);
	nrf.setCRCLength(RF24_CRC_16);
	nrf.maskIRQ(true,true,false);
	
	reset_ClearPending();
	reset_StartEx(CySoftwareReset);
	
	nrf_int_StartEx(nrf_isr);
	
	USB_CDC_Init();
	
	uint32 timeout=1000; //timeout for blocking write
	
	for(;;) if(USB_DataIsReady())
	{	
		uint8 buffer[64];
		uint32 l=USB_GetAll(buffer);
		
		if(!l) continue;
		
		LED_Write(1);
		
		switch(buffer[0])
		{
			case 0: //normal packet transfer
				nrf_int_Disable();
				if(l!=1&&nrf.write(buffer+1,l-1))
				{
					if(nrf.available())
					{
						buffer[0]=nrf.getDynamicPayloadSize(); //length of ACK payload
						if(buffer[0]) nrf.read(buffer+1,buffer[0]); //read ACK payload back into buffer
					}
					else buffer[0]=0; //no error; zero length response
				}
				else buffer[0]=0x80|snprintf((char*)buffer+1,63,"%s","ACK not reveived from nrf");
				nrf_int_Enable();
				break;
			
			case 1: //set TX address
				//TODO check address width
				nrf.openWritingPipe(buffer+1);
				buffer[0]=0; //no error
				break;
			
			case 2: //set RX address
				//TODO check address width
				//TODO allow opening multiple pipes
				nrf.openReadingPipe(1,buffer+1);
				nrf.setAutoAck(1,false);
				buffer[0]=0;
				break;
				
			case 3: //send multicast packet
				nrf.write(buffer+1,l-1,true);
				buffer[0]=0;
				break;
				
			case 0xb0: //neighbour discovery
			case 0xb1:
				num_responses=0;
				nrf.write(buffer,l,true);
				nrf.startListening();
				CyDelay(15); //longer than largest possible delay of 255*50us
				nrf.stopListening();
				
				buffer[0]=2;
				*(uint16*)(buffer+1)=num_responses;
				for(uint32 i=0;i<num_responses;i++)
				{
					while(!USB_CDCIsReady());
    				USB_PutData(buffer,(buffer[0]&0x3f)+1);
					
					buffer[0]=8;
					memcpy(buffer+1,nd_responses[i],8);
				}
				break;
			
			//TODO separate command using nrf.writeBlocking instead of nrf.write (can specify long timeout and pipelines packets in FIFO - good for large data transfer such as flashing)
			//TODO extra command to set TX address and send payload in single packet
			//TODO add RX functionality
			//TODO configure address width, CRC, dynamic payloads, data rate, channel, retries, timeout
			//TODO standby/powerdown modes (not that useful since always USB powered)
				
			default:
				buffer[0]=0x80|snprintf((char*)buffer+1,63,"%s","Command from USB unknown");
				
		}
		
		while(!USB_CDCIsReady());
    	USB_PutData(buffer,(buffer[0]&0x3f)+1); //send response back over USB
		
		LED_Write(0);
	}
}