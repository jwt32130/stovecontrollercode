/*******************************************************************
** File        : SX1211driver.c                                   **
********************************************************************
**                                                                **
** Version     : V 1.0                                            **
**                                                                **
** Written by  : Chaouki ROUAISSIA                                **
**                                                                **
** Date        : 25-09-2006                                       **
**                                                                **
** Project     : API-1211                                         **
**                                                                **
********************************************************************
**                                                                **
**                                                                **
** Changes     : V 2.4 / CRo - 06-08-2007                         **
**               - No change                                      **
**               V 2.4.1 / CRo - 23-04-2008                       **
**               - Corrected size check error in SendRFFrame      **
**                                                                **
**                                                                **
********************************************************************
** Description : SX1211 transceiver drivers Implementation for the**
**               XE8000 family products !!! Packet mode !!!       **
*******************************************************************/

/*******************************************************************
** Include files                                                  **
*******************************************************************/
#include "SX1211Driver.h"

/*******************************************************************
** Global variables                                               **
*******************************************************************/
static   uint8_t RFState = RF_STOP;     // RF state machine
static   uint8_t *pRFFrame;             // Pointer to the RF frame
static   uint8_t RFFramePos;            // RF payload current position
static   uint8_t RFFrameSize;           // RF payload size
static  uint16_t ByteCounter = 0;       // RF frame byte counter
static   uint8_t PreMode = RF_STANDBY;  // Previous chip operating mode
static   uint8_t SyncSize = 4;          // Size of sync word
static   uint8_t SyncValue[4];       // Value of sync word
static  uint32_t RFFrameTimeOut = RF_FRAME_TIMEOUT(1600); // Reception counter value (full frame timeout generation)

uint8_t RegistersCfg[] = { // SX1211 configuration registers values
		DEF_MCPARAM1 | RF_MC1_STANDBY | RF_MC1_BAND_915H | RF_MC1_VCO_TRIM_00 | RF_MC1_RPS_SELECT_1,                    
		DEF_MCPARAM2 | RF_MC2_MODULATION_FSK | RF_MC2_DATA_MODE_PACKET | RF_MC2_OOK_THRESH_TYPE_PEAK | RF_MC2_GAIN_IF_00,                      
		DEF_FDEV | RF_FDEV_100,                          
		DEF_BITRATE | RF_BITRATE_25000,                       
		DEF_OOKFLOORTHRESH | RF_OOKFLOORTHRESH_VALUE,                  
		DEF_MCPARAM6 | RF_MC6_FIFO_SIZE_64 | RF_MC6_FIFO_THRESH_VALUE,                   
		DEF_R1 | 143,                            
		DEF_P1 | 114,                            
		DEF_S1 | 65,                            
		DEF_R2 | RF_R2_VALUE,                            
		DEF_P2 | RF_P2_VALUE,                            
		DEF_S2 | RF_S2_VALUE,                            
		DEF_PARAMP | RF_PARAMP_11,                         
		
		DEF_IRQPARAM1 | RF_IRQ0_RX_STDBY_PAYLOADREADY | RF_IRQ1_RX_STDBY_CRCOK | RF_IRQ1_TX_TXDONE,                     
		DEF_IRQPARAM2 | RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY | RF_IRQ2_PLL_LOCK_PIN_ON,                     
		DEF_RSSIIRQTHRESH | RF_RSSIIRQTHRESH_VALUE,                 
		
		DEF_RXPARAM1 | RF_RX1_PASSIVEFILT_378 | RF_RX1_FC_FOPLUS100,                      
		DEF_RXPARAM2 | RF_RX2_FO_100,                      
		DEF_RXPARAM3 | RF_RX3_POLYPFILT_OFF | RF_RX3_SYNC_SIZE_32 | RF_RX3_SYNC_TOL_0,                      
		DEF_RES19,                       
      //RSSI Value (Read only)               
		DEF_RXPARAM6 | RF_RX6_OOK_THRESH_DECSTEP_000 | RF_RX6_OOK_THRESH_DECPERIOD_000 | RF_RX6_OOK_THRESH_AVERAGING_00,                       
		
		DEF_SYNCBYTE1 | 0x69, // 1st byte of Sync word,                     
		DEF_SYNCBYTE2 | 0x81, // 2nd byte of Sync word,                     
		DEF_SYNCBYTE3 | 0x7E, // 3rd byte of Sync word,                     
		DEF_SYNCBYTE4 | 0x96, // 4th byte of Sync word,                     
		
		DEF_TXPARAM | RF_TX_FC_200 | RF_TX_POWER_PLUS10,                       
		
		DEF_OSCPARAM | RF_OSC_CLKOUT_ON | RF_OSC_CLKOUT_427,                     

		DEF_PKTPARAM1 | RF_PKT1_MANCHESTER_OFF | 64,                  
		DEF_NODEADRS  | RF_NODEADRS_VALUE,                 
		DEF_PKTPARAM3 | RF_PKT3_FORMAT_VARIABLE | RF_PKT3_PREAMBLE_SIZE_32 | RF_PKT3_WHITENING_OFF | RF_PKT3_CRC_ON | RF_PKT3_ADRSFILT_00,                    
		DEF_PKTPARAM4 | RF_PKT4_AUTOCLEAR_ON | RF_PKT4_FIFO_STANDBY_WRITE
   
};

/*******************************************************************
** Configuration functions                                        **
*******************************************************************/

/*******************************************************************
** InitRFChip : This routine initializes the RFChip registers     **
**              Using Pre Initialized variables                   **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void InitRFChip (void){
        uint16_t i;
    // Initializes SX1211
    // SPIInit();
    // set_bit(PORTO, (SCK + NSS_DATA + NSS_CONFIG + MOSI));
    // set_bit(PORTP, (SCK + NSS_DATA + NSS_CONFIG + MOSI));

    for(i = 0; (i + 1) <= REG_PKTPARAM4; i++){
        char tempmsg[20];
        sprintf((char*)tempmsg, "Send:0x%x\n\r", RegistersCfg[i]);
        SendUSB(tempmsg);
        if(i < REG_RSSIVALUE){
            WriteRegister(i, RegistersCfg[i]);
        }
        else{
            WriteRegister(i + 1, RegistersCfg[i]);
        }
        Wait(1000);
        uint8_t response = ReadRegister(i);
        sprintf((char*)tempmsg, "Recv:0x%x\n\r", response);
        
    }

    SyncSize = ((RegistersCfg[REG_RXPARAM3] >> 3) & 0x03) + 1;
    for(i = 0; i < SyncSize; i++){
        SyncValue[i] = RegistersCfg[REG_SYNCBYTE1 - 1 + i];
    }

    if(RegistersCfg[REG_BITRATE] == RF_BITRATE_1600){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(1600);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_2000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(2000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_2500){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(2500);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_5000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(5000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_8000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(8000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_10000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(10000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_20000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(20000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_25000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(25000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_40000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(40000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_50000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(50000);
    }
    else if(RegistersCfg[REG_BITRATE] == RF_BITRATE_100000){
        RFFrameTimeOut = RF_FRAME_TIMEOUT(100000);
    }
    else {
        RFFrameTimeOut = RF_FRAME_TIMEOUT(1600);
    }

    SetRFMode(RF_STANDBY);
}

/*******************************************************************
** SetRFMode : Sets the SX1211 operating mode                     **
********************************************************************
** In  : mode                                                     **
** Out : -                                                        **
*******************************************************************/
void SetRFMode(uint8_t mode){
    if(mode != PreMode){
        if(mode == RF_TRANSMITTER){
        
        		if (PreMode == RF_SLEEP){
	               WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
        		   Wait(TS_OS);        		
	               WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		   Wait(TS_FS);         		
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait(TS_TR);
        		}

        		else if (PreMode == RF_STANDBY){
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		   Wait(TS_FS);         		
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait(TS_TR);
        		}

        		else if (PreMode == RF_SYNTHESIZER){
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait(TS_TR);
        		}

        		else if (PreMode == RF_RECEIVER){
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_TRANSMITTER);
        		   Wait(TS_TR);
        		}		        
        		PreMode = RF_TRANSMITTER;
        }
        
        else if(mode == RF_RECEIVER){
        
            if (PreMode == RF_SLEEP){
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
        		   Wait(TS_OS);        		
	               WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		   Wait(TS_FS); 
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait(TS_RE);
        		}

        		else if (PreMode == RF_STANDBY){
	               WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		   Wait(TS_FS); 
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait(TS_RE);
        		}

        		else if (PreMode == RF_SYNTHESIZER){
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait(TS_RE);     		
        		}

        		else if (PreMode == RF_TRANSMITTER){	
        		   WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_RECEIVER);
        		   Wait(TS_RE);
        		}
        		PreMode = RF_RECEIVER;
        }
        
        else if(mode == RF_SYNTHESIZER){
        
            if (PreMode == RF_SLEEP){
	            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
        		   Wait(TS_OS);        		
	            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		   Wait(TS_FS); 
        		}

        		else if (PreMode == RF_STANDBY){
        	    WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		Wait(TS_FS); 
        		}

        		else {
        	    WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SYNTHESIZER);        		
        		}
            
            PreMode = RF_SYNTHESIZER;
        }
        
        else if(mode == RF_STANDBY){
        
            if (PreMode == RF_SLEEP){
	            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);        		
        		Wait(TS_OS);
        		}

        		else {
	            WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_STANDBY);       		
        		}
        		
        		PreMode = RF_STANDBY;
        }
        
        else {// mode == RF_SLEEP
        WriteRegister(REG_MCPARAM1, (RegistersCfg[REG_MCPARAM1] & 0x1F) | RF_SLEEP);
        PreMode = RF_SLEEP;
        }
    }
}

/*******************************************************************
** WriteRegister : Writes the register value at the given address **
**                  on the SX1211                                 **
********************************************************************
** In  : address, value                                           **
** Out : -                                                        **
*******************************************************************/
void WriteRegister(uint8_t address, uint8_t value){
    
    // SPIInit();
    address = (address << 1) & 0x3E ;
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    // SPINssData(1);
    // SPINssConfig(0);
    HAL_SPI_Transmit(&hspi1, address, sizeof(address), 10);
    HAL_SPI_Transmit(&hspi1, value, sizeof(value), 10);
    // SpiInOut(address);
    // SpiInOut(value);
    // SPINssConfig(1);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
}

/*******************************************************************
** ReadRegister : Reads the register value at the given address on**
**                the SX1211                                      **
********************************************************************
** In  : address                                                  **
** Out : value                                                    **
*******************************************************************/
uint8_t ReadRegister(uint8_t address){
    uint8_t value = 0;

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
    // SPIInit();
    // SPINssData(1);
    address = ((address << 1) & 0x7E) | 0x40;
    HAL_SPI_Transmit(&hspi1, address, sizeof(address), 10);


    HAL_SPI_Receive(&hspi1, &value, 1, 5);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
    
    // SPINssConfig(0);
    // SpiInOut(address);
    // value = SpiInOut(0);
    // SPINssConfig(1);

    return value;
}

void Wait(uint16_t us) {
    __HAL_TIM_DISABLE(&htim6);
  __HAL_TIM_SET_COUNTER(&htim6, 0);
  __HAL_TIM_SET_AUTORELOAD(&htim6, us);
  __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE(&htim6);

  while (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE) == RESET)
  {
  }
}
/*******************************************************************
** Communication functions                                        **
*******************************************************************/
#if 0
/*******************************************************************
** SendRfFrame : Sends a RF frame                                 **
********************************************************************
** In  : *buffer, size                                            **
** Out : *pReturnCode                                             **
*******************************************************************/
void SendRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode){
    if((size+1) > (((RegistersCfg[REG_MCPARAM6])>>6)+1)*16){  // If (size + length byte) > FIFO size
        RFState |= RF_STOP;
        *pReturnCode = ERROR;
        return;
    }

    RFState |= RF_BUSY;
    RFState &= ~RF_STOP;
    RFFrameSize = size;
    pRFFrame = buffer;
    
    SetRFMode(RF_STANDBY);
    WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_WRITE);
    
    SendByte(RFFrameSize);
    for(ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize;){
            SendByte(pRFFrame[RFFramePos++]); 
            ByteCounter++; 
    }

    SetRFMode(RF_TRANSMITTER); // Cf RF_IRQ0_TX_FIFOEMPTY_START_FIFONOTEMPTY 
    
    do{
    }while(!(RegPAIn & IRQ_1)); // Wait for TX done
    Wait(1000); // Wait for last bit to be sent (worst case bitrate)

    //SetRFMode(RF_SLEEP);
    SetRFMode(RF_STANDBY);


    RFState |= RF_STOP;
    RFState &= ~RF_TX_DONE;
    *pReturnCode = OK;
} // void SendRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)

/*******************************************************************
** ReceiveRfFrame : Receives a RF frame                           **
********************************************************************
** In  : -                                                        **
** Out : *buffer, size, *pReturnCode                              **
*******************************************************************/
void ReceiveRfFrame(uint8_t *buffer, uint8_t *size, uint8_t *pReturnCode){
    
       uint8_t TempRFState;
       
       *pReturnCode = RX_RUNNING;

       TempRFState = RFState; 

    if(TempRFState & RF_STOP){
        pRFFrame = buffer;
        RFFramePos = 0;
        RFFrameSize = 2;
                
        RegIrqEnMid |= 0x02; // Enables Port A pin 1 interrupt IRQ_1 (CRCOK)

        SetRFMode(RF_RECEIVER);
        EnableTimeOut(true);
        RFState |= RF_BUSY;
        RFState &= ~RF_STOP;
        RFState &= ~RF_TIMEOUT;
        return;
    }
    else if(TempRFState & RF_RX_DONE){

        SetRFMode(RF_STANDBY);        
        WriteRegister(REG_PKTPARAM4, (RegistersCfg[REG_PKTPARAM4-1] & 0xBF) | RF_PKT4_FIFO_STANDBY_READ);
            
        RFFrameSize = ReceiveByte();
        for(ByteCounter = 0, RFFramePos = 0; ByteCounter < RFFrameSize; ){
        		pRFFrame[RFFramePos++] = ReceiveByte();
            ByteCounter++; 
        }
        //SetRFMode(RF_SLEEP);
        
        *size = RFFrameSize;
        *pReturnCode = OK;
        RFState |= RF_STOP;
        EnableTimeOut(false);
        RFState &= ~RF_RX_DONE;
        RegIrqEnMid &= ~0x02; // Disables Port A pin 1 interrupt IRQ_1 (CRCOK)
        SPINssData(1);
        return;
    }
    else if(TempRFState & RF_ERROR){
        RFState |= RF_STOP;
        RFState &= ~RF_ERROR;
        *pReturnCode = ERROR;
        RegIrqEnMid &= ~0x02; // Disables Port A pin 1 interrupt IRQ_1 (CRCOK)
        SPINssData(1);
        return;
    }
    else if(TempRFState & RF_TIMEOUT){
        RFState |= RF_STOP;
        RFState &= ~RF_TIMEOUT;
        EnableTimeOut(false);
        *pReturnCode = RX_TIMEOUT;
        RegIrqEnMid &= ~0x02; // Disables Port A pin 1 interrupt IRQ_1 (CRCOK)
        SPINssData(1);
        return;
    }
} // void ReceiveRfFrame(uint8_t *buffer, uint8_t size, uint8_t *pReturnCode)


/*******************************************************************
** Transceiver specific functions                                 **
*******************************************************************/

/*******************************************************************
** ReadRssi : Reads the Rssi value from the SX1211                **
********************************************************************
** In  : -                                                        **
** Out : value                                                    **
*******************************************************************/
uint16_t ReadRssi(void){
	uint16_t value;
	value = ReadRegister(REG_RSSIVALUE);  // Reads the RSSI result
	return value;
}

/*******************************************************************
** Utility functions                                              **
*******************************************************************/

/*******************************************************************
** Wait : This routine uses the counter A&B to create a delay     **
**        using the RC ck source                                  **
********************************************************************
** In   : cntVal                                                  **
** Out  : -                                                       **
*******************************************************************/
void Wait(uint16_t cntVal){
    RegCntOn &= 0xFC;                              // Disables counter A&B
    RegEvnEn &= 0x7F;                              // Disables events from the counter A&B
    RegEvn = 0x80;                                 // Clears the event from the CntA on the event register
    RegCntCtrlCk =  (RegCntCtrlCk & 0xFC) | 0x01;  // Selects RC frequency as clock source for counter A&B
    RegCntConfig1 |= 0x34;                         // A&B counters count up, counter A&B are in cascade mode
    RegCntA       = (uint8_t)(cntVal);                 // LSB of cntVal
    RegCntB       = (uint8_t)(cntVal >> 8);            // MSB of cntVal
    RegEvnEn      |= 0x80;                         // Enables events from CntA
    RegEvn        |= 0x80;                         // Clears the event from the CntA on the event register
    asm("clrb %stat, #0");                         // Clears the event on the CoolRISC status register
    RegCntOn      |= 0x03;                         // Enables counter A&B
    do{
        asm("halt");
    }while ((RegEvn & 0x80) == 0x00);              // Waits the event from counter A
    RegCntOn      &= 0xFE;                         // Disables counter A
    RegEvnEn      &= 0x7F;                         // Disables events from the counter A
    RegEvn        |= 0x80;                         // Clears the event from the CntA on the event register
    asm("clrb %stat, #0");                         // Clears the event on the CoolRISC status register
} // void Wait(uint16_t cntVal)

/*******************************************************************
** EnableTimeOut : Enables/Disables the RF frame timeout          **
********************************************************************
** In  : enable                                                   **
** Out : -                                                        **
*******************************************************************/
void EnableTimeOut(uint8_t enable){
    RegCntCtrlCk = (RegCntCtrlCk & 0xFC) | 0x03;        // Selects 128 Hz frequency as clock source for counter A&B
    RegCntConfig1 |=  0x34;                             // A&B counters count up, counter A&B  are in cascade mode

    RegCntA = (uint8_t)RFFrameTimeOut;                      // LSB of RFFrameTimeOut
    RegCntB = (uint8_t)(RFFrameTimeOut >> 8);               // MSB of RFFrameTimeOut

    if(enable){
        RegIrqEnHig |= 0x10;                            // Enables IRQ for the counter A&B
        RegCntOn |= 0x03;                               // Enables counter A&B
    }
    else{
        RegIrqEnHig &= ~0x10;                           // Disables IRQ for the counter A&B
        RegCntOn &= ~0x03;                              // Disables counter A&B
    }
} // void EnableTimeOut(uint8_t enable)

/*******************************************************************
** InvertByte : Inverts a byte. MSB -> LSB, LSB -> MSB            **
********************************************************************
** In  : b                                                        **
** Out : b                                                        **
*******************************************************************/
uint8_t InvertByte(uint8_t b){
    asm("   move %r0, #0x08");
    asm("LoopInvertByte:");
    asm("   shl  %r3");
    asm("   shrc %r2");
    asm("   dec  %r0");
    asm("   jzc  LoopInvertByte");
} // uint8_t InvertByte(uint8_t b)

/*******************************************************************
** SpiInOut : Sends and receives a byte from the SPI bus          **
********************************************************************
** In  : outputByte                                               **
** Out : inputByte                                                **
*******************************************************************/
uint8_t SpiInOut (uint8_t outputByte){
    uint8_t bitCounter;
    uint8_t inputByte = 0;

    SPIClock(0);
    for(bitCounter = 0x80; bitCounter != 0x00; bitCounter >>= 1){
        if (outputByte & bitCounter){
            SPIMosi(1);
        }
        else{
            SPIMosi(0);
        }
        SPIClock(1);
        if (SPIMisoTest()){
            inputByte |= bitCounter;
        }
        SPIClock(0);
    }  // for(BitCounter = 0x80; BitCounter != 0x00; BitCounter >>= 1)
    SPIMosi(0);

    return inputByte;
} // uint8_t SpiInOut (uint8_t outputByte)

/*******************************************************************
** SX1211 Buffered interrupt handlers                             **
*******************************************************************/

/*******************************************************************
** Handle_Irq_Pa1 : Handles the interruption from the Pin 1 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void Handle_Irq_Pa1 (void){ // IRQ_1 = CRC_OK
 
     RFState |= RF_RX_DONE;
     RFState &= ~RF_BUSY;
        
} //End Handle_Irq_Pa1

/*******************************************************************
** Handle_Irq_Pa2 : Handles the interruption from the Pin 2 of    **
**                  Port A                                        **
********************************************************************
** In  : -                                                        **
** Out : -                                                        **
*******************************************************************/
void Handle_Irq_Pa2 (void){ // IRQ_0 


} //End Handle_Irq_Pa2

/*******************************************************************
** Handle_Irq_CntA : Handles the interruption from the Counter A  **
********************************************************************
** In              : -                                            **
** Out             : -                                            **
*******************************************************************/
void Handle_Irq_CntA (void){
    RFState |= RF_TIMEOUT;
    RFState &= ~RF_BUSY;
} //End Handle_Irq_CntA

#endif