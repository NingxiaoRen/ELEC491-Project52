#define LED      2
#define RXTX     15
#define REG_DATA 16
#define CD_PD    17
#define SS_CTRL  9
#define SIZE     40

  /******************************************************************************
     CD_PD   REG_DATA   RXTX            MODE                DIRECTION
                HIGH    HIGH     Control regiter read   MOSI <-- RXD/OUTPUT
                HIGH     LOW     Control regiter write  MISO --> TXD/INPUT
                LOW     HIGH          Data read         MOSI <-- RXD/OUTPUT
                LOW      LOW          Data write        MISO --> TXD/INPUT
   CD_PD High: Not transfer;  LOW: Transfer
  *******************************************************************************/
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;
volatile uint8_t temp1 = 0, temp2 = 0, cc_byte = 0, check_pointer = 0;
volatile uint8_t spi_buffer[SIZE], check_buffer[3];
volatile uint8_t i = 0;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  SPI_SlaveInit();
  digitalWrite(SS_CTRL, LOW);
  PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan (PORTB)
  PCMSK0 |= (1 << PCINT0);   // set PCINT0 to trigger an interrupt on state change (pin pb1 (SW1 button))
  sei();    // turn on interrupts
}

void loop() {
  digitalWrite(REG_DATA, LOW);
  digitalWrite(RXTX, HIGH);
  if(pos_write != pos_read){
    // Combine two bytes to one byte received data 
    if(cc_byte == 0)
    {
      // Shifting MSB four bits 
      temp1 = spi_buffer[pos_read];
      spi_buffer[pos_read]=0;
      while((temp1&0xf0)>0x30) temp1 = temp1>>1;
      if ((temp1&0xf0) == 0x30) {cc_byte = 1;
      }else{ cc_byte = 0;}
      temp1 = temp1<<4;
      //cc_byte = 1;
    }
    else
    {
      // Shifting LSB four bits 
      temp2 = spi_buffer[pos_read];
      spi_buffer[pos_read]=0x00;
      while((temp2&0xf0)>0x30) temp2=temp2>>1;
      temp2 &= 0x0f;
      temp2 |= temp1; 
      cc_byte = 0;
      // Check if the received data is correct. 
      check_buffer[check_pointer] = temp2;
      check_pointer++;
      if(check_pointer >2)
      {
        check_pointer = 0;
        if((check_buffer[0] == check_buffer[1] ) && (check_buffer[1] == check_buffer[2]))
        {
          temp2 = check_buffer[0];
          command_library(temp2);    
         }
         else
          temp2 = 0xE;
        Serial.println(temp2,HEX);
      }
      temp1 = 0;
      temp2 = 0;
    }
     pos_read++;
     if (pos_read > SIZE - 1) pos_read = 0;
  }
}

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for SPI transfer complete
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (SPI_STC_vect){
  //byte c = SPDR;
  spi_buffer[pos_write] = SPDR;
  //Serial.println(spi_buffer[pos_write],HEX);
  pos_write++;
  if(pos_write>SIZE-1) pos_write = 0;
} 

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for PCINT1/PB1/D9 changes (SS/PCINT2/PB2)
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (PCINT0_vect){
  //To stop the interrupt we need to set SS/PB2 ?? PB1 ??  Low, which means that there is nothing in the line
  //To continue we need to set the SS high
  if(PINB & (1<<PB0)){//rising
    //Serial.println("CD_PD Rising");
    digitalWrite(SS_CTRL,HIGH);
  }else{//falling
    //Serial.println("CD_PD Falling");
    digitalWrite(SS_CTRL,LOW);
  }
}

/*******************************************************************************
* Function Name  : SPI_SlaveInit
* Description    : Configure Arduino pins for slave mode
* Input          : None
* Output         : None
* Return         : SPDR
*******************************************************************************/
void SPI_SlaveInit(void){
  /* Configure Arduino as a slave, Set MISO output, all other input */
  pinMode(SCK,INPUT);   // PB5 --- ST7540_CLR/T
  pinMode(SS,INPUT);    // PB2 --- GND
  pinMode(MOSI,INPUT);  // PB3 --- ST7540_RXD/OUTPUT
  pinMode(MISO,OUTPUT); // PB4 --- ST7540_TXD/INPUT
  pinMode(SS_CTRL,OUTPUT);
  /* Configure pins for ST7540 */
  pinMode(RXTX, OUTPUT);
  pinMode(REG_DATA, OUTPUT);
  pinMode(CD_PD, INPUT);
  /* Enable SPI */
  SPCR = (1<<SPE)|(0<<MSTR);
  Modem_CtrlWrite();
  Modem_CtrlRead();
  SPCR |= (1<<SPIE);
}

/*******************************************************************************
* Function Name  : SPI_SlaveReceive
* Description    : Arduino SPI receives data from the buffer ( Modem )
* Input          : None
* Output         : None
* Return         : SPDR
*******************************************************************************/
uint8_t SPI_SlaveReceive(void){
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}

/*******************************************************************************
* Function Name  : SPI_SlaveTransmit
* Description    : Arduino SPI transmits data as a slave device.
* Input          : transmitted data
* Output         : None
* Return         : None
*******************************************************************************/
void SPI_SlaveTransmit(uint8_t cData){
  /* Start transmission */
  SPDR = cData;
  /* Wait for transmission complete*/ 
  while(!(SPSR & (1<<SPIF)));
}

/*******************************************************************************
* Function Name  : DataCorrection_Transmit
* Description    : Divide one byte data into two bytes by adding a header to each
				   four bits of the data. the header will also serve as the address
				   of each device. we can use the header for bit correction and for 
				   ack protocols between master and slaves.
* Input          : transmitted data
* Output         : None
* Return         : None
*******************************************************************************/
void DataCorrection_Transmit (uint8_t temp){
  uint8_t temp1 = 0, temp2 = 0, i = 0;
  /* Configure the Most Siganificant Four bits of the Byte as: 0001 MSFB */
  temp1 = (temp >> 4) | 0x10;
  /* Configure the Least Siganificant Four bits of the Byte as: 0001 LSFB */
  temp2 = (temp & 0x0f) | 0x10;
  /* Transmit each original byte 3 times for failure correction*/
  digitalWrite(REG_DATA, LOW);
  digitalWrite(RXTX, LOW);
  for (i=0; i<3; i++) {
      SPI_SlaveTransmit(temp1);
      SPI_SlaveTransmit(temp2); 
  }  
  digitalWrite(RXTX,HIGH);
}

/*******************************************************************************
* Function Name  : Modem_CtrlRead
* Description    : Request control register data from ST7540
* Input          : None
* Output         : None
*******************************************************************************/
void Modem_CtrlRead()
{
  /* Drive REG_DATA and RXTX high to request control register data from ST7540 */
  digitalWrite(REG_DATA, HIGH);
  digitalWrite(RXTX, HIGH);
  uint8_t temp1,temp2,temp3, temp4,temp5, temp6, temp7, temp8;
  //uint64_t correction1 = 0;
  //uint64_t correction2 = 0;
  digitalWrite(SS_CTRL,LOW);
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  temp4 = SPI_SlaveReceive();
  temp5 = SPI_SlaveReceive();
  temp6 = SPI_SlaveReceive();
  temp7 = SPI_SlaveReceive();
  //temp8 = SPI_SlaveReceive();
  Serial.println("Re 1");
  Serial.println(temp1,BIN);
  Serial.println(temp2,BIN);
  Serial.println(temp3,BIN);
  Serial.println(temp4,BIN);
  Serial.println(temp5,BIN);
  Serial.println(temp6,BIN);
  Serial.println(temp7,BIN);
  //correction1 =  ((uint64_t)temp1<<56) | ((uint64_t)temp2<<48) | ((uint64_t)temp3<<40) | ((uint64_t)temp4) <<32 | ((uint64_t)temp5<<24) | ((uint64_t)temp6<<16) | ((uint64_t)temp7<<8);// | ((uint64_t)temp8);
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  temp4 = SPI_SlaveReceive();
  temp5 = SPI_SlaveReceive();
  temp6 = SPI_SlaveReceive();
  temp7 = SPI_SlaveReceive();
  Serial.println("Re 2");
    Serial.println(temp1,BIN);
  Serial.println(temp2,BIN);
  Serial.println(temp3,BIN);
  Serial.println(temp4,BIN);
  Serial.println(temp5,BIN);
  Serial.println(temp6,BIN);
  Serial.println(temp7,BIN);
  //temp8 = SPI_SlaveReceive();
  //correction2 =  ((uint64_t)temp1<<56) | ((uint64_t)temp2<<48) | ((uint64_t)temp3<<40) | ((uint64_t)temp4) <<32 | ((uint64_t)temp5<<24) | ((uint64_t)temp6<<16) | ((uint64_t)temp7<<8);// | ((uint64_t)temp8);
  //while(correction1 != correction2)
  digitalWrite(SS_CTRL,HIGH);
  /*Serial.println("C 1");
  Serial.println(correction1,BIN);
  Serial.println("C 2");
  Serial.println(correction2,BIN);
  */
}
  /*--------------------------------------------------------------
    Registers configuration for ST7540
    --------------------------------------------------------------
  
    +-------+--------+-------+
    | 0x93  |  0x94  |  0x17 |
    +-------+--------+-------+
    10010011 10010100 00010111
    |||||||| |||||||| ||||||||
    |||||||| |||||||| |||||+++- Frequency   : 132.5 KHz (default)
    |||||||| |||||||| |||++---- Baud Rate   : 2400 bps (default)
    |||||||| |||||||| ||+------ Deviation   : 0.5 (default)
    |||||||| |||||||| |+------- WatchDog    : Disabled
    |||||||| |||||||+ +-------- Tx Timeout  : Disabled
    |||||||| |||||++- --------- Freq D.T.   : 3m usec
    |||||||| ||||+--- --------- Reserved    : 0
    |||||||| ||++---- --------- Preamble    : With Conditioning
    |||||||| |+------ --------- Mains I.M.  : Synchronous
    |||||||+ +------- --------- Output Clock: Off
    ||||||+- -------- --------- Output V.L. : Off
    |||||+-- -------- --------- Header Rec. : Disabled
    ||||+--- -------- --------- Frame Len C.: Disabled
    |||+---- -------- --------- Header Len  : 16 bit
    ||+----- -------- --------- Extended Rgs: Disabled
    |+------ -------- --------- Sensitivity : Normal 
    +------- -------- --------- Input Filter: Enabled
  */
void Modem_CtrlWrite(void){
  digitalWrite(REG_DATA, HIGH);
  digitalWrite(RXTX, LOW);
  digitalWrite(SS_CTRL,LOW);
  SPI_SlaveTransmit(0x08);
  SPI_SlaveTransmit(0x99);
  SPI_SlaveTransmit(0x58);
  SPI_SlaveTransmit(0xAF);
  SPI_SlaveTransmit(0x92);
  SPI_SlaveTransmit(0x17);
  digitalWrite(SS_CTRL,HIGH);
}

void command_library(uint8_t command){
  switch(command){
    case 0xAC:
      digitalWrite(LED,HIGH);
      break;
    case 0xAD:
      digitalWrite(LED,LOW);
      break;
    default:
      break;
  }
}
