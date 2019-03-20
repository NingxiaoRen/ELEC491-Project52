#define LED      2
#define RXTX     15
#define REG_DATA 16
#define CD_PD    17
#define SS_CTRL  9
#define SIZE     100

  /******************************************************************************
     CD_PD   REG_DATA   RXTX            MODE                DIRECTION
                HIGH    HIGH     Control regiter read   MOSI <-- RXD/OUTPUT
                HIGH     LOW     Control regiter write  MISO --> TXD/INPUT
                LOW     HIGH          Data read         MOSI <-- RXD/OUTPUT
                LOW      LOW          Data write        MISO --> TXD/INPUT
   CD_PD High: Not transfer;  LOW: Transfer
  *******************************************************************************/
volatile uint8_t cmd;
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;
volatile uint16_t pos_check = 0;
volatile uint8_t spi_buffer[SIZE], check_buffer[4];
volatile uint8_t i = 0;
const uint16_t MASTER_HEADER = 0x9B50;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  Serial.begin(115200);
  SPI_SlaveInit();
  pos_read = 0;pos_write = 0;
  digitalWrite(SS_CTRL, LOW);
  PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan (PORTB)
  PCMSK0 |= (1 << PCINT0);   // set PCINT0 to trigger an interrupt on state change (pin pb1 (SW1 button))
  sei();    // turn on interrupts
  digitalWrite(REG_DATA, LOW);
  digitalWrite(RXTX, HIGH);
}

void loop() {
  if(pos_write != pos_read)
  {
      Serial.print(pos_read);
      Serial.print("  ");
      Serial.print(pos_write);
      Serial.print("  ");
      Serial.println(spi_buffer[pos_read], HEX);
      check_buffer[pos_check] = spi_buffer[pos_read];
      spi_buffer[pos_read] = 0;
      pos_read++; pos_check ++;
      if (pos_read > SIZE - 1) pos_read = 0;
      if (pos_check == 4)
      {
        pos_check = 0;
        if((check_buffer[0]==check_buffer[1]) ||(check_buffer[2]==check_buffer[3]))
        {
          if((check_buffer[0]==check_buffer[1]))
            cmd = check_buffer[0];
          if((check_buffer[2]==check_buffer[3]))
            cmd = check_buffer[2]; 
          command_library(cmd); 
        }
        else
        {
          //Serial.println("Resend commands !!");
        }
       }
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
  delay(100);
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
void DataCorrection_Transmit (const uint16_t header, uint8_t command)
{
	// Transmit mode
	digitalWrite(REG_DATA, LOW);
	digitalWrite(RXTX, LOW);
	// Send the header
	SPI_SlaveTransmit(header>>8);
	SPI_SlaveTransmit(header);
	// Send the command twice
	for (i=0; i<2; i++) 
		SPI_SlaveTransmit(command); 
	// End transmission
	digitalWrite(RXTX, HIGH);
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
  SPI_SlaveTransmit(0x01);
  SPI_SlaveTransmit(0x9B);
  SPI_SlaveTransmit(0x58);
  SPI_SlaveTransmit(0xAF);
  SPI_SlaveTransmit(0x92);
  SPI_SlaveTransmit(0x17);
  digitalWrite(SS_CTRL,HIGH);
}

/*******************************************************************************
* Function Name  : Modem_CtrlRead
* Description    : Request control register data from ST7540
* Input          : None
* Output         : None
*******************************************************************************/
void Modem_CtrlRead()
{
  char buf[32]; 
  uint64_t readBuffer = 0;
  uint64_t control_reg = 0x029B58AF92170000;
  // Drive REG_ATA and RXTX high to request control register data from ST7540 
  Serial.print("CtrlReg Check: ");
  digitalWrite(REG_DATA,HIGH);
  digitalWrite(RXTX, HIGH);
  // Reading data 
  digitalWrite(SS_CTRL,LOW);
  for(i = 0; i < 8; i ++)
    readBuffer = (readBuffer<<8) | SPI_SlaveReceive();
  digitalWrite(SS_CTRL,HIGH);
  // Verify data
  while(((readBuffer>>32) & 0x02000000)!= 0x02000000)
    readBuffer = readBuffer<<1;  
  sprintf(buf, "%08lX", readBuffer>>32);
  Serial.print(buf);
  sprintf(buf, "%08lX", readBuffer);
  Serial.print(buf);
  if(readBuffer == control_reg)
      Serial.println("  Pass !!");
  else
      Serial.println("  Fail !!!");
}


void command_library(uint8_t command){
  switch(command){
    case 0xAC:
      digitalWrite(LED,HIGH);
	  Serial.println("AC ON");
      break;
    case 0xAD:
      digitalWrite(LED,LOW);
	  Serial.println("AD OFF");
      break;
    default:
      break;
  }
}
