#include <SPI.h>

#define LED      2
#define RXTX     15
#define REG_DATA 18
#define CD_PD    19
#define SS_CTRL  9
#define SIZE     200

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
volatile uint8_t  temp1 = 0, temp2 = 0, cc_byte = 0, check_pointer = 0;
//volatile uint8_t = instruction1 = 0;
volatile uint8_t spi_buffer[SIZE], check_buffer[3];

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  SPI_SlaveInit();
  digitalWrite(SS_CTRL, LOW);
  // SPI.attachInterrupt();   // now turn on interrupts
  digitalWrite(RXTX, HIGH);
  //delay(100);
  //digitalWrite(LED, LOW);
}

void loop() {
	digitalWrite(SS_CTRL, LOW);
	Modem_CtrlWrite();
	digitalWrite(SS_CTRL,HIGH );

	digitalWrite(SS_CTRL, LOW);
	Modem_CtrlRead();
	digitalWrite(SS_CTRL,HIGH );
/*
if(pos_write != pos_read){
  // Combine two bytes to one byte received data 
  if(cc_byte == 0)
  {
    // Shifting MSB four bits 
    temp1 = spi_buffer[pos_read];
    while((temp1&0xf0)>0x10) temp1 = temp1>>1;
    temp1 = temp1<<4;
    cc_byte = 1;
  }
  else
  {
    // Shifting LSB four bits 
    temp2 = spi_buffer[pos_read];
    while((temp2&0xf0)>0x10) temp2=temp2>>1;
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
        temp2 = check_buffer[0];
      else
        temp2 = "error";
      Serial.println(temp2);
    }
    temp1 = 0;
    temp2 = 0;
  }
}
pos_read++;
if (pos_read > SIZE - 1) pos_read = 0; */
}

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for SPI transfer complete
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (SPI_STC_vect){
  spi_buffer[pos_write] = SPDR;
  pos_write++;
  if(pos_write>SIZE-1) pos_write = 0;
} 

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for PCINT1/PB1/D9 changes (SS/PCINT2/PB2)
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (PCINT1_vect){
  //To stop the interrupt we need to set SS/PB2 ?? PB1 ??  Low, which means that there is nothing in the line
  //To continue we need to set the SS high
  if(CD_PD == LOW){
    PORTB &= ~(1<<PB1);
  }else{
    PORTB |= (1<<PB1);
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
  pinMode(SCK,INPUT);   // PB5 --- GND
  pinMode(SS,INPUT);    // PB2 --- ST7540_CLR/T
  pinMode(MOSI,INPUT);  // PB3 --- ST7540_RXD/OUTPUT
  pinMode(MISO,OUTPUT); // PB4 --- ST7540_TXD/INPUT
  pinMode(SS_CTRL,OUTPUT);
  /* Configure pins for ST7540 */
  pinMode(RXTX, OUTPUT);
  pinMode(REG_DATA, OUTPUT);
  pinMode(CD_PD, INPUT);
  /* Enable SPI */
  SPCR = (1<<SPE)|(0<<MSTR);
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
  uint8_t temp1,temp2,temp3;
  uint16_t correction;
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  correction = ((temp1 & 0xFF00)| (temp2 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
  correction = ((temp2 & 0xFF00)| (temp3 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
  correction = ((temp3 & 0xFF00)| (temp1 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
}

void Modem_CtrlWrite(void){
  digitalWrite(REG_DATA, HIGH);
  digitalWrite(RXTX, LOW);
  SPI_SlaveTransmit(0x13);
  SPI_SlaveTransmit(0xB2);
  SPI_SlaveTransmit(0x32);
}