#include <SPI.h>

#define LED 2
#define RXTX  15
#define REG_DATA 18
#define CD_PD 19

#define SIZE 200
  /******************************************************************************
     CD_PD   REG_DATA   RXTX                       MODE
                HIGH    HIGH     Control regiter read through MOSI - RXD/OUTPUT
                HIGH     LOW     Control regiter write through MISO - TXD/INPUT
                LOW     HIGH          Data read through MOSI - RXD/OUTPUT
		        LOW      LOW          Data write through MISO - TXD/INPUT
   CD_PD High: Not transfer;  LOW: Transfer
  *******************************************************************************/
void SPI_SlaveInit(void);
byte SPI_SlaveReceive(void);
void DataCorrection_Transmit (uint8_t);

volatile byte buf[SIZE], check[3];
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;
volatile uint8_t = temp1 = 0, temp2 = 0, cc_byte = 0, check_pointer = 0;
volatile uint8_t = instruction1 = 0;

void setup() {
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  SPI_SlaveInit();
  
  // now turn on interrupts
  // SPI.attachInterrupt();
  digitalWrite(RXTX, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

void loop() {

//Modem_CtrlRead()

//From the transmission, we know that we send a header byte which corresponds to the device

//Check the received data 
if(pos_write != pos_read){

  if(cc_byte == 0){
    temp1 = buf[pos_read];
    while((temp1&0xf0)>0x10) temp1 = temp1>>1;
    //Shift to MSB after correction the temp value
    temp1 = temp1<<4;
    cc_byte = 1;
  }else{
    temp2 = buf[pos_read];
    while((temp2&0xf0)>0x10) temp2=temp2>>1;
    temp2 &= 0x0f;
    //cleared MSB, or it with the address chunk to obtain full amount of data
    temp2 |= temp1; 
    cc_byte = 0;

    if(temp2 == instruction1){
      check_pointer ++;
      check_ok = 1;
    }else{
      check_ok;
    }
  temp1 = 0;
  temp2 = 0;
  }
}
pos_read++;
if (pos_read > SIZE - 1) pos_read = 0; 

}

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for SPI transfer complete
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (SPI_STC_vect){
//if (CD_PD == LOW){
  buf[pos_write] = SPDR;
  pos_write++;
  if(pos_write>SIZE-1) pos_write = 0;
//}
  if (buf[pos_write] != 0){
    if (buf[pos_write - 1] == 0){
      Serial.println(0);
    }
    Serial.println(buf[pos_write]);
  }
} 

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for PCINT1/PB1/D9 changes (SS/PCINT2/PB2)
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
ISR (PCINT1_vect){
  //To stop the interrupt we need to set SS ?? PB1 ??  Low, which means that there is nothing in the line
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
byte SPI_SlaveReceive(void){
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
  for (i=0; i<3; i++) {
      SPI_SlaveTransmit(temp1);
      SPI_SlaveTransmit(temp2); 
  }  
}

/*******************************************************************************
* Function Name  : DataCorrection_Receive
* Description    : Divide one byte data into two bytes by adding a header to each
				   four bits of the data. the header will also serve as the address
				   of each device. we can use the header for bit correction and for 
				   ack protocols between master and slaves.
* Input          : transmitted data
* Output         : None
* Return         : None
*******************************************************************************/
void DataCorrection_Receive (uint8_t temp){
  uint8_t temp1 = 0, temp2 = 0, i = 0;
  /* Configure the Most Siganificant Four bits of the Byte as: 0001 MSFB */
  temp1 = (temp >> 4) | 0x10;
  /* Configure the Least Siganificant Four bits of the Byte as: 0001 LSFB */
  temp2 = (temp & 0x0f) | 0x10;
  /* Transmit each original byte 3 times for failure correction*/
  for (i=0; i<3; i++) {
      SPI_SlaveTransmit(temp1);
      SPI_SlaveTransmit(temp2); 
  }  
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
  byte temp1,temp2,temp3;
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  Serial.println(temp1,BIN);
  Serial.println(temp2,BIN);
  Serial.println(temp3,BIN);
}
