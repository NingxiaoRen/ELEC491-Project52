#define LED      2
#define RXTX     15
#define REG_DATA 18
#define CD_PD    19
#define SS_CTRL  9
#define BUTTON   2

uint8_t i = 0;
volatile int button = 0;
volatile int light_flag = 0;
  /******************************************************************************
     CD_PD   REG_DATA   RXTX            MODE                DIRECTION
                HIGH    HIGH     Control regiter read   MOSI <-- RXD/OUTPUT
                HIGH     LOW     Control regiter write  MISO --> TXD/INPUT
                LOW     HIGH          Data read         MOSI <-- RXD/OUTPUT
                LOW      LOW          Data write        MISO --> TXD/INPUT
   CD_PD High: Not transfer;  LOW: Transfer
  *******************************************************************************/
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(BUTTON,INPUT);
  attachInterrupt(0, pin_ISR, RISING);
  //digitalWrite(LED, HIGH);
  Serial.begin(115200);
  SPI_SlaveInit();
  digitalWrite(SS_CTRL,LOW);
}
 
void loop() {
 
  
  //if (digitalRead(CD_PD)==HIGH) // When the lines are not busy
  //{    
    //Serial.println("Data transmission.");
    //digitalWrite(SS_CTRL,LOW);
//    DataCorrection_Transmit(0x12);
//    DataCorrection_Transmit(0x34);
//    DataCorrection_Transmit(0x56);
//    DataCorrection_Transmit(0x78);
//    DataCorrection_Transmit(0x90);
//    DataCorrection_Transmit(0xAB);
//    DataCorrection_Transmit(0xCD);
//    DataCorrection_Transmit(0xEF);
    //digitalWrite(SS_CTRL,HIGH);
    //delay(1000);
  //}*/
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
  pinMode(MOSI,INPUT);  // PB3 --- ST7540_RXD
  pinMode(MISO,OUTPUT); // PB4 --- ST7540_TXD
  pinMode(SS_CTRL,OUTPUT); 
  /* Configure pins for ST7540 */
  pinMode(RXTX, OUTPUT);
  pinMode(REG_DATA, OUTPUT);
  pinMode(CD_PD, INPUT);
  /* Enable SPI */
  SPCR = (1<<SPE)|(0<<MSTR);
  Modem_CtrlWrite();
  Modem_CtrlRead();
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
  //delay(200);
  digitalWrite(RXTX, HIGH);
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
  uint8_t temp1,temp2,temp3, temp4;
  uint32_t correction1 = 0;
  uint32_t correction2 = 0;
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  temp4 = SPI_SlaveReceive();
  correction1 =  ((uint32_t)temp1<<24) | ((uint32_t)temp2<<16) | ((uint32_t)temp3<<8) | ((uint32_t)temp4);
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
  temp4 = SPI_SlaveReceive();
  correction2 =  ((uint32_t)temp1<<24) | ((uint32_t)temp2<<16) | ((uint32_t)temp3<<8) | ((uint32_t)temp4);
  //while(correction1 != correction2)
  Serial.println("C 1");
  Serial.println(correction1,BIN);
  Serial.println("C 2");
  Serial.println(correction2,BIN);
  //Serial.println("byte 3");
  //Serial.println(temp3,BIN);
  /*correction = (((temp1| correction)<<8)| (temp2 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
  //Serial.println(temp1,BIN);
  correction = 0;
  correction = (((temp2| correction)<<8)| (temp3 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
  //Serial.println(temp2,BIN);
  correction = 0;
  correction = (((temp3| correction)<<8)| (temp1 & 0x00FF)) << 1;
  Serial.println(correction >> 8,HEX);
  //Serial.println(temp3,BIN);*/
}

void Modem_CtrlWrite(void){
  digitalWrite(REG_DATA, HIGH);
  digitalWrite(RXTX, LOW);
  SPI_SlaveTransmit(0x13);
  SPI_SlaveTransmit(0xB2);
  SPI_SlaveTransmit(0x32);
}

void pin_ISR(){
  button = digitalRead(BUTTON);
  if ((button == HIGH) & (light_flag == 0)){
    DataCorrection_Transmit(0xAC);
    light_flag = 1;
  }else{
    DataCorrection_Transmit(0xAD);
    light_flag = 0;
  }
}
