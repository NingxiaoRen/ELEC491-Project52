#include <SPI.h>

#define LED 2
#define RXTX  15
#define REG_DATA 18
#define CD_PD 19
#define SIZE 200

void SPI_SlaveInit(void);
byte SPI_SlaveReceive(void);
void modem_correction (uint8_t);

volatile byte buf[SIZE];
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;

void setup() 
{
  pinMode(LED, OUTPUT);
  Serial.begin(115200);
  SPI_SlaveInit();
  //configure modem
  pinMode(RXTX, OUTPUT);
  pinMode(REG_DATA, OUTPUT);
  pinMode(CD_PD, INPUT);
  // now turn on interrupts
  SPI.attachInterrupt();
  digitalWrite(RXTX, HIGH);
  delay(100);
  digitalWrite(LED, LOW);
}

/* SPI transfter complete interrupt routine*/
ISR (SPI_STC_vect)
{
//if (CD_PD == LOW){
  buf[pos_write] = SPDR;
  pos_write++;
  if(pos_write>SIZE-1) pos_write = 0;
//}
  if (buf[pos_write] != 0)
  {
    if (buf[pos_write - 1] == 0)
    {
      Serial.println(0);
    }
    Serial.println(buf[pos_write]);
  }
} 
/* Pin_1 change interrupt routine */
ISR (PCINT1_vect)
{
  if(CD_PD == LOW)
    SS = LOW;
  else
    SS = HIGH;
}
/* Start Looping*/
void loop() 
{
  // The following commented code 1`reads from the control registers
  /*digitalWrite(REG_DATA, HIGH);
  digitalWrite(RXTX, HIGH);
  byte temp1,temp2,temp3;
  //while ((PINB  &   (1<<SCK))!=0);
  temp1 = SPI_SlaveReceive();
  temp2 = SPI_SlaveReceive();
  temp3 = SPI_SlaveReceive();
//delay(10);
  Serial.println(temp1,BIN);
  Serial.println(temp2,BIN);
  Serial.println(temp3,BIN);*/  

  if(buf[pos_write] == 170){
    if (buf[pos_write - 1] == 170){
      if (buf[pos_write - 2] == 170){
        digitalWrite(LED, HIGH);
      }
    }
  }
}
/* Initilize Arduino_SPI to Slave mode*/
void SPI_SlaveInit(void)
{
  /* Set MISO output, all other input */
  pinMode(SCK,INPUT);
  pinMode(MOSI,INPUT);
  pinMode(SS,INPUT);
  pinMode(MISO,OUTPUT);
  /* Enable SPI */
  SPCR = (1<<SPE)|(0<<MSTR);
}

/* Receive data from SPI buffer */
byte SPI_SlaveReceive(void)
{
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}

/* Transimit data */
void SPI_SlaveTransmit(uint8_t cData)
{
  /* Start transmission */ 
  SPDR = cData;
  /* Wait for transmission complete*/ 
  while(!(SPSR & (1<<SPIF)));
}

/* Correct data */
void modem_correction (uint8_t temp)
{
  uint8_t temp1 = 0, temp2 = 0, i = 0;
  /*first configure MSB*/
  temp1 = temp>>4; //add header 0001 for bit correction
  temp1 |= 0x10;   //shift correction protocol
  /*then configure LSB*/
  temp2=temp&0x0f;
  temp2|=0x10;
  /*Transmit each original byte 3 times for failure correction*/
  for (i=0; i<3; i++) 
  {
      SPI_SlaveTransmit(temp1);
      SPI_SlaveTransmit(temp2); 
  }  
}
