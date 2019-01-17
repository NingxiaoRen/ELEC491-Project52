#include <SPI.h>

#define LED 2
#define RXTX  15
#define REG_DATA 18
#define CD_PD 19

#define SIZE 200

void SPI_SlaveInit(void);
byte SPI_SlaveReceive(void);
void modem_correction (uint8_t);

volatile byte buf[SIZE], check[3];
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;
volatile uint8_t = temp1 = 0, temp2 = 0, cc_byte = 0, check_pointer = 0;
volatile uint8_t = instruction1 = 0;

void setup() {
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

// SPI interrupt routine
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

}  // end of interrupt routine SPI_STC_vect

//To stop the interrupt we need to set SS Low, which means that there is nothing in the line
//To continue we need to set the SS high

ISR (PCINT1_vect){
  if(CD_PD == LOW){
    PORTB &= ~(1<<PB1);
  }else{
    PORTB |= (1<<PB1);
  }
}
 
void loop() {

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

//  if(buf[pos_write] == 170){
//    if (buf[pos_write - 1] == 170){
//      if (buf[pos_write - 2] == 170){
//        digitalWrite(LED, HIGH);
//      }
//    }
//  }

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

void SPI_SlaveInit(void){
  pinMode(SCK,INPUT);
  pinMode(MOSI,INPUT);
  pinMode(SS,INPUT);
  pinMode(MISO,OUTPUT);
  //Enable SPI
  SPCR = (1<<SPE)|(0<<MSTR);
  
  //SPSR =(1<<SPI2X);
}

byte SPI_SlaveReceive(void){
  /* Wait for reception complete */
  while(!(SPSR & (1<<SPIF)));
  /* Return Data Register */
  return SPDR;
}

void SPI_SlaveTransmit(uint8_t cData){
  //Start transmission
  SPDR = cData;
  //Wait for transmission complete
  while(!(SPSR & (1<<SPIF)));
}

void modem_correction (uint8_t temp){
  uint8_t temp1 = 0, temp2 = 0, i = 0;

  //add header 0001 for bit correction
  //shift correction protocol

  //THIS HEADER WILL ALSO SERVE AS THE ADDRESS OF EACH DEVICE
  //WE CAN USE THIS HEADER/ADDRESS FOR BIT CORRECTON AND FOR ACK PROTOCOLS BETWEEN MASTER AND SLAVES

  //first configure MSB
  temp1 = temp>>4;
  temp1 |= 0x10;
  //then configure LSB
  temp2=temp&0x0f;
  temp2|=0x10;

  //Transmit each original byte 3 times 
  //for failure correction
  for (i=0; i<3; i++) {
      SPI_SlaveTransmit(temp1);
      SPI_SlaveTransmit(temp2); 
  }  
}
