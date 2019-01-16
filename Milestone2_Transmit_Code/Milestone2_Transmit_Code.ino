#define LED 2
#define RXTX  15
#define REG_DATA 18
#define CD_PD 19

// Comment this out for transmit code

void SPI_SlaveInit(void);
byte SPI_SlaveReceive(void);
void modem_correction (uint8_t);

void setup() {
  pinMode(LED, OUTPUT);
//  digitalWrite(LED, HIGH);
  Serial.begin(115200);
  SPI_SlaveInit();
  
  //configure modem
  pinMode(RXTX, OUTPUT);
  pinMode(REG_DATA, OUTPUT);
  pinMode(CD_PD, INPUT);
  
  delay(100);
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

// STILL GOOD SEND CODE:
  if (digitalRead(CD_PD)==HIGH){    // SEND A MESSAGE!
    // The following code is where the good stuff happens
    digitalWrite(REG_DATA, LOW);
    digitalWrite(RXTX, LOW);
    delay(10);
    Serial.println("Data transmission.");
    digitalWrite(RXTX, HIGH);
    modem_correction(0x15);
    delay(100);
    digitalWrite(RXTX, LOW);
  }

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
