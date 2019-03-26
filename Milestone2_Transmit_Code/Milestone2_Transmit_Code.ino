#include <LiquidCrystal_I2C.h>
#include <Wire.h> 
//#define BUTTON1  2
//#define BUTTON2  3
#define SS_CTRL  9
#define RXTX     15
#define REG_DATA 16
#define CD_PD    17
#define SIZE     40

#define SELECT   2
#define DOWN     4
int buttonPress = 0;
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
 
uint8_t i = 0;
volatile int light1_flag = 0;
volatile int light2_flag = 0;
const uint16_t SLAVE1_HEADER = 0x9B58;
const uint16_t SLAVE2_HEADER = 0x9118;
volatile uint16_t pos_write = 0;
volatile uint16_t pos_read = 0;
volatile uint8_t check_pointer = 0;
volatile uint8_t spi_buffer[SIZE], check_buffer[2];

volatile int selection0 = 1;
volatile int selection1 = 1;
volatile int selection2 = 1;
volatile boolean buttonState_UP = HIGH;
volatile int prt = 0;
volatile int MENU = 0;

  /******************************************************************************
     CD_PD   REG_DATA   RXTX            MODE                DIRECTION
                HIGH    HIGH     Control regiter read   MOSI <-- RXD/OUTPUT
                HIGH     LOW     Control regiter write  MISO --> TXD/INPUT
                LOW     HIGH          Data read         MOSI <-- RXD/OUTPUT
                LOW      LOW          Data write        MISO --> TXD/INPUT
   CD_PD High: Not transfer;  LOW: Transfer
  *******************************************************************************/
void setup() {
//  pinMode(BUTTON1,INPUT);
//  pinMode(BUTTON2,INPUT);
//  attachInterrupt(0, pin_ISR1, RISING);
//  attachInterrupt(1, pin_ISR2, RISING);
  Serial.begin(115200);
  SPI_SlaveInit();  
  digitalWrite(SS_CTRL,LOW);
  Serial.begin(115200);
  pinMode(SELECT,INPUT);
  digitalWrite(SELECT,HIGH);
  pinMode(DOWN,INPUT);
  digitalWrite(DOWN,HIGH);
  lcd.begin(20,4);
  lcd.setCursor(3,0);
  lcd.print("Hello Team 52");
  lcd.setCursor(2,1);
  lcd.print("Welcome to your");
  lcd.setCursor(4,2);
  lcd.print("Smart Home "); 
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Select a device to");
  lcd.setCursor(5,1);
  lcd.print("control:");
  lcd.setCursor(0,2);
  lcd.print("Device 01 <<");
  lcd.setCursor(0,3);
  lcd.print("Device 02");
  delay(1000);
  /*PCICR |= (1 << PCIE0);     // set PCIE0 to enable PCMSK0 scan (PORTB)
  PCMSK0 |= (1 << PCINT0);   // set PCINT0 to trigger an interrupt on state change (pin pb1 (SW1 button))
  sei();    // turn on interrupts*/
}
 
void loop() {
  buttonPress = digitalRead(SELECT);
  if(buttonPress == 0){
    delay(50);
    buttonPress = digitalRead(SELECT);
    if(buttonPress == 1){
       switch (MENU) {
      case 0: 
        if(selection0 == 1){
          MENU = 1;
          lcd.clear();
          prt = 1;
          selection0 = 1;
        }else if (selection0 == 2){
          MENU = 2;
          lcd.clear();
          prt = 1;
          selection0 = 1;
        }
      break;
      case 1:
        if(selection1 == 1){
          Serial.println("LED ON");
          DataCorrection_Transmit(SLAVE1_HEADER, 0xFC);
        }else if(selection1 == 2){
          Serial.println("LED OFF");
          DataCorrection_Transmit(SLAVE1_HEADER, 0xFD);
        }else if(selection1 == 3){
          MENU = 0;
          lcd.clear();
          prt = 1;
          selection1 = 1;
        }
      break;
      case 2:
        if(selection2 == 1){
          Serial.println("Play Music 1");
          DataCorrection_Transmit(SLAVE2_HEADER, 0xF1);
        }
        else if(selection2 == 2){
          Serial.println("Play Music 2");
          DataCorrection_Transmit(SLAVE2_HEADER, 0xF2);
        }
        else if (selection2 == 3){
          MENU = 0;
          lcd.clear();
          prt = 1;
          selection2 = 1;
        }
      break;
      }
    }  
  }

  buttonPress = digitalRead(DOWN);
  if(buttonPress == 0){
    delay(50);
    buttonPress = digitalRead(DOWN);
    if(buttonPress == 1){
      switch (MENU) {
      case 0: 
        selection0 += 1;
        prt = 1;
      break;
      case 1:
        selection1 += 1;
        prt = 1;
      break;
      case 2:
        selection2 += 1;
        prt = 1;
      break;
      }
    }
      switch (MENU) {
        case 0:
          if (selection0 > 2){
            selection0 = 1;
          }
        break;
        case 1:
          if (selection1 > 3){
            selection1 = 1;
          }
        break;
        case 2:
          if (selection2 > 3){
            selection2 = 1;
          }
        break;
      }
    }
  
  if (selection0 == 1 && prt == 1 && MENU == 0){
    lcd.setCursor(0,0);
    lcd.print("Select a device to");
    lcd.setCursor(5,1);
    lcd.print("control:");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Device 01 <<");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Device 02");
    prt = 0;
  }else if(selection0 == 2 && prt == 1 && MENU == 0){
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(0,3);
    lcd.print("Device 02 <<");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Device 01");
    prt = 0;
  }

  if (selection1 == 1 && prt == 1 && MENU == 1){ //Device 1
    lcd.setCursor(0,0);
    lcd.print("Device 1 Menu");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Turn ON  <<");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(15,3);
    lcd.print("HOME");
    prt = 0;
  }else if(selection1 == 2 && prt == 1 && MENU == 1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Turn ON");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF <<");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(15,3);
    lcd.print("HOME");
    prt = 0;
  }else if(selection1 == 3 && prt == 1 && MENU == 1){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Turn ON");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(12,3);
    lcd.print(">> HOME");
    prt = 0;
  }

  if (selection2 == 1 && prt == 1 && MENU == 2){ // Device 2
    lcd.setCursor(0,0);
    lcd.print("Device 2 Menu");
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Turn ON <<");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF ");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(15,3);
    lcd.print("HOME");
    prt = 0;
  }else if(selection2 == 2 && prt == 1 && MENU == 2){
    lcd.setCursor(0,1);
    lcd.print("                    ");
    lcd.setCursor(0,1);
    lcd.print("Turn ON");
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF <<");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(15,3);
    lcd.print("HOME");
    prt = 0;
  }else if(selection2 == 3 && prt == 1 && MENU == 2){
    lcd.setCursor(0,2);
    lcd.print("                    ");
    lcd.setCursor(0,2);
    lcd.print("Turn OFF");
    lcd.setCursor(0,3);
    lcd.print("                    ");
    lcd.setCursor(12,3);
    lcd.print(">> HOME");
    prt = 0;
  }
}

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for SPI transfer complete
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
/*ISR (SPI_STC_vect){
  //byte c = SPDR;
  spi_buffer[pos_write] = SPDR;
  //Serial.println(spi_buffer[pos_write],HEX);
  pos_write++;
  if(pos_write>SIZE-1) pos_write = 0;
} */

/*******************************************************************************
* Function Name  : ISR
* Description    : Interrupt Service Routine for PCINT1/PB1/D9 changes (SS/PCINT2/PB2)
* Input          : SPI transfer complete
* Output         : Print data in SPI buffer
*******************************************************************************/
/*ISR (PCINT0_vect){
  //To stop the interrupt we need to set SS/PB2 ?? PB1 ??  Low, which means that there is nothing in the line
  //To continue we need to set the SS high
  if(PINB & (1<<PB0)){//rising
    //Serial.println("CD_PD Rising");
    digitalWrite(SS_CTRL,HIGH);
  }else{//falling
    //Serial.println("CD_PD Falling");
    digitalWrite(SS_CTRL,LOW);
  }
}*/

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
void DataCorrection_Transmit (const uint16_t header, uint8_t command)
{
	// Transmit mode
	digitalWrite(REG_DATA, LOW);
	digitalWrite(RXTX, LOW);
	// Send the header
	SPI_SlaveTransmit((uint8_t)(header>>8));
	SPI_SlaveTransmit((uint8_t)header);
	// Send the command twice
  SPI_SlaveTransmit(command);
	SPI_SlaveTransmit(command);
  SPI_SlaveTransmit(command);
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
  SPI_SlaveTransmit(0x08);
  SPI_SlaveTransmit(0x9B);
  SPI_SlaveTransmit(0x50);
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
  uint64_t control_reg = 0x089B50AF92170000;
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
  while(((readBuffer>>32) & 0x08000000)!= 0x08000000)
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

// Buttons
//void pin_ISR1(){
//  button1 = digitalRead(BUTTON1);
//  if ((button1 == HIGH) & (light1_flag == 0)){
//    DataCorrection_Transmit(0x10,0xAC);
//    light1_flag = 1;
//  }else{
//    DataCorrection_Transmit(0x10,0xAD);
//    light1_flag = 0;
//  }
//}
//
//void pin_ISR2(){
//  button2 = digitalRead(BUTTON2);
//  if ((button2 == HIGH) & (light2_flag == 0)){
//    DataCorrection_Transmit(0x20,0xAC);
//    light2_flag = 1;
//  }else{
//    DataCorrection_Transmit(0x20,0xAD);
//    light2_flag = 0;
//  }
//}
