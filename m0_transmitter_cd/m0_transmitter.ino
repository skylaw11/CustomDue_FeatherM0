
#include <SPI.h>
#include <RH_RF95.h>

//for m0
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

//// for teensy
//#define RFM95_CS 10
//#define RFM95_RST 9
//#define RFM95_INT 2


// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 433.0
#define DATALEN 200       //max size of dummy length

// Blinky on receipt
#define LED 14

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//initialize global variables
char temp[DATALEN];   //lora
uint8_t payload[RH_RF95_MAX_MESSAGE_LEN];
//char dummy_data[DATALEN] = ">>PIVTA*b*016EFD2000026ED43000036EF73000046E114000056E034000066E2E4000076E144000086EEF3000096ED030000A6E0830000B6E3330000C6E1430000D6E363000*170325153449";
// uint8_t len = sizeof(buf);
char send_data[200];

void setup() {
  // put your setup code here, to run once:
	Serial.begin(9600);
  Serial1.begin(9600);
	init_lora();
}

void loop() {
  if (Serial1.available() > 0) {
  String serial_line = Serial1.readStringUntil('\r\n');
  Serial.println(serial_line);
  // put your main code here, to run repeatedly:
//if (Serial1.available() > 0){
//
//for(int i=0; i<Serial1.available(); i++)
//{
//  send_data[i] = Serial1.read();
//  
//}
// Serial.print(send_data);
// send_thru_lora(send_data);
serial_line.toCharArray(send_data, 200); 
send_thru_lora(send_data);

}

	//delay(1000);
}

void init_lora(){
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("LoRa TX Test!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);  
}

void send_thru_lora(char* radiopacket){
  
  int length = sizeof(payload);
  int i=0, j=0;
  // char temp[DATALEN];   //lora
  // uint8_t payload[RH_RF95_MAX_MESSAGE_LEN]; //store the data

  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server

  //do not stack
  for(i=0; i<200; i++){
    payload[i] = (uint8_t)'0';
  }

  for(i=0; i<length; i++){
    payload[i] = (uint8_t)radiopacket[i];
  }

  payload[i] = (uint8_t)'\0';

  Serial.println((char*)payload);


  Serial.println("sending payload!");
  rf95.send(payload, length);

  delay(100);  
}
