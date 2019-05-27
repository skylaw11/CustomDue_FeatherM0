#include "variant.h"
#include <due_can.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_INA219.h>
#include <avr/pgmspace.h>
#include <XBee.h>
#include <stdbool.h>

#include <RTCDue.h>


#define ATCMD     "AT"
#define ATECMDTRUE  "ATE"
#define ATECMDFALSE "ATE0"
#define ATRCVCAN    "ATRCV"
#define ATSNDCAN    "ATSND"
#define ATGETSENSORDATA    "ATGSDT"
#define ATSNIFFCAN  "ATSNIFF"
#define ATDUMP    "ATDUMP"
#define OKSTR     "OK"
#define ERRORSTR  "ERROR"
#define ATSD      "ATSD"
#define DATALOGGER Serial1
#define powerM Serial2


#define VERBOSE 0
#define RELAYPIN 44
#define LED1 48
#define POLL_TIMEOUT 5000
#define BAUDRATE 9600
#define ARQTIMEOUT 30000


#define ENABLE_RTC 0
#define CAN_ARRAY_BUFFER_SIZE 100

// #define comm_mode "ARQ" // 1 for ARQ, 2 XBEE
char comm_mode[5] = "ARQ";

const char base64[64] PROGMEM = {'A','B','C','D','E','F','G','H','I','J','K','L','M',
'N','O','P','Q','R','S','T','U','V','W','X','Y','Z','a','b','c','d','e','f','g','h',
'i','j','k','l','m','n','o','p','q','r','s','t','u','v','w','x','y','z','0','1','2',
'3','4','5','6','7','8','9','+','/'};

XBee xbee = XBee();
long timestart = 0;
long timenow = 0;
long arq_start_time = 0;
// Group: Global Variables
// These are the variables declared globally.

/* 
  Variable: b64
  global variable that turns on the b64 data representation mode. 0 by default.

  An AT Command can be used to toggle this. *AT+SWITCHB64* command toggles on and off
  the b64 data representation. 

  Variable declaration:
  --- Code
  uint8_t b64 = 0;
  --- 
*/
uint8_t b64 = 0;

/* 
  Variable: payload
  global variable uint8_t array that will hold the data from xbee

  Variable declaration:
  --- Code
  uint8_t payload[200];
  --- 
*/
uint8_t payload[200];

XBeeAddress64 addr64 = XBeeAddress64(0x00, 0x00); //sets the data that it will only send to the coordinator
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
XBeeResponse response = XBeeResponse();
ZBRxResponse rx = ZBRxResponse();

int g_chip_select = SS3;
uint8_t datalogger_flag = 0;


/* 
  Variable: xbee_response
  global variable char array that will hold the command frame received from the xbee

  Variable declaration:
  --- Code
  char xbee_response[200];
  --- 
*/
char xbee_response[200];

/*
  Variable: g_gids
  2 dimensional integer array that stores the links the unique id with the geographic id
  
  Variable declaration:
  --- Code
  int g_gids[40][2];
  ---
*/
int g_gids[40][2];

/*
  Variable: g_num_of_nodes
  integer that stores the number of nodes. This variable is overwritten by *<process_config_line>*. This variable is also used by *<init_char_arrays>*.

  Variable declaration:
  --- Code
  uint8_t g_num_of_nodes = 40;
  --- 
*/
uint8_t g_num_of_nodes = 40;

/*
  Variable: g_mastername
  global char array that holds the 5 char mastername. This variable is overwrittern by *<process_config_line>*. This variable defaults to "XXXXX".

  *SD Card Config Line* usage:
  --- Code
  mastername = INATA
  ---
*/
char g_mastername[6] = "XXXXX";

/*
  Variable: g_turn_on_delay
  integer that determines the delay in centi seconds ( ie. 100 centiseconds = 1 sec ) introduced by the firmware after the column switch turns on. 
  This ensures the voltage stability for the column.

  *SD Card Config Line* usage:
  --- Code
  turn_on_delay = 100
  ---
*/
uint8_t g_turn_on_delay = 10;

/*
  Variable: g_sensor_version
  integer that determines the sensor version ( 1, 2, or 3 ). This variable is overwrittern by *<process_config_line>*. 

  *SD Card Config Line* usage:
  --- Code
  sensorVersion = 3
  ---*/
uint8_t g_sensor_version = 3;

/*
  Variable: g_datalogger_version
  integer that determines the datalogger version ( 2 ARQ or 3 Regular,V3 ). This variable is overwrittern by *<process_config_line>*. 
  
  Variable declaration:
  --- Code
  uint8_t g_datalogger_version = 3;
  ---
*/
uint8_t g_datalogger_version = 2;

/* 
  Variable: broad_timeout
  integer that determines the timeout duration of broadcast (in milliseconds ). This variable is overwrittern by *<process_config_line>*. 

  *SD Card Config Line* usage:
  --- Code
  brodcast_timeout = 3000  ---
*/
int broad_timeout = 3000;

/* 
  Variable: has_piezo
  boolean variable that decides the sampling of the piezometer readout board

  *SD Card Config Line* usage:
  --- Code
  PIEZO = 1
  ---
*/
bool has_piezo = false;

/* 
  Variable: g_sampling_max_retry
  integer that determines the number of column sampling retry
  
  *SD Card Config Line* usage:
  --- Code
  sampling_max_retry = 3
  ---
*/
int g_sampling_max_retry = 3;

/* 
  Variable: g_delim
  global char array that holds the delimiter used for separating data from different commands
  
  Variable declaration:
  --- Code
  char g_delim[2] = "~";
  ---
*/
char g_delim[2] = "~";
// CAN-related
char g_temp_dump[2500];
char g_final_dump[5000];
char g_no_gids_dump[2500];

/* 
  Variable: text_message
  char array that holds the formatted messages to be sent

  Intial Variable declaration:
  --- Code
  char text_message[5000];
  ---
  
  See Also:

  <build_txt_msgs>
*/
char text_message[10000];

String g_string;
String g_string_proc;

CAN_FRAME g_can_buffer[CAN_ARRAY_BUFFER_SIZE];

/*
  Variable: g_timestamp
  global String that holds the timestamp.This variable is overwritten by the timestamp from the string sent by the ARQ. This variable defaults to "TIMESTAMP000".

  Intial Variable declaration:
  --- Code
 String g_timestamp = "TIMESTAMP000";
  ---
*/
// String g_timestamp = "180607142000";
String g_timestamp = "TIMESTAMP000";

//current sensor
Adafruit_INA219 ina219;
bool ate=true;

//Group: Main Loop Functions
/* 
  Function: setup

    
    - Sets the baudrate for the Serial communications.

    - Initializes the CAN-related functions.

    - Initializes the global Strings objects used to compile data.

    - Initialize the gids ( geographic ids ) array before use. 

    - Initializes the SD-related functions.

    - Open the CONFIG.txt file, read and load to ram.

    - Display the settings read from CONFIG.txt.

    - Set the GPIO RELAYPIN to output.

  Parameters:
    
    n/a

  Returns:
    
    n/a
  
  See Also:

    <init_can> <init_strings> <init_gids> <init_sd> <open_config>
*/
void turn_on_column(){
  digitalWrite(RELAYPIN, HIGH);
  digitalWrite(LED1,HIGH);
  arqwait_delay(g_turn_on_delay);
}

void turn_off_column(){
  digitalWrite(RELAYPIN, LOW);
  digitalWrite(LED1, LOW);
  arqwait_delay(1000);
}

void arqwait_delay(int milli_secs){
  int func_start = 0;
  if (strcmp(comm_mode,"FEATHER") == 0){
    while ( (millis() - func_start ) < milli_secs ){
//      if ( (millis() - arq_start_time) >= ARQTIMEOUT ) {
//        arq_start_time = millis();
//        Serial.println("ARQWAIT");
//        DATALOGGER.print("ARQWAIT");
//      }
    }
  } else {
    delay(milli_secs);
  }
}

void read_data_from_column(char* column_data, int sensor_version, int sensor_type){
    get_data(11,1,column_data);
    get_data(12,1,column_data);
   // get_data(10,1,column_data);
   // get_data(13,1,column_data);

}

void setup() {
  Serial.begin(BAUDRATE);
  DATALOGGER.begin(9600);
  // powerM.begin(9600);
  ina219.begin();
  pinMode(RELAYPIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  init_can();
  init_char_arrays();
  init_gids();
  init_sd(); 
  open_config();
  //print_stored_config();

  if (g_datalogger_version == 3){
    strncpy(comm_mode,"XBEE",4);
    xbee.setSerial(DATALOGGER);  
  } else if(g_datalogger_version == 2){
    strncpy(comm_mode,"ARQ",3);
  } else if(g_datalogger_version == 4){
    strncpy(comm_mode,"FEATHER",7);  
  } else {
    Serial.print("g_datalogger_version == ");
    Serial.println(g_datalogger_version);
    strncpy(comm_mode,"ARQ",3);
  }
   // Serial.print("Comms: "); Serial.println(comm_mode);
}


/* 
  Function: loop

    - Waits for any activity in the DATALOGGER Serial or xbee Serial for 20 secs.

    - Calls <operation>

    - Sets the datalogger_flag

  Parameters:
    
    n/a

  Returns:
    
    n/a
  
  See Also:

    <operation> <getATCommand> 
*/
void loop(){
//  Serial.println("OK!");
//  delay(500);
//if(Serial.available()>0){
 // if(Serial.read() == 'k'){      
      read_data_from_column(g_final_dump, g_sensor_version,1);
      build_txt_msgs(comm_mode, g_final_dump, text_message);
      g_final_dump[0] = 0;
     // char *token1 = strtok(text_message,"~");
      
//  }
      delay(10000);
//}
 //     }
}

void operation(int sensor_type, char communication_mode[]){
  int counter= 0;
  int num_of_tokens = 0;
  read_data_from_column(g_final_dump, g_sensor_version, sensor_type);// matagal ito.
  Serial.print("g_final_dump: ");
  Serial.println(g_final_dump);
  build_txt_msgs(comm_mode, g_final_dump, text_message); 
}

void build_txt_msgs(char mode[], char* source, char* destination){
  char *token1,*token2;
  char dest[5000] = {};
  char idf = '0';
  char identifier[2] = {};
  char temp[6];
  char temp_id[5];
  char pad[12] = "___________";
  char master_name[8] = "";
  int cutoff = 0, num_text_to_send = 0, num_text_per_dtype = 0;
  int name_len = 0,char_cnt = 0,c=0;
  int i,j;
  int token_length = 0;

  for (int i = 0; i < 5000; i++) {
      destination[i] = '\0';
  }

  String timestamp = getTimestamp(mode);
  char Ctimestamp[12] = "";
  for (int i = 0; i < 12; i++) {
      Ctimestamp[i] = timestamp[i];
  }
  Ctimestamp[12] = '\0';
  
  token1 = strtok(source, g_delim);
  while ( token1 != NULL){
    c=0;
    idf = check_identifier(token1,4);
    identifier[0] = idf;
    identifier[1] = '\0';
    cutoff = check_cutoff(idf);
    remove_extra_characters(token1, idf);

    writeData(timestamp,String(token1));
    num_text_per_dtype = ( strlen(token1) / cutoff );
    if ((strlen(token1) % cutoff) != 0){
      num_text_per_dtype++;
    }

    if (g_sensor_version == 1){
        name_len = 8;
        strncpy(master_name,g_mastername,4);
        strncat(master_name,"DUE",4);
    } else if ( idf == 'p'){
        name_len = 8;
        strncpy(master_name,g_mastername,6);
        strncat(master_name,"PZ",2);
    } else {
        name_len = 6;
        strncpy(master_name,g_mastername,6);
    }

    token_length = strlen(token1); 
    for (i = 0; i < num_text_per_dtype; i++){
      strncat(dest,pad,11);
      strncat(dest,master_name, name_len);
      strncat(dest,"*", 2);
      if (idf != 'p'){ // except piezo
          strncat(dest,identifier,2);
          strncat(dest,"*", 2);
      }
      for (j=0; j < (cutoff); j++ ){
        strncat(dest,token1,1);
        c++;
        token1++;
        if (c == (token_length)){
          break;
        }
      }
      if (strcmp(comm_mode,"XBEE") == 0){
      // Baka dapat kapag V3 ito. 
        strncat(dest,"*",1);
        strncat(dest,Ctimestamp,12);
      }
      strncat(dest,"<<",2);
      strncat(dest,g_delim,1);
    }
    num_text_to_send = num_text_to_send + num_text_per_dtype;
    token1 = strtok(NULL, g_delim);
  }
  token2 = strtok(dest, g_delim);
  c=0;
  while( token2 != NULL ){
    c++;
    char_cnt = strlen(token2) + name_len - 24;
    idf = check_identifier(token1,2);
    identifier[0] = idf;
    identifier[1] = '\0';
    sprintf(pad, "%03d", char_cnt);
    strncat(pad,">>",3);
    sprintf(temp, "%02d/", c);
    strncat(pad,temp,4);
    sprintf(temp,"%02d#",num_text_to_send);
    strncat(pad,temp,4);
    strncpy(token2,pad,11);
    // strncat(token2,"<<",3);
    Serial1.println(token2);
    Serial.println(token2);
    strncat(destination,token2, strlen(token2));
    strncat(destination, g_delim, 2);
    token2 = strtok(NULL, g_delim);

  }

  if (destination[0] == '\0'){
    no_data_parsed(destination);
    writeData(timestamp,String("*0*ERROR: no data parsed"));
  }
//  Serial.println(F("================================="));
}

String getTimestamp(char communication_mode[]){
return String("0TIMESTAMP");
}


char check_identifier(char* token, int index_msgid){
  char idfier = '0';
  char *last_char;
  int id_int;
  char temp_id[5];

  strncpy(temp_id,token,4);
  id_int = strtol(temp_id,&last_char,16);
  if (id_int == 255) {
    idfier = 'p';
    return idfier;
  } else {
    switch (token[index_msgid]) {
        case '1': {
          if (token[index_msgid+1] == '5') {
            idfier = 'b';
          }
          else if (token[index_msgid+1] == 'A') {
            idfier = 'c';
          }
          else if (token[index_msgid+1] == '6') {
            idfier = 'd';
          }
          break;
        } case '0': {
          if (token[index_msgid+1] == 'B')
            idfier = 'x';
          else if (token[index_msgid+1] == 'C')
            idfier = 'y';
          else if (token[index_msgid+1] == '8')
            idfier = 'x';
          else if (token[index_msgid+1] == '9')
            idfier = 'y';
          else if (token[index_msgid+1] == 'A')
            idfier = 'b';
          else if (token[index_msgid+1] == 'D')
            idfier = 'c';
          break;
        } case '2': {
         //Serial.println(token[index_msgid+1]);
         if (token[index_msgid+1] == '0'){
           idfier = 'x';
         } else if (token[index_msgid+1] == '1'){
           idfier = 'y';
         } else if (token[index_msgid+1] == '9'){ //Version 4 idf
           idfier = 'x';
         }else if (token[index_msgid+1] == 'A'){
           idfier = 'y';
         }
         break;
        } case '6': {
          idfier = 'b';
          break;
        } case '7': {
          idfier = 'c';
          break;
        } case 'F' :{
          idfier = 'p';
        } default: {
          idfier = '0';
          break;
        }
      }
  }
  return idfier;
}


int check_cutoff(char idf){
  int cutoff=0;
  switch (idf) {
    case 'b': {
      cutoff = 130;
      break;
    } case 'x': {
      cutoff = 135;  //15 chars only for axel
      break;
    } case 'y': {
      cutoff = 135;  //15 chars only for axel
      break;
    } case 'c': {
      cutoff = 133;  //15 chars only for axel
      break;
    } case 'd': {
      cutoff = 144;  //15 chars only for axel
      break;
    } case 'p' :{
      cutoff = 135; 
      break;
    }

    default: {
            cutoff = 0;
            if (g_sensor_version ==1){
                cutoff = 135;
            }
      break;
    }
  }
  return cutoff;
}



void remove_extra_characters(char* columnData, char idf){
  int i = 0,cmd = 1;
  char pArray[2500] = "";
  int initlen = strlen(columnData);
  char *start_pointer = columnData;

  // dagdagan kapag kailangan
  if (idf == 'd'){
    cmd = 9;
  } else if ( (idf == 'x' ) || (idf == 'y') ){
    cmd = 1;
  } else if ((idf == 'p')){
    cmd = 10;
  } else if (idf=='b'){
    cmd = 3;
  } else if (idf == 'c'){
    cmd = 3;
  }
  for (i = 0; i < initlen; i++, columnData++) {
  // for (i = 0; i < 23; i++,) {
    switch (cmd) {
      case 1: {// axel data //15
        if (i % 20 != 0 && i % 20 != 1  && i % 20 != 8 && i % 20 != 12 && i % 20 != 16 ) {
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 2: { // calib soms // 10
        if (i % 20 != 0 && i % 20 != 1 && i % 20 != 8 && i % 20 != 12 && i % 20 < 14 ) {
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 3: { //raw soms //7
          if (i % 20 != 0 && i % 20 != 1 && i % 20 != 8 && i % 20 < 13 ) { // dating 11 yung last na number
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 4: { // old format
        if (i%20 != 0 && i%20!= 1 && i%20 != 4 && i%20 != 8 && i%20 != 12){
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 8: { // old axel /for 15
        if (i % 20 != 0 && i % 20 != 1 && i % 20 != 8 && i % 20 != 12 && i % 20 != 16 ) {
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 9: {  // diagnostics for v3 sensors //takes temp only 
      // final format is gid(2chars)msgid(2chars)temp_hex(4chars)
        if (i % 20 != 0 && i % 20 != 1 && i % 20 != 6 && i % 20 != 7 && i % 20 != 8 && i % 20 != 9 && i % 20 != 14 && i % 20 != 15 && i % 20 != 16 && i % 20 != 17 && i % 20 != 18 && i % 20 != 19 ) {
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 10:{
        if (i % 20 != 0 && i % 20 != 1 && i % 20 != 8 && i % 20 != 12 && i % 20 != 16){
          strncat(pArray, columnData, 1);
        }
        break;
      }
      case 41: { // axel wrong gids
        if (i % 20 != 4 && i % 20 != 5 && i % 20 != 8 && i % 20 != 12 && i % 20 != 16 ) {
          strncat(pArray, columnData, 1);
        }
        break;
      }
    }
  }

  columnData = start_pointer;
  sprintf(columnData, pArray,strlen(pArray));
}

void no_data_parsed(char* message){
  
  sprintf(message, "040>>1/1#", 3);
  strncat(message, g_mastername, 5);
  strncat(message, "*0*ERROR: no data parsed<<+", 27);
}
