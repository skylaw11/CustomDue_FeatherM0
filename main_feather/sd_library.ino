/* 
  Function: sd_init

    Assert the chip_select pin for SD use.

  Parameters:

    n/a

  Returns:

    -1 fail, 0 success.

  See Also:

    <setup>
*/
int init_sd(){
  if (!SD.begin(g_chip_select)){
    Serial.println("SD initialization failed!");
    return -1;
  }
  return 0;
}

/* 
  Function: init_gids
    
    *g_gids[i][0]* - *uids*
    *g_gids[i][1]* - *gids*

    - Set the uids to 0s.
    - Set the gids to the numbers: 1 - 40.

  Parameters:

    n/a

  Returns:

    n/a

  See Also:
  
    <setup>
*/
void init_gids(){
  if (VERBOSE == 1){Serial.println(F("init_uids()"));}
  for (int i=0; i < g_num_of_nodes; i++ ){
    g_gids[i][0] = 0; // unique id
    g_gids[i][1] = i+1; // geographic id ( 1- 40)
  }
  return;
}
/* 
  Function: open_config

    Open the CONFIG.txt file in the SD card and process
    each line.  

  Parameters:

    n/a

  Returns:

    n/a

  See Also:
  
    <getATCommand>
*/
void open_config(){
  File g_file;
  // address for a character
  char *temp;               
  int max_char_per_line = 1000;
  char one_line[max_char_per_line];
  g_file = SD.open("CONFIG.txt");

//  if (!g_file){
//    Serial.println("CONFIG.txt not found.");
//      hard_code();
//    return;
//  } 
  memset(one_line,0,sizeof(one_line));
  // temp starts at address of one_line
  temp = one_line; 
  // para may paglalagyan ng result ng .peek()            
  while((*temp = g_file.peek())!= -1){  
    // start ulit sa address ng one_line  
    temp = one_line;          
    while(((*temp = g_file.read()) != '\n')) {
      temp++;
      if(*temp == -1)
        break;
    }
    if (process_config_line(one_line)){
      break;
    }
    memset(one_line,0,sizeof(one_line));
  }
  //Serial.println("Finished config processing");
  g_file.close();
  return;
}

/* 
  Function: process_config_line

    Reads one line config and interprets accordingly.

  Parameters:

    *one_line - pointer to a char array that contains the current line of Config being processed.

  Returns:

    0 - successful read of Config line
    1 - reached the end of Config

  See Also:
  
    <open_config>
*/
unsigned int process_config_line(char *one_line){
  String str1;
  int temp_int = 0;
  str1 = String(one_line);
  if ( (str1.startsWith("mastername")) | (str1.startsWith("MasterName")) ){
    get_value_from_line(str1).toCharArray(g_mastername,6);
    return 0;

  } else if(str1.startsWith("turn_on_delay")){
    g_turn_on_delay = get_value_from_line(str1).toInt();
    return 0;

  } else if(str1.startsWith("PIEZO") | str1.startsWith("Piezo")){
    temp_int = get_value_from_line(str1).toInt();
    if (temp_int == 1){
      has_piezo = true;
    } else if ( temp_int == 0){
      has_piezo = false;
    } else {
      has_piezo = false;
    }
    return 0;

  } else if(str1.startsWith("dataloggerVersion")){
    g_datalogger_version = get_value_from_line(str1).toInt();
    return 0;

  } else if(str1.startsWith("sensorVersion")){
    g_sensor_version = get_value_from_line(str1).toInt();
    return 0;

  } else if( (str1.startsWith("broadcastTimeout")) | (str1.startsWith("broadcast_timeout"))
    | (str1.startsWith("broadcasttimeout")) | (str1.startsWith("BroadcastTimeout")) ){
    broad_timeout = get_value_from_line(str1).toInt();
    return 0;

  } else if( (str1.startsWith("sampling_max_retry")) | 
    (str1.startsWith("sampling_max_num_of_retry")) ){
    g_sampling_max_retry = get_value_from_line(str1).toInt();
    return 0;

  }else if((str1.startsWith("columnids")) | (str1.startsWith("column1")) 
    | (str1.startsWith("columnIDs"))) {
    g_num_of_nodes = process_column_ids(str1);
    return 0;

  } else if(str1.startsWith("b64")){
    b64 = get_value_from_line(str1).toInt();
    return 0;

  } else if((str1.startsWith("endofconfig")) | (str1.startsWith("ENDOFCONFIG"))){
    return 1;

  } else {
    return 0;
  }
}
/* 
  Function: process_column_ids

    Find the unique ids from string and place in *<g_gids>*

  Parameters:

    line - String object containing ids

  Returns:

    integer number of unique ids read.

  See Also:
    <process_config_line>
*/
int process_column_ids(String line){
  String delim = ",";
  int i,i0,i1,i2,ilast,id;
  i = 0;
  ilast = line.lastIndexOf(delim);
  i0 = line.lastIndexOf("=");
  i1 = line.indexOf(delim);
  id =  line.substring(i0+1,i1).toInt();
  g_gids[i][0] = id;
  i++;
  while ((i1 <= ilast) & ( i < 40 )){
    i2 = line.indexOf(delim,i1+1);
    id = line.substring(i1+1,i2).toInt();
    g_gids[i][0] = id;
    i++;
    i1 = i2;
    if (i1 == -1)
      break;
  }
  return i; // dating i+1
}
/* 
  Function: print_stored_config

    Print in the gids and uids from the global g_gids and,
    other variables read from CONFIG.txt

  Parameters:

    n/a

  Returns:

    n/a

  See Also:

    <init_gids>
*/
void print_stored_config(){
  char gid[2],uid[4],desc[25];
  Serial.println(F("================================="));
  Serial.println(F("Geographic ID\t\tUnique ID"));
  Serial.println(F("================================="));
  for (int i = 0; i<g_num_of_nodes; i++){
    sprintf(gid,"%2d",g_gids[i][1]);
    sprintf(uid,"%4d",g_gids[i][0]);
    Serial.print("\t");
    Serial.print(gid);
    Serial.print("\t\t");
    Serial.println(uid);
  }
  Serial.println(F("================================="));
  sprintf(desc,"%-24s","Master Name:"); Serial.print(desc); 
  Serial.println(g_mastername);
  sprintf(desc,"%-24s","Number of Nodes"); Serial.print(desc); 
  Serial.println(g_num_of_nodes);
  sprintf(desc,"%-24s","Datalogger Version: "); Serial.print(desc); 
  Serial.println(g_datalogger_version);
  sprintf(desc,"%-24s","Sensor Version: "); Serial.print(desc); 
  Serial.println(g_sensor_version);
  sprintf(desc,"%-24s","Broadcast Timeout: "); Serial.print(desc); 
  Serial.println(broad_timeout);
  sprintf(desc,"%-24s","Sampling Max Retry: "); Serial.print(desc); 
  Serial.println(g_sampling_max_retry);
  sprintf(desc,"%-24s","Turn On Delay: "); Serial.print(desc); 
  Serial.println(g_turn_on_delay);

  Serial.println(F("================================="));
}
/* 
  Function: get_value_from_line

    Extract the value from current line of the config.

  Parameters:

    line - String of one line read from config 

  Returns:

    String after "=" stripped of spaces

  See Also:

    <process_config_line>
*/
String get_value_from_line(String line){
  String delim = "=";
  String other_chars = " "; 
  int index,i;
  while ((i=line.indexOf(other_chars)) != -1){
    line.remove(i,1);   
  }
  index = line.lastIndexOf(delim);
  return line.substring(index+1);
}

/* 
  Function: writeData

    Write data to sd card. Copied from rtc_sd_due ( Duelogger )

  Parameters:

    data -  String data to be written

  Returns:

    -1 - for failure

  See Also:
    <>
*/
int8_t writeData(String fname,String data){
  File sdFile;
  char filename[100]= {};
  char logger_file_name[7] = {};
  String timeString;
  // Serial.println(g_timestamp);

  // if (!SD.begin(6,g_chip_select)) {
  //  Serial.println(" SD.begin() Failed!");
  //  return -1;
  // }
  delay(20);
  
  for(int i=0; i<6 ; i++){
    logger_file_name[i]= fname[i];
  }
  //fname.substring(0,6).toCharArray(filename,6);
  
  strcpy(filename,logger_file_name);
  strcat(filename,".TXT");
  // Serial.println(filename);
  sdFile = SD.open(filename,FILE_WRITE);
  if(!sdFile){
    Serial.println("Can't Write to file");
    return -1;
  }
  sdFile.print(g_mastername);
  sdFile.print(",");
  sdFile.print(data);
  sdFile.print(",");
  sdFile.print(g_timestamp);
  sdFile.println();
  sdFile.close();//close the file
  // Serial.println("writing to SD"); 
}
