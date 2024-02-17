// Counters
int x=0; // counter to be sent to PC
const int skip_threshold = 15000; // number of main loops that are skipped until get a new counter value
int skip_counter = 0; //counter for skipping main loops

// Variables for managing serial communication with the PC
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
char messageFromPC_1[buffSize] = {0};

// Variables for controlling LEDs
const int red_pin=11;
const int yellow_pin=12;
const int green_pin=13;
String  current_alert="none";
String new_alert="none";

void setup() {
  Serial.begin(57600);
  Serial.setTimeout(1);
  
  pinMode(red_pin, OUTPUT);
  pinMode(yellow_pin, OUTPUT);
  pinMode(green_pin, OUTPUT);
  
  // tell the PC we are ready
  Serial.println("<Arduino is ready>");
}

void messageCb(){
  new_alert=messageFromPC_1;
  current_alert=new_alert;
}

void parseData() {

    // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC_1, strtokIndx); // copy it to messageFromPC

}

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
    
    messageCb();
  
  } 
}

void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<");
    Serial.print(x);
    Serial.println(">");
  }
} 

void activation()
{
  if (current_alert=="none"){
      digitalWrite(green_pin, LOW);   // deactivate the green led
      digitalWrite(red_pin, LOW);   // deactivate the red led
      digitalWrite(yellow_pin, LOW);   // deactivate the yellow led
  }
  
  if (current_alert=="green"){
      digitalWrite(green_pin, HIGH);   // activate the green led
      digitalWrite(red_pin, LOW);   // deactivate the red led
      digitalWrite(yellow_pin, LOW);   // deactivate the yellow led
  }
  if (current_alert=="yellow"){
      digitalWrite(yellow_pin, HIGH);   // activate the yellow led
      digitalWrite(red_pin, LOW);   // deactivate the red led
      digitalWrite(green_pin, LOW);   // deactivate the green led
  }
  if (current_alert=="red"){
      digitalWrite(red_pin,HIGH);   // activate the red led
      digitalWrite(yellow_pin,LOW);   // deactivate the yellow led
      digitalWrite(green_pin,LOW);   // deactivate the green led
  }
}

void loop() {
  getDataFromPC();
  activation();
  if (skip_counter>=skip_threshold) {
    x=x+1;
    skip_counter=0;
  }
  else {
    skip_counter ++;
  }
  replyToPC();
}
