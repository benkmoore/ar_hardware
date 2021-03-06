#define RX        7  //For Arduino Mega
#define TX        8

//#define RxTx 3
#define Re    3
#define De    4

#define Transmit    HIGH
#define Receive     LOW

//#define Pin13LED         13

uint8_t byteIn[3];
long response = 0;
int byteOut;
int i = 0;
void setup()   /****** SETUP: RUNS ONCE ******/
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial ready for transmitting");

  //pinMode(Pin13LED, OUTPUT);
  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);

  //digitalWrite(RxTx, Receive);   //Initialize transciever to receive data
  RS485Receive();

  Serial2.begin(115200);        // set the data rate

}//--(end setup )---


void loop()   
{
//    if (Serial.available())
//    {
  byteOut = 88;//Serial.read(); //84;        //Locally store the transmitted string

  RS485Transmit();
  Serial2.write(byteOut);      // Send byte to encoder
  Serial.print("Sent: ");
  Serial.println(byteOut);
  Serial.println();

  //digitalWrite(Pin13LED, LOW);      // Off momentarily
  delay(10);
  Serial2.flush();
  RS485Receive();
  delay(25);
//    }
  i = 0;
  while (Serial2.available())       //Look for data from encoder
  {
    //Serial.println("Received");
    //digitalWrite(Pin13LED, LOW);        // Off momentarily
    byteIn[i] = Serial2.read();     // Read received byte
    //Serial.println("byte " + String(i) +": " + String(byteIn[i], BIN));

    delay(10);
    i ++;
    //Serial.println(response,BIN);
    //Serial.println("-----------------------");

//    unsigned int len = 0;

 
  }
//
//  Serial.println("byte 1: " + String(byteIn[1], BIN));
//  Serial.println("byte 2: " + String(byteIn[2], BIN));

  byteIn[2] = byteIn[2] << 2;
  byteIn[1] = byteIn[1] >> 2;


  Serial.println("byte 1: " + String(byteIn[1], BIN));
//  Serial.println("byte 2: " + String(byteIn[2], BIN));

  byteIn[2] = byteIn[2] >> 2;
  Serial.println("byte 2: " + String(byteIn[2], BIN));

  response = byteIn[2];
  response = (response << 6) + byteIn[1];
  //remove checksum
  //  response = (response >> 2);
  //  response<<=12;

  Serial.print("response: ");
  Serial.println(response, BIN);
  Serial.println(response);




}

void RS485Transmit()
{
  digitalWrite(Re, LOW);
  digitalWrite(De, HIGH);
}

void RS485Receive()
{
  digitalWrite(Re, HIGH);
  digitalWrite(De, LOW);
}
