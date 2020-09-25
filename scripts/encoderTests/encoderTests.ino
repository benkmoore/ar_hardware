#define RX        7  //For Arduino Mega
#define TX        8

//#define RxTx 3
#define Re    4
#define De    5

#define Transmit    HIGH
#define Receive     LOW

//#define Pin13LED         13

uint8_t byteIn[3];
long response = 0;
int byteOut;
int i = 0;
int encNumber[] = {76, 80, 84, 88};

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

void loop()   /****** LOOP: RUNS CONSTANTLY ******/
{
//  for (int j = 0; j < 4; j++) {
    //    if (Serial.available())
    //    {


    byteOut = encNumber[1];//Serial.read(); //84;        //Locally store the transmitted string

    RS485Transmit();
    Serial2.write(byteOut);      // Send byte to encoder
    Serial.print("Sent: ");
    Serial.println(byteOut);

    delay(1);
    Serial2.flush();
    RS485Receive();
//    delay(25);
    //    }
    i = 0;
    while (Serial2.available())       //Look for data from encoder
    {
      //Serial.println("Received");
      byteIn[i] = Serial2.read();     // Read received byte

//      delay(10);
      i ++;

//    }

    //remove checksum
    byteIn[2] = byteIn[2] << 2;
    
    //remove checksum
    byteIn[1] = byteIn[1] >> 2;


//    Serial.println("byte 1: " + String(byteIn[1], BIN));

    byteIn[2] = byteIn[2] >> 2;
//    Serial.println("byte 2: " + String(byteIn[2], BIN));

    response = byteIn[2];
    response = (response << 6) + byteIn[1];

    Serial.print("response: ");
//    Serial.println(response, BIN);
    Serial.println(response);
    Serial.println();



//    delay(1000);

  }
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
