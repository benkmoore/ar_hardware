/*
  this script is to reset the position of the 4 encoders to 0.
*/

#define RX        7
#define TX        8

//#define RxTx 3
#define Re    3
#define De    4

#define Transmit    HIGH
#define Receive     LOW


uint8_t byteIn[3];
long response = 0;
int byteOut;
int i = 0;
int encNumber [4] = {76, 80, 84, 88};



void setup()
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);

  //digitalWrite(RxTx, Receive);   //Initialize transciever to receive data
  //  RS485Receive();
  Serial2.begin(115200);        // set the data rate
  RS485Transmit();


}


void loop()
{

  Serial2.write("N^");      // Send byte to encoder
  delay(10);

  Serial2.write("R^");      // Send byte to encoder
  delay(10);

  Serial2.write("V^");      // Send byte to encoder
  delay(10);

  Serial2.write("Z^");      // Send byte to encoder
  delay(10);


  RS485Transmit();

  for (int j = 0; j < 4; j++) {
    RS485Transmit();
    Serial2.write(encNumber[j]);      // Send byte to encoder
   

    delay(10);
    RS485Receive();

    delay(10);
    //    Serial2.flush();
    response = listen();
Serial.println(response);
    delay(100);
  }


  delay(500);
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

long listen() {
  i = 0;
  for (int ind = 0; ind < 3; ind ++) {
    byteIn[ind] = 0;
  }
  if (Serial2.available()) {
    while (Serial2.available())
    {
      //Serial.println("Received");
      byteIn[i] = Serial2.read();
      delay(10);
      i ++;
    }
  }
  //remove checksum (two most significant bits)
  byteIn[2] = byteIn[2] << 2;
  byteIn[2] = byteIn[2] >> 2;

  //remove two least significant bits as our encoders are 12 bit, not 14 bit
  //(2 bytes read = 16 bits - 2 LSB's and 2 MSB'2 gives 12 bit encoder data)
  byteIn[1] = byteIn[1] >> 2;
  response = byteIn[2];
  response = (response << 6) + byteIn[1];
  return response;
}
