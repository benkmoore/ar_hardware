/*
  this script is to reset the desired encoders position to 0. 
  To use, enter the 0, 1, 2, or 3 for the desired encoder. )is the encoder on the outer of the Y arm, 
  1 inner Y, 2 inner X, 3 is the encoder on the outer of the X arm.
  the encoder should reset and then return its current position.
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
int byteInSer;
int resetBytes;
int indexx = 0;
int i = 0;
//these addresses are the DEC version of 0x4C, 0x50, 0x54, 0x58
int encNumber [4] = {76, 80, 84, 88};




void setup()  
{
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Serial ready for transmitting");

  pinMode(Re, OUTPUT);
  pinMode(De, OUTPUT);

  //digitalWrite(RxTx, Receive);   //Initialize transciever to receive data
  RS485Receive();
  Serial2.begin(115200);        // set the data rate

}


void loop()   
{
  if (Serial.available())
  {
    indexx = Serial.parseInt(); 
    RS485Transmit();

    //to reset the position of the encoder, the bytes sent must be: encoder address +2 for the first byte and 94 for the second byte
    resetBytes = ((encNumber[indexx]+2)*1000) + 94;

    Serial2.write(resetBytes);      // Send byte to encoder
    Serial.print("Sent: ");
    Serial.println(resetBytes);
    delay(10);
    Serial2.flush();
    RS485Receive();
    delay(25);



  }
    //after the encoder has reset we ask the encoder for its position by sending it its address
  RS485Transmit();
  Serial2.write(encNumber[indexx]);      // Send byte to encoder
  Serial.print("Sent: ");
  Serial.println(encNumber[indexx]);
  delay(10);
  Serial2.flush();
  RS485Receive();
  delay(25);
  
  i = 0;
  while (Serial2.available())      
  {
    //Serial.println("Received");
    byteIn[i] = Serial2.read();     
    delay(10);
    i ++;
  }

  //remove checksum (two most significant bits)
  byteIn[2] = byteIn[2] << 2;
  byteIn[2] = byteIn[2] >> 2;

  //remove two least significant bits as our encoders are 12 bit, not 14 bit 
  //(2 bytes read = 16 bits - 2 LSB's and 2 MSB'2 gives 12 bit encoder data)
  byteIn[1] = byteIn[1] >> 2;
  //    Serial.println("byte 1: " + String(byteIn[1], BIN));
  //    Serial.println("byte 2: " + String(byteIn[2], BIN));
  response = byteIn[2];
  response = (response << 6) + byteIn[1];

  Serial.print("Response: ");
  //    Serial.println(response, BIN);
  Serial.println(response);
  Serial.println();

  delay(1000);
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
