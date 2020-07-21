#include <Encoder.h>

// Setup encoder
Encoder encoder(3, 4);

float generateGBN(float prob_switch, float prev_gbn) {
    prob = random(0, 1);
    float gbn = prev_gbn;
    if prob < prob_switch { gbn = -prev_gbn; }
    return gbn;
}

void setup() {
 Serial.begin(57600);
 float u = 1.0; //amplitude of input
}

void loop() {
    u = generateGBN(0.1, u);
    y = encoder.read();
    t = millis();
	Serial.print(y);
	Serial.print(";");
	Serial.print(u);
	Serial.print(";");
	Serial.print(t);
	Serial.println();
}
