const int ledPin = 7;
bool state = 0;

void setup()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    Serial.begin(115200);
}

void loop()
{
    if (Serial.available() > 0)
    {
        state = Serial.read();
    }

    if (state == 0)
    {
        digitalWrite(ledPin, LOW);
        Serial.println("LED: OFF"); // Send back, to the phone, the String "LED: ON"
        state = 0;
    }
    else if (state == '1')
    {
        digitalWrite(ledPin, HIGH);
        Serial.println("LED: ON");
        state = 0;
    }
}