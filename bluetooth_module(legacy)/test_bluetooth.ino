#include <SoftwareSerial.h>

// SoftwareSerial bluetooth(10, 11);

const int ledPin = 7;
char state = '0';

void setup()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    Serial.begin(9600);
    Serial.println("Begin!");
    Serial1.begin(9600);
    // bluetooth.begin(9600);
    // bluetooth.println("Begin!");
}

void loop()
{
    // if (bluetooth.available() > 0)
    // {
    //     state = bluetooth.read();

    //     if (state == '0')
    //     {
    //         digitalWrite(ledPin, LOW);
    //         Serial.println("Light off");
    //     }
    //     else if (state == '1')
    //     {
    //         digitalWrite(ledPin, HIGH);
    //         Serial.println("Light on");
    //     }
    // }
    if (Serial.available() > 0)
    {
        state = Serial.read();

        if (state == '0')
        {
            digitalWrite(ledPin, LOW);
            Serial.println("Light off");
        }
        else if (state == '1')
        {
            digitalWrite(ledPin, HIGH);
            Serial.println("Light on");
        }
    }
    // Serial.println(state);
}