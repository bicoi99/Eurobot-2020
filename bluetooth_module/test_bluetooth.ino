#include <SoftwareSerial.h>

SoftwareSerial bluetooth(10, 11);

const int ledPin = 7;
char state;

void setup()
{
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    Serial.begin(9600);
    bluetooth.begin(9600);
}

void loop()
{
    // Serial.println(Serial.read());
    // Serial.println(state);
    if (bluetooth.available() > 0)
    {
        state = bluetooth.read();

        if (state == '0')
        {
            digitalWrite(ledPin, LOW);
        }
        else if (state == '1')
        {
            digitalWrite(ledPin, HIGH);
        }
        Serial.println(state);
    }

    // if (state == 0)
    // {
    //     digitalWrite(ledPin, LOW);
    //     // Serial.println("LED: OFF"); // Send back, to the phone, the String "LED: ON"
    //     state = 0;
    // }
    // else if (state)
    // {
    //     digitalWrite(ledPin, HIGH);
    //     Serial.println("LED: ON");
    //     state = 0;
    // }
}

// //Include the module so we don't
// //have to use the default Serial
// //so the Arduino can be plugged in
// //to a computer and still use bluetooth
// #include <SoftwareSerial.h>

// //Define the pins used for receiving
// //and transmitting information via Bluetooth
// const int rxpin = 2;
// const int txpin = 3;
// //Variable to store input value
// //initialized with arbitrary value
// char k = 'A';
// //Connect the Bluetooth module
// SoftwareSerial bluetooth(rxpin, txpin);

// //Define the pin to control the light
// int lightbulb = 7;

// void setup()
// {
//     //Set the lightbulb pin to put power out
//     pinMode(lightbulb, OUTPUT);
//     //Initialize Serial for debugging purposes
//     Serial.begin(9600);
//     Serial.println("Serial ready");
//     //Initialize the bluetooth
//     bluetooth.begin(9600);
//     bluetooth.println("Bluetooth ready");
// }

// void loop()
// {
//     Serial.println(bluetooth.available())
//         //Check for new data
//         if (bluetooth.available())
//     {
//         //Remember new data
//         k = bluetooth.read();
//         //Print the data for debugging purposes
//         Serial.println(k);
//     }
//     //Turn on the light if transmitted data is H
//     if (k == 'H')
//     {
//         digitalWrite(7, HIGH);
//     }
//     //Turn off the light if transmitted data is L
//     else if (k == 'L')
//     {
//         digitalWrite(7, LOW);
//     }
//     //Wait ten milliseconds to decrease unnecessary hardware strain
//     delay(10);
// }