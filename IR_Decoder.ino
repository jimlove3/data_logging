/*
  IR Remote Decoder
  -----------------
  Reads signals from an IR remote using a 3-pin IR receiver (VS1838B,
  TSOP38238, etc.) and prints the decoded protocol, address, and command
  to the Serial Monitor.

  Library: IRremote by Armin Joachimsmeyer (v4.x)
           Install via Sketch > Include Library > Manage Libraries > "IRremote"

  Wiring (receiver facing you, dome side up, pins down):
    Left pin   -> Signal  -> Arduino pin 2
    Middle pin -> GND
    Right pin  -> 5V (or 3.3V)
  NOTE: pinouts vary between modules. Check your part's datasheet, or if you
  have a 3-pin breakout board, follow its S / - / + labels.

  Serial Monitor: 115200 baud
*/

#include <Arduino.h>
#include <IRremote.hpp>

const uint8_t IR_RECEIVE_PIN = 2;

void setup() {
  Serial.begin(115200);
  while (!Serial) { }  // Wait for serial on boards that need it

  // Start the receiver; blink the board LED on each received signal
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.println(F("IR decoder ready. Point a remote at the sensor and press a button."));
  Serial.print(F("Listening on pin "));
  Serial.println(IR_RECEIVE_PIN);
  Serial.println();
}

void loop() {
  if (IrReceiver.decode()) {

    // Unknown protocol: print raw timing data so you can analyze or replay it
    if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
      Serial.println(F("Unknown protocol - raw timing data:"));
      IrReceiver.printIRResultRawFormatted(&Serial, true);
    } else {
      // Known protocol: print a readable summary
      Serial.print(F("Protocol: "));
      Serial.println(getProtocolString(IrReceiver.decodedIRData.protocol));

      Serial.print(F("Address:  0x"));
      Serial.println(IrReceiver.decodedIRData.address, HEX);

      Serial.print(F("Command:  0x"));
      Serial.println(IrReceiver.decodedIRData.command, HEX);

      Serial.print(F("Raw data: 0x"));
      Serial.println(IrReceiver.decodedIRData.decodedRawData, HEX);

      if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        Serial.println(F("(repeat - button held down)"));
      } else {
        // Optional: map command codes to button names once you know them
        handleCommand(IrReceiver.decodedIRData.command);
      }

      // Shows how to resend this signal with IrSender
      IrReceiver.printIRSendUsage(&Serial);
    }

    Serial.println(F("-----------------------------"));

    IrReceiver.resume();  // Ready for the next signal
  }
}

// Fill in the case values with the commands YOUR remote reports.
// These are placeholders (common on the cheap 21-key "Car MP3" remote).
void handleCommand(uint16_t command) {
  switch (command) {
    case 0x45: Serial.println(F("Button: CH-"));   break;
    case 0x46: Serial.println(F("Button: CH"));    break;
    case 0x47: Serial.println(F("Button: CH+"));   break;
    case 0x40: Serial.println(F("Button: >>|"));   break;
    case 0x43: Serial.println(F("Button: PLAY"));  break;
    case 0x15: Serial.println(F("Button: VOL+"));  break;
    case 0x07: Serial.println(F("Button: VOL-"));  break;
    default:   Serial.println(F("Button: (not mapped yet)")); break;
  }
}
