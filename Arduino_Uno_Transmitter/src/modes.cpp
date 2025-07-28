#include <RF24.h>
#include <Arduino.h>
#include <Arduino_transmitter_main.h>
#include <modes.h>
#include <LibPrintf.h>

// Poti
int poti = A0;       // Poti Anschluss A0

// State variables
boolean button_state = 0; // init state taster
boolean button_state_old = 0;
boolean taster_change = 0;
byte program = 0;     // Mode/Status Zählervariable
byte program_old = 0; // alter Wert

// mode [0-3]
// picture
// hue [0-255]
// saturation [0-255]
// value_brightness [0-255]
// velocity [0-255]
byte data[6] = {0}; // data array

byte array0[6][5] = {};

int brightness = 0;
int brightness_old = 0;

void video_light_mode_HW(RF24* radio, receiver* teensy)
{
  // Read poti analog input
  brightness = map(analogRead(poti), 0, 1023, 0, 50);

  // Read button input
  button_state = digitalRead(TASTER);

  // Increment program counter
  if (button_state_old == LOW and button_state == HIGH)
  {
    program++;
    button_state_old = HIGH;
  }
  if (button_state_old == HIGH and button_state == LOW)
  {
    button_state_old = LOW;
  }

  // Send on change of program or brightness value
  if (brightness_old != brightness || program_old != program)
  {
    brightness_old = brightness;
    program_old = program;

    if (program > 30)
    {
      program = 0;
    }

    // Serial.print("Program counter: ");
    // Serial.println(program);

    switch (program)
    {
    case 0:
      for (uint8_t i = 0; i < NUM_RECEIVERS; i++)
      {
        data[1] = 0; // Black
        send_data(&teensy[i], data, (uint8_t) sizeof(data), pipe_address, radio, retries_global, delay_retries_global);
        data[1] = 99; // Blink red 1st LED Pixel
        send_data(&teensy[i], data, (uint8_t) sizeof(data), pipe_address, radio, retries_global, delay_retries_global);
      }
      break;

    default:
      for (uint8_t i = 0; i < NUM_RECEIVERS; i++)
      {
        data[1] = program;
        data[3] = brightness;
        send_data(&teensy[i], data, (uint8_t) sizeof(data), pipe_address, radio, retries_global, delay_retries_global);
      }
      break;
    }
  }
}

void pass_on_message(RF24* radio, receiver* teensy, message message_from_pc)
{
  // Serial.print("Video Light Mode - Remote");
  data[0] = message_from_pc.mode;
  data[1] = message_from_pc.picture;
  data[2] = message_from_pc.hue;
  data[3] = message_from_pc.saturation;
  data[4] = message_from_pc.value_brightness;
  data[5] = message_from_pc.velocity;
  // Send on change:
  static message old_message;
  if(message_from_pc.picture != old_message.picture
  || message_from_pc.hue != old_message.hue
  || message_from_pc.mode != old_message.mode
  || message_from_pc.receiver_id != old_message.receiver_id
  || message_from_pc.saturation != old_message.saturation
  || message_from_pc.value_brightness != old_message.value_brightness
  || message_from_pc.velocity != old_message.velocity)
  {
    send_data(&teensy[message_from_pc.receiver_id - 1], data, (uint8_t) sizeof(data), pipe_address, radio, retries_global, delay_retries_global);
    send_receiver_info_to_serial(teensy);
    old_message = message_from_pc;
  }
}

// void send_receiver_info_to_serial(receiver* teensy)
// {
//   // Battery Levels
//   printf("%.2f %.2f %.2f %.2f %.2f %.2f ", (double) teensy[0].voltage, (double) teensy[1].voltage, (double) teensy[2].voltage, (double) teensy[3].voltage, (double) teensy[4].voltage, (double) teensy[5].voltage);
//   // Signal Strength
//   printf("%.2f %.2f %.2f %.2f %.2f %.2f\n", (double) teensy[0].signalStrength, (double) teensy[1].signalStrength, (double) teensy[2].signalStrength, (double) teensy[3].signalStrength, (double) teensy[4].signalStrength, (double) teensy[5].signalStrength);
// }
void send_receiver_info_to_serial(receiver* teensy)
{
  float data[12] = {
    teensy[0].voltage, teensy[1].voltage, teensy[2].voltage,
    teensy[3].voltage, teensy[4].voltage, teensy[5].voltage,
    teensy[0].signalStrength, teensy[1].signalStrength, teensy[2].signalStrength,
    teensy[3].signalStrength, teensy[4].signalStrength, teensy[5].signalStrength
  };
  Serial.write((uint8_t*)data, sizeof(data));  // sends 12 * 4 = 48 bytes
}


/// @brief Send data via the transceiver module
/// @param teensy Target
/// @param pipe_address Pipe_Address; usually fixed; theoretically 6 different pipes possible
/// @param content data array with 5 bytes to be send
/// @param size Size of data_array (content) - must be calculated out of this function
int send_data(receiver* teensy, byte *data, uint8_t size, const uint8_t *pipe_address, RF24 *radio, int retries, int retry_delay)
{
    digitalWrite(LED_RED, HIGH);
    radio->setChannel(teensy->channel);
    radio->setRetries(retry_delay, retries);
    radio->openWritingPipe(pipe_address);
    int status = radio->write(data, size);
    teensy->signalStrength = (1 - (radio->getARC()) / (float) retries) * 100.0;
    int ackPayload = 0;
    if (status)
    {
        if (radio->isAckPayloadAvailable())
        {
          radio->read(&ackPayload, sizeof(ackPayload));
          teensy->voltage = ackPayload/1024.0*2.0*3.3;
        }
        radio->flush_rx();
    }
    digitalWrite(LED_RED, LOW);
//#define DEBUG_SEND
#ifdef DEBUG_SEND
    Serial.print("Data: ");
    Serial.print(data[0]);
    Serial.print(" ");
    Serial.print(data[1]);
    Serial.print(" ");
    Serial.print(data[2]);
    Serial.print(" ");
    Serial.print(data[3]);
    Serial.print(" ");
    Serial.print(data[4]);
    Serial.print("\t to channel:");
    Serial.print(teensy->channel);
    Serial.print(" pipe: ");
    Serial.print(pipe_address[0]);
    Serial.print(pipe_address[1]);
    Serial.print(pipe_address[2]);
    Serial.print(pipe_address[3]);
    Serial.print(pipe_address[4]);
    Serial.print(pipe_address[5]);
    Serial.print(" teensy->voltage: ");
    Serial.println(teensy->voltage);
    // Serial.print("\t size_1: ");
    // Serial.print(size_1);
    // Serial.print("\t size parameter: ");
    // Serial.print(size);
    // Serial.print("\taddress content: ");
    // Serial.println((unsigned int)content, HEX);
#endif
  return status;
}


/// @brief Sending every poi 100 packets and checking how much got received successfully.
///        Starting at the first element of CHs array for all ch_total count
/// @param teensy Array with references to all teensy receivers
/// @param radio Radio object used to send/receive data
/// @param total Total count of receivers
void signal_strength(RF24 *radio, receiver *teensy, int8_t total)
{
    const uint16_t num_test_bytes = 20;
    const uint16_t repititions = 50;
    const uint16_t time_wait = 1000; // Wait time till the next test
    static unsigned long old_time = 0;

    if (millis() > (old_time + time_wait))
    {
      // radio->setRetries(0, 0); // No retries allowed
      for (int j = 0; j < total; j++)
      {
        byte buffer[num_test_bytes] = {0};
        int counter = 0;
        unsigned long startTime, endTime;
        int signalStrength = 0;
        uint8_t status = 0;
        startTime = millis();
        for (uint8_t i = 0; i < repititions; i++)
        {
          // status = radio->write(buffer, sizeof(buffer)); // send x bytes of data. It does not matter what it is
          status = send_data(&teensy[j], buffer, sizeof(buffer), pipe_address, radio, 0, 0);
          if (status)
          {
            counter++;
          }
        }
        endTime = millis();
        float speed = (float)counter * num_test_bytes / (endTime - startTime);
        signalStrength = (float)counter / repititions * 100.0;
        teensy[j].signalStrength = signalStrength;
        // Serial.print("Channel:");
        // Serial.print(CHs[j]);
        // Serial.print("\t");
        // Serial.print("Signal strength: ");
        // Serial.print(signalStrength);
        // Serial.print(" %\t Speed: ");
        // Serial.print(speed);
        // Serial.print(" kB/s");
        // Serial.print("\t Duration: ");
        // Serial.print(endTime - startTime);
        // Serial.print(" ms ");
        // Serial.print("\t Data: ");
        // Serial.print((float)counter * num_test_bytes / 1000.0);
        // Serial.print(" kB");
        // Serial.print("\t Voltage: ");
        // Serial.print(teensy[j].voltage);
        // Serial.println(" V");
      }
      // Serial.println(" ");
      old_time = millis();
      send_receiver_info_to_serial(teensy);
    }
}

void get_serial_message(message *message_input) {
  const byte START_BYTE = 0xAA;

  // Look for the start byte first
  if (Serial.available() >= 8) {
    if (Serial.peek() == START_BYTE) {
      Serial.read(); // consume the start byte
      message_input->mode = Serial.read();
      message_input->receiver_id = Serial.read();
      message_input->picture = Serial.read();
      message_input->hue = Serial.read();
      message_input->saturation = Serial.read();
      message_input->value_brightness = Serial.read();
      message_input->velocity = Serial.read();
    } else {
      // Skip unexpected byte
      Serial.read();
    }
  }
}

