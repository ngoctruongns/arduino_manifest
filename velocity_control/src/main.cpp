#include "process_data_packet.h"
#include "velocity_control.h"
#include <Arduino.h>

// Defne platform to
#define BAUD_RATE 19200

/*******          Global variable        ************/

uint8_t rx_buffer[BUFFER_SIZE] = {0};
uint8_t tx_buffer[BUFFER_SIZE] = {0};

int16_t left_rpm = 0;
int16_t right_rpm = 0;
int32_t left_encoder = 0;
int32_t right_encoder = 0;

// Send encoder data every 100ms
unsigned long last_send_time = 0;
const unsigned long send_interval = 500; // milliseconds

/*******          Function        ************/

void encoderSend(int32_t left_enc, int32_t right_enc)
{
    WheelEncType enc_data;
    enc_data.type = WHEEL_ENC_COMMAND;
    enc_data.left_enc = left_enc;
    enc_data.right_enc = right_enc;
    uint8_t data_len = sizeof(WheelEncType);

    // Prepare data package
    uint8_t tx_len = encoderAllPackage((uint8_t *)&enc_data, data_len, tx_buffer);

    if (tx_len > 0) {
        // Send data via Serial
        Serial.write(tx_buffer, tx_len);
        Serial.println("Encoder send");
    }

    printf("Sending: L= %ld, R= %ld\n", left_enc, right_enc);
}

void decoderCmdVel(uint8_t *buff, uint8_t len)
{
    CmdVelType cmd_vel;
    memcpy(&cmd_vel, buff, len);
    left_rpm  = cmd_vel.left_rpm;
    right_rpm = cmd_vel.right_rpm;

    // Print rpm value
    printf("Received: L= %d, R= %d\n", left_rpm, right_rpm);
    Serial.print("Received: L= ");
    Serial.print(left_rpm);
    Serial.print(", R= ");
    Serial.println(right_rpm);
}

void setup()
{
    Serial.begin(BAUD_RATE);
    Serial.print("cmd_vel: ");
    Serial.print(sizeof(CmdVelType));
    Serial.print(", wheel_enc: ");
    Serial.println(sizeof(WheelEncType));
    // setup PWM, driver motor...
    delay(1000); // wait for serial monitor
    last_send_time = millis();
}

void loop()
{
    // Send encoder data periodically
    if (millis() - last_send_time >= send_interval) {
        last_send_time = millis();

        // Read encoder values
        left_encoder++;  // replace with actual encoder reading
        right_encoder--; // replace with actual encoder reading

        encoderSend(left_encoder, right_encoder);
        // Serial.println(last_send_time);
    }

    if (Serial.available()) {
        uint8_t b = (uint8_t) Serial.read();

        uint8_t cmd_len = handleRxByteConcurrent(b, rx_buffer);
        if (cmd_len > 0) {
            // A complete package is received
            // Process command based on type
            uint8_t cmd_type = rx_buffer[0];
            switch (cmd_type) {
                case DEBUG_STRING:
                    // Handle debug string if needed
                    break;
                case CMD_VEL_COMMAND:
                    if (cmd_len == sizeof(CmdVelType)) {
                        decoderCmdVel(rx_buffer, cmd_len);
                    }
                    break;
                default:
                    // Unknown command type
                    break;
            }
        }
    }
}
