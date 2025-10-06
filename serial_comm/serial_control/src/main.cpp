#include <Arduino.h>

#define BAUD_RATE              19200
#define STX                    0xAA
#define ETX                    0xDD
#define CMD_VEL_PACKAGE_SIZE   7
#define WHEEL_ENC_PACKAGE_SIZE 11

// Velocity command package format:
// Big endian format
// [STX][Left RPM High][Left RPM Low][Right RPM High][Right RPM Low][CRC][ETX]

// Format packet: STX | L_MSB | L_LSB | R_MSB | R_LSB | CRC | ETX
// CRC = XOR of L_MSB, L_LSB, R_MSB, R_LSB

static uint8_t cmd_buffer[CMD_VEL_PACKAGE_SIZE];
int16_t left_rpm = 0;
int16_t right_rpm = 0;

// Wheel encoder package format:
// Big endian format
// [STX][L B0][L B1][L B2][L B3][R B0][R B1][R B2][R B3][CRC][ETX]
static uint8_t encoder_buffer[WHEEL_ENC_PACKAGE_SIZE];
int32_t left_encoder = 0;
int32_t right_encoder = 0;

// Send encoder data every 100ms
unsigned long last_send_time = 0;
const unsigned long send_interval = 100; // milliseconds

uint8_t calculateCRC(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    for (size_t i = 1; i < length - 2; ++i) { // Exclude STX and ETX, CRC
        crc ^= data[i];
    }
    return crc;
}

void encoderSend(int32_t left_enc, int32_t right_enc)
{
    encoder_buffer[0] = STX;
    encoder_buffer[1] = (left_enc >> 24) & 0xFF;
    encoder_buffer[2] = (left_enc >> 16) & 0xFF;
    encoder_buffer[3] = (left_enc >> 8) & 0xFF;
    encoder_buffer[4] = left_enc & 0xFF;
    encoder_buffer[5] = (right_enc >> 24) & 0xFF;
    encoder_buffer[6] = (right_enc >> 16) & 0xFF;
    encoder_buffer[7] = (right_enc >> 8) & 0xFF;
    encoder_buffer[8] = right_enc & 0xFF;
    encoder_buffer[9] = calculateCRC(encoder_buffer, WHEEL_ENC_PACKAGE_SIZE);
    encoder_buffer[10] = ETX;

    // Serial.write(encoder_buffer, WHEEL_ENC_PACKAGE_SIZE);
}

void decoderCmdVel(void)
{
    left_rpm = (int16_t)((cmd_buffer[1] << 8) | cmd_buffer[2]);
    right_rpm = (int16_t)((cmd_buffer[3] << 8) | cmd_buffer[4]);

    // Print rpm value
    Serial.print("Received: L= ");
    Serial.print(left_rpm);
    Serial.print(", R= ");
    Serial.println(right_rpm);
}

void setup()
{
    Serial.begin(BAUD_RATE);
    Serial.println("Waiting for data...");
    // setup PWM, driver motor...
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
        Serial.println(last_send_time);
    }

    static uint8_t idx = 0;

    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (idx == 0 && b != STX)
            continue; // Wait STX
        cmd_buffer[idx++] = b;

        if (idx == CMD_VEL_PACKAGE_SIZE) { // Received full package
            idx = 0;
            Serial.println("Get full packet");
            if (cmd_buffer[CMD_VEL_PACKAGE_SIZE - 1] != ETX)
                continue; // Check ETX

                uint8_t crc = calculateCRC(cmd_buffer, CMD_VEL_PACKAGE_SIZE);
                if (crc == cmd_buffer[CMD_VEL_PACKAGE_SIZE - 2]) {
                decoderCmdVel();
                // setMotorRPM(left_rpm, right_rpm);
            } else {
                Serial.print("CMD_VEL Fail CRC: ");
                Serial.print(crc);
                Serial.print(" != ");
                Serial.println(cmd_buffer[CMD_VEL_PACKAGE_SIZE - 2]);
            }
        }
    }
}
