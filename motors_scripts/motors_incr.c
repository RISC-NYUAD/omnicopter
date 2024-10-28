#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>

void sendSerialData(uint8_t* data, size_t size) {
    int serial = open("/dev/ttyACM0", O_WRONLY | O_NOCTTY);
    FILE *serial2 = fopen("/home/ama10362/Desktop/test", "w");

    if (serial < 0) {
        printf("Failed to open serial port\n");
        exit(1);
    }

    if (serial2 == NULL) {
        printf("Failed to open file\n");
        exit(1);
    }

    struct termios options;
    tcgetattr(serial, &options);
    cfsetispeed(&options, B115200);  // Set input baud rate
    cfsetospeed(&options, B115200);  // Set output baud rate
    tcsetattr(serial, TCSANOW, &options);

    write(serial, data, size);
    fwrite(data, 1, size, serial2);

    fclose(serial2);
    close(serial);
}

double getElapsedTime(struct timespec *start) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (now.tv_sec - start->tv_sec) + (now.tv_nsec - start->tv_nsec) / 1e9;
}

int main(int argc, char *argv[])
{
    if (argc != 10)
    {
        printf("Wrong number of arguments\n");
        exit(1);
    }

    for (int i = 1; i <= 8; i++)
    {
        if (atoi(argv[i]) < 1040 || atoi(argv[i]) > 1960)
        {
            printf("One or more thrusts are invalid\n");
            exit(1);
        }
    }

    if (atoi(argv[9]) < 0 || atoi(argv[9]) > 30)
    {
        printf("Time parameter is invalid\n");
        exit(1);
    }

    int duration = atoi(argv[9]);
    int steps = duration * 10;  // 100ms intervals
    float sleepInterval = 0.1;  // 100ms interval as float

    int16_t targetSpeeds[8];
    float currentSpeeds[8] = {1040, 1040, 1040, 1040, 1040, 1040, 1040, 1040};
    float increments[8];

    for (int i = 0; i < 8; i++)
    {
        targetSpeeds[i] = atoi(argv[i + 1]);
        increments[i] = (float)(targetSpeeds[i] - currentSpeeds[i]) / (float)steps;
    }

    uint8_t dataSize = 16;
    uint8_t checksum = 0;
    uint8_t buffer[22];

    buffer[0] = 36;
    buffer[1] = 77;
    buffer[2] = 60;
    buffer[3] = dataSize;
    buffer[4] = 214;

    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);

    for (int step = 0; step < steps; step++) {
        checksum = dataSize ^ 214;

        for (int i = 0; i < 8; i++) {
            currentSpeeds[i] += increments[i];
            int16_t speedToSend = (int16_t)currentSpeeds[i];  // Convert to integer for sending
            memcpy(&buffer[5 + (i * 2)], &speedToSend, sizeof(speedToSend));

            for (size_t j = 0; j < sizeof(speedToSend); j++) {
                checksum ^= buffer[5 + (i * 2) + j];
            }
        }

        memcpy(&buffer[21], &checksum, sizeof(checksum));
        sendSerialData(buffer, sizeof(buffer));

        // Calculate elapsed time
        while (getElapsedTime(&start) < (step + 1) * sleepInterval) {
            // Busy-waiting until 100ms passes for this step
        }
    }

    // Send the final target speeds
    checksum = dataSize ^ 214;
    for (int i = 0; i < 8; i++) {
        memcpy(&buffer[5 + (i * 2)], &targetSpeeds[i], sizeof(targetSpeeds[i]));

        for (size_t j = 0; j < sizeof(targetSpeeds[i]); j++) {
            checksum ^= buffer[5 + (i * 2) + j];
        }
    }
    memcpy(&buffer[21], &checksum, sizeof(checksum));
    sendSerialData(buffer, sizeof(buffer));

    /* Stop the motors */
    int16_t stopSpeed = 1000;
    for (int i = 0; i < 8; i++) {
        currentSpeeds[i] = stopSpeed;
    }

    // Stop motors for an additional second
    clock_gettime(CLOCK_MONOTONIC, &start);
    while (getElapsedTime(&start) < 1.0) {
        checksum = dataSize ^ 214;

        for (int j = 0; j < 8; j++) {
            memcpy(&buffer[5 + (j * 2)], &stopSpeed, sizeof(stopSpeed));
            for (size_t k = 0; k < sizeof(stopSpeed); k++) {
                checksum ^= buffer[5 + (j * 2) + k];
            }
        }

        memcpy(&buffer[21], &checksum, sizeof(checksum));
        sendSerialData(buffer, sizeof(buffer));
        usleep(100000);  // 100ms to stop motors safely
    }

    return 0;
}

