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

int main(int argc, char *argv[])
{
    int i;

    if (argc != 10)
    {
        printf("%s","Wrong number of arguments\n");
        exit (1);
    }
    i = 1;
    while (i <= 8)
    {
        if (atoi(argv[i]) < 1000 || atoi(argv[i]) > 1960)
        {
            printf("%s","One or more thrusts are invalid\n");
            exit(1);
        }    
        i++;
    }
    if (atoi(argv[9]) < 0 || atoi(argv[9]) > 16)
    {
        printf("%s","Time parameter is invalid\n");
        exit(1);       
    }
    time_t  start_time = time(NULL);
    int     duration = atoi(argv[9]);

      // Set propeller speeds
    int16_t motor1Speed = atoi(argv[1]);
    int16_t motor2Speed = atoi(argv[2]);
    int16_t motor3Speed = atoi(argv[3]);
    int16_t motor4Speed = atoi(argv[4]);
    int16_t motor5Speed = atoi(argv[5]);
    int16_t motor6Speed = atoi(argv[6]);
    int16_t motor7Speed = atoi(argv[7]);
    int16_t motor8Speed = atoi(argv[8]);

    uint8_t dataSize = 16;
    uint8_t checksum = 0;

    // Create buffer for the serial data
    uint8_t buffer[22];

    buffer[0] = 36;
    buffer[1] = 77;
    buffer[2] = 60;
    buffer[3] = dataSize;
    buffer[4] = 214;

    checksum ^= dataSize;
    checksum ^= 214;

    memcpy(&buffer[5], &motor1Speed, sizeof(motor1Speed));
    for (size_t i = 0; i < sizeof(motor1Speed); i++) {
        checksum ^= buffer[i + 5];
    }

    memcpy(&buffer[7], &motor2Speed, sizeof(motor2Speed));
    for (size_t i = 0; i < sizeof(motor2Speed); i++) {
        checksum ^= buffer[i + 7];
    }

    memcpy(&buffer[9], &motor3Speed, sizeof(motor3Speed));
    for (size_t i = 0; i < sizeof(motor3Speed); i++) {
        checksum ^= buffer[i + 9];
    }

    memcpy(&buffer[11], &motor4Speed, sizeof(motor4Speed));
    for (size_t i = 0; i < sizeof(motor4Speed); i++) {
        checksum ^= buffer[i + 11];
    }

    memcpy(&buffer[13], &motor5Speed, sizeof(motor5Speed));
    for (size_t i = 0; i < sizeof(motor5Speed); i++) {
        checksum ^= buffer[i + 13];
    }

    memcpy(&buffer[15], &motor6Speed, sizeof(motor6Speed));
    for (size_t i = 0; i < sizeof(motor6Speed); i++) {
        checksum ^= buffer[i + 15];
    }

    memcpy(&buffer[17], &motor7Speed, sizeof(motor7Speed));
    for (size_t i = 0; i < sizeof(motor7Speed); i++) {
        checksum ^= buffer[i + 17];
    }

    memcpy(&buffer[19], &motor8Speed, sizeof(motor8Speed));
    for (size_t i = 0; i < sizeof(motor8Speed); i++) {
        checksum ^= buffer[i + 19];
    }

    memcpy(&buffer[21], &checksum, sizeof(checksum));

    while (difftime(time(NULL), start_time) < duration) {
        // Send the serial data
        sendSerialData(buffer, sizeof(buffer));
        //printf("hello");
        //printf("%s", buffer);
        usleep(100000);
    }

/*Stop the motors by wriring zeros and dending again*/
    checksum = 0;
    checksum ^= dataSize;
    checksum ^= 214;
    motor1Speed = 1000;
    motor2Speed = 1000;
    motor3Speed = 1000;
    motor4Speed = 1000;
    motor5Speed = 1000;
    motor6Speed = 1000;
    motor7Speed = 1000;
    motor8Speed = 1000;
    memcpy(&buffer[5], &motor1Speed, sizeof(motor1Speed));
    for (size_t i = 0; i < sizeof(motor1Speed); i++) {
        checksum ^= buffer[i + 5];
    }

    memcpy(&buffer[7], &motor2Speed, sizeof(motor2Speed));
    for (size_t i = 0; i < sizeof(motor2Speed); i++) {
        checksum ^= buffer[i + 7];
    }

    memcpy(&buffer[9], &motor3Speed, sizeof(motor3Speed));
    for (size_t i = 0; i < sizeof(motor3Speed); i++) {
        checksum ^= buffer[i + 9];
    }

    memcpy(&buffer[11], &motor4Speed, sizeof(motor4Speed));
    for (size_t i = 0; i < sizeof(motor4Speed); i++) {
        checksum ^= buffer[i + 11];
    }

    memcpy(&buffer[13], &motor5Speed, sizeof(motor5Speed));
    for (size_t i = 0; i < sizeof(motor5Speed); i++) {
        checksum ^= buffer[i + 13];
    }

    memcpy(&buffer[15], &motor6Speed, sizeof(motor6Speed));
    for (size_t i = 0; i < sizeof(motor6Speed); i++) {
        checksum ^= buffer[i + 15];
    }

    memcpy(&buffer[17], &motor7Speed, sizeof(motor7Speed));
    for (size_t i = 0; i < sizeof(motor7Speed); i++) {
        checksum ^= buffer[i + 17];
    }

    memcpy(&buffer[19], &motor8Speed, sizeof(motor8Speed));
    for (size_t i = 0; i < sizeof(motor8Speed); i++) {
        checksum ^= buffer[i + 19];
    }

    memcpy(&buffer[21], &checksum, sizeof(checksum));

    i = 0;
    while (i < 10) {
        // Send the serial data
        sendSerialData(buffer, sizeof(buffer));
        //printf("hello");
        //printf("%s", buffer);
        usleep(100000);
        i++;
    }

    return (0);
}
