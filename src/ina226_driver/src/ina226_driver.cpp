#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>

#define INA226_ADDR 0x40               // I2C address for INA226
#define INA226_BUS_VOLTAGE 0x02        // Register for Bus Voltage
#define INA226_CURRENT 0x04             // Register for Current
#define INA226_POWER 0x03               // Register for Power

// Function to read 16-bit data from a specific register
int read_register(int file, uint8_t reg);

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "ina226_driver");
    ros::NodeHandle nh;

    // Create a publisher for the voltage measurements
    ros::Publisher voltage_pub = nh.advertise<std_msgs::Float32>("ina226/voltage", 1000);

    int file;
    char *filename = "/dev/i2c-5";  // Adjust based on your setup

    // Open I2C bus
    if ((file = open(filename, O_RDWR)) < 0) {
        ROS_ERROR("Failed to open the i2c bus");
        return 1;
    }

    // Set the I2C address
    if (ioctl(file, I2C_SLAVE, INA226_ADDR) < 0) {
        ROS_ERROR("Failed to acquire bus access and/or talk to slave");
        close(file);
        return 1;
    }

    ros::Rate loop_rate(1); // 1 Hz

    while (ros::ok()) {
        // Read Bus Voltage
        int voltage_raw = read_register(file, INA226_BUS_VOLTAGE);
        if (voltage_raw < 0) {
            close(file);
            return 1;
        }

        // Convert to voltage (INA226 gives voltage in mV)
        float voltage = voltage_raw * 0.001;  // Convert mV to V

        // Create a Float32 message and publish the voltage
        std_msgs::Float32 voltage_msg;
        voltage_msg.data = voltage * 1.25;
        voltage_pub.publish(voltage_msg);

        // Log the measured voltage
        //ROS_INFO("Measured Voltage: %.3f V", voltage);

        // Sleep for a second before the next reading
        loop_rate.sleep();
    }

    // Close the I2C bus
    close(file);
    return 0;
}

// Function to read 16-bit data from a specific register
int read_register(int file, uint8_t reg) {
    uint8_t buf[2];
    if (write(file, &reg, 1) != 1) {
        ROS_ERROR("Failed to write to the i2c bus");
        return -1;
    }
    if (read(file, buf, 2) != 2) {
        ROS_ERROR("Failed to read from the i2c bus");
        return -1;
    }
    return (buf[0] << 8) | buf[1];  // Combine high and low bytes
}

