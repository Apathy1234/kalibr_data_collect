#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/String.h"
#include "serial_mcu.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <string>
#include <vector>
using namespace std;

#define SERIAL_PORT "/dev/ttyUSB0"
#define SERIAL_BAUDRATE 57600

static uint8_t dataRevState = 0; 
static uint8_t rxBuffer[50];
static uint16_t dataLen = 0, dataCnt = 0;
static sensor imuUsed;

serial::Serial serUsed;



void enumerate_ports(void)
{
    vector<serial::PortInfo> devices_found = serial::list_ports();

    vector<serial::PortInfo>::iterator iter = devices_found.begin();

    while (iter != devices_found.end())
    {
        serial::PortInfo device = *iter++;

        ROS_INFO("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str());
    }
}

bool open_serial(void)
{
    try
    {
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serUsed.setPort(SERIAL_PORT);
        serUsed.setBaudrate(SERIAL_BAUDRATE);
        serUsed.setTimeout(to);
        serUsed.open();
    }
    catch (serial::IOException &e)
    {
        ROS_ERROR_STREAM("Unable to open port!");
        return false;
    }

    if (serUsed.isOpen())
    {
        ROS_INFO_STREAM("Serial port initialized!");
    }
    else
    {
        ROS_ERROR_STREAM("Sad!");
        return false;
    }

    return true;
}

void serial_write_data(uint8_t *dataToSend, size_t length)
{
    serUsed.write(dataToSend, length);
    serUsed.flushOutput();
}

bool serial_read_data(uint8_t *result, size_t length)
{
    bool yes = false;
    size_t byteRead = serUsed.read(result, length);
    yes = byteRead >0 ? true:false;
    //serUsed.flushInput();
    return yes;
}

bool serial_read_byte(uint8_t *result)
{
    bool yes = false;
    size_t byteRead = serUsed.read(result, 1);
    yes = byteRead >0 ? true:false;
    //serUsed.flushInput();
    return yes;
}

bool message_parse(uint8_t data)
{
    if ( dataRevState == 0 && data == 0xAA )
    {
        dataRevState = 1;
        rxBuffer[0] = data;
        // ROS_INFO_STREAM("match 1!");
    }
    else if ( dataRevState == 1 && data == 0xAA ) 
    {
        dataRevState = 2;
        rxBuffer[1] = data;
        // ROS_INFO_STREAM("match 2!");
    }
    else if ( dataRevState == 2 && data == 0x02 )
    {
        dataRevState = 3;
        rxBuffer[2] = data;
        // ROS_INFO_STREAM("match 3!");
    }
    else if ( dataRevState == 3 && data < 50 )
    {
        dataRevState = 4;
        rxBuffer[3] = data;
        dataLen = data;
        dataCnt = 0;
        //ROS_INFO_STREAM("match 4!");
    }
    else if ( dataRevState == 4 && dataLen > 0 )
    {
        dataLen--;
        rxBuffer[4 + dataCnt++] = data;
        if ( dataLen == 0)
        {
            dataRevState = 5;
        }
        //ROS_INFO_STREAM("match 5!");
    }
    else if (dataRevState == 5 )
    {
        dataRevState = 0;
        rxBuffer[4 + dataCnt] = data;
        //ROS_INFO_STREAM("I have get the data!");
        return true;
    }
    else 
    {
        dataRevState = 0;
        //ROS_INFO_STREAM("failed");
    }
    return false;
}

void message_analyze(void)
{
    imuUsed.ax = (int16_t)((rxBuffer[4] << 8) | rxBuffer[5])/1000.0f;
    imuUsed.ay = (int16_t)((rxBuffer[6] << 8) | rxBuffer[7])/1000.0f;
    imuUsed.az = (int16_t)((rxBuffer[8] << 8) | rxBuffer[9])/1000.0f;
    imuUsed.gx = (int16_t)((rxBuffer[10] << 8) | rxBuffer[11])/1000.0f;
    imuUsed.gy = (int16_t)((rxBuffer[12] << 8) | rxBuffer[13])/1000.0f;
    imuUsed.gz = (int16_t)((rxBuffer[14] << 8) | rxBuffer[15])/1000.0f;
    //cout << "ax:" << imuUsed.ax << ", ay:"<<imuUsed.ay <<", az:" << imuUsed.az <<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_mcu");
    ros::NodeHandle n;

    ros::Publisher imuPub = n.advertise<sensor_msgs::Imu>("imu0", 1);
    static uint8_t data = 0;
    static bool dataUpdate = false;
    
    sensor_msgs::Imu imuData;

    open_serial();

    //ros::Rate loopRate(1000);
    while (ros::ok())
    {
        if( serial_read_byte(&data) )
        {
            dataUpdate = message_parse(data);
        }
        if ( dataUpdate )
        {
            message_analyze();
            imuData.header.stamp = ros::Time::now();
            imuData.header.frame_id = "global";
            imuData.angular_velocity.x = imuUsed.gx;
            imuData.angular_velocity.y = imuUsed.gy;
            imuData.angular_velocity.z = imuUsed.gz;
            imuData.linear_acceleration.x = imuUsed.ax;
            imuData.linear_acceleration.y = imuUsed.ay;
            imuData.linear_acceleration.z = imuUsed.az;
            imuPub.publish(imuData);
            dataUpdate = false;
        }
     

        ros::spinOnce();
        //loopRate.sleep();
    }
}