#include "RoboteqSerial.hpp"

class Timer
{
public:
    Timer()
    {
        time = millis();
    }

    ~Timer()
    {
        Serial.println(millis() - time);
    }

private:
    unsigned long time;
};

RoboteqSerial::RoboteqSerial(Stream &stream)
    : _serial(stream)
{
}

int RoboteqSerial::sendQuery(const char *message)
{
    this->_serial.write(message, strlen(message));
    this->_serial.flush();

    return 0;
}

int RoboteqSerial::queryFaultFlags()
{
    this->sendQuery("?FF\r\n");

    int faultflag;
    if ((faultflag = this->readQuery("FF=")) > -1)
    {
        return faultflag;
    }

    return -1;
}

int RoboteqSerial::queryVoltage()
{
    this->sendQuery("?V 2\r\n");

    int voltage;
    if ((voltage = this->readQuery("V=")) > -1)
    {
        return voltage;
    }

    return -1;
}

int RoboteqSerial::readQuery(const char *message)
{
    String inputString;
    unsigned long startTime = millis();
    while (millis() - startTime < timeout && _serial.available())
    {
        inputString = _serial.readStringUntil('\r');

        if (inputString.startsWith(message))
        {
            return inputString.substring(inputString.indexOf("=") + 1).toInt();
            break;
        }
    }
    return -1;
}
