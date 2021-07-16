#include <Arduino.h>
#include "MATLABcommunicate.h"
#include <array>


bool handshake()
{
    char ready{}; // holds the handshake character
    if(Serial.available() > 0)
    {
        Serial.readBytes(&ready, 1);
        if(ready == 'r')
        {
        Serial.print('y');
        return true;
        }
    
    }
    return false;
}


std::array<double, Settings::numValues> getData()
{
    std::array<double, Settings::numValues> data{};
    char buffer[500]{};
    char sep{','};
    char rc{};
    std::size_t index{0};
    byte buffIndex{0};

    // works best to just grab the whole message and dump into a char array
    while(Serial.available() > 0)
    {
        rc = Serial.read();
        buffer[buffIndex] = rc;
        ++buffIndex;
    }
    buffer[buffIndex] = '\0';
    
    // then parse it out
    char holder[50];
    byte holderIndex{0};

    for(char unit : buffer)
    {
        //Serial.println(unit == sep);
        if(unit != sep  && unit != '\n')
        {
            holder[holderIndex] = unit;
            ++holderIndex;
        }
        else
        {
            holder[holderIndex] = '\0';
            holderIndex = 0;
            data[index] = atof(holder);
            ++index;
        }
        
    }
    return data;

}