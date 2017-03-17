#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "circularBuffer.h"

CircularBuff::CircularBuff()
{
    readIndex = 0;
    writeIndex = 0;
    isFull = false;
    isCommandReady = false;
    
    for(int i = 0; i < CIRC_BUFFER_SIZE; i++)
    {
        data[i] = '\0';
    }
}

void CircularBuff::setCommandReady(bool newIsCommandReady)
{
    isCommandReady = newIsCommandReady;
}


bool CircularBuff::getCommandReady()
{
    return isCommandReady;
}

//Function to put a character into the circular buffer
returnCode_t CircularBuff::putChar(const char dataIn)
{
    if(readIndex == writeIndex && isFull)
    {
        return BUFFER_OVERFLOW_ERROR;
    }
    
    data[writeIndex] = dataIn;
    writeIndex += 1; // increment index
    writeIndex &= CIRC_BUFFER_SIZE - 1; // efficient way to wrap index
    
    if(readIndex == writeIndex)
    {
        isFull == true; //buffer is filled, more writes will overwrite unread data
    }
    
    return NO_ERR;
}

//Function to get a character from the circular buffer
returnCode_t CircularBuff::getChar(char *dataOut)
{
    if(readIndex == writeIndex && !isFull)
    {
        return BUFFER_EMPTY_ERROR;
    }
    
    *dataOut = data[readIndex];
    readIndex += 1; // increment index
    readIndex &= CIRC_BUFFER_SIZE - 1; // efficient way to wrap index
    
    isFull = false; //we just read something so it can't be full anymore
    
    return NO_ERR;
}

//Function to read a command
// - reads command string from the circular buffer
returnCode_t CircularBuff::readCommand(char *commandString)
{
    //Check whether command is ready
    if(!isCommandReady)
    {
        return NO_COMMAND_READY_ERROR;
    }
    
    //Reset commandString
    for (int i = 0; i < CIRC_BUFFER_SIZE; i++)
    {
        commandString[i] = '\0';
    }
    
    //Read full command into commandString
    char tempChar = '\0';
    uint8_t commandStringLength = 0;
    getChar(&tempChar);
    
    while(tempChar != '\n' && tempChar != '\r')
    {
        commandString[commandStringLength] = tempChar; //add char to string        
        commandStringLength++;
        
        getChar(&tempChar); //get next char
    }
    
    //TODO potential bug - if the more characters have been read, incrementing writeIndex, during the command read
    if(readIndex == writeIndex && !isFull)
    {
        isCommandReady = false; //we've emptied the buffer
    }
    
    return NO_ERR;
}
