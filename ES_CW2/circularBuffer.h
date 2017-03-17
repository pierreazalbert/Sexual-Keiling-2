#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

//defines
#define CIRC_BUFFER_SIZE 64 //maximum command length is 49+endline (longest tune command)(should be 2^n for efficient wrapping to work)

class CircularBuff
{
    private:
    
    volatile uint8_t readIndex;
    volatile uint8_t writeIndex;
    volatile char data[CIRC_BUFFER_SIZE];
    volatile bool isFull;
    volatile bool isCommandReady;
    
    returnCode_t getChar(char *dataOut);

    public:
    
    CircularBuff(); // constructor
    void setCommandReady(bool newIsCommandReady);
    bool getCommandReady();
    returnCode_t putChar(const char dataIn);
    returnCode_t readCommand(char *commandString);
};



#endif