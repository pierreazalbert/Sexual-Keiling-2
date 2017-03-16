#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "parser.h"

//Photointerrupter input pins
#define I1pin D2
#define I2pin D11
#define I3pin D12

//Incremental encoder input pins
#define CHA   D7
#define CHB   D8

//Motor Drive output pins   //Mask in output byte
#define L1Lpin D4           //0x01
#define L1Hpin D5           //0x02
#define L2Lpin D3           //0x04
#define L2Hpin D6           //0x08
#define L3Lpin D9           //0x10
#define L3Hpin D10          //0x20

//Mapping from sequential drive states to motor phase outputs
/*
State   L1  L2  L3
0       H   -   L
1       -   H   L
2       L   H   -
3       L   -   H
4       -   L   H
5       H   L   -
6       -   -   -
7       -   -   -
*/
//Drive state to output table
const int8_t driveTable[] = {0x12,0x18,0x09,0x21,0x24,0x06,0x00,0x00};

//Mapping from interrupter inputs to sequential rotor states. 0x00 and 0x07 are not valid
const int8_t stateMap[] = {0x07,0x05,0x03,0x04,0x01,0x00,0x02,0x07};
//const int8_t stateMap[] = {0x07,0x01,0x03,0x02,0x05,0x00,0x04,0x07}; //Alternative if phase order of input or drive is reversed

//Phase lead to make motor spin
const int8_t lead = -2;  //2 for forwards, -2 for backwards

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
// DigitalIn redundant as values can be read with IX_interrupt.read()
//DigitalIn I1(I1pin);
//DigitalIn I2(I2pin);
//DigitalIn I3(I3pin);
InterruptIn I1_interrupt(I1pin);
InterruptIn I2_interrupt(I2pin);
InterruptIn I3_interrupt(I3pin);

//Motor Drive outputs
DigitalOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
DigitalOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
DigitalOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Serial Port
RawSerial pc(SERIAL_TX, SERIAL_RX); // RTOS interrupts don't work with Serial, RawSerial is used instead

volatile uint8_t intState = 0;
uint8_t orState;

//Target parameters
#define targetTurns (4)
#define targetVel (0)

#define CIRC_BUFFER_SIZE 64 //maximum command length is 49+endline (longest tune command)
//Define Circular Buffer (should be 2^n for efficient wrapping to work)
typedef struct
{
    uint8_t readIndex;
    uint8_t writeIndex;
    char data[CIRC_BUFFER_SIZE];
    bool full;
    bool commandReady;
} circularBuffer_t;
volatile circularBuffer_t circularBuffer;

//Function to put a character into the circular buffer
returnCode_t circularBufferPut(const char dataIn)
{
    if(circularBuffer.readIndex == circularBuffer.writeIndex && circularBuffer.full)
    {
        return BUFFER_OVERFLOW_ERROR;
    }
    
    circularBuffer.data[circularBuffer.writeIndex] = dataIn;
    circularBuffer.writeIndex += 1; // increment index
    circularBuffer.writeIndex &= CIRC_BUFFER_SIZE - 1; // efficient way to wrap index
    
    if(circularBuffer.readIndex == circularBuffer.writeIndex)
    {
        circularBuffer.full == true; //buffer is filled, more writes will overwrite unread data
    }
    
    return NO_ERR;
}

//Function to get a character from the circular buffer
returnCode_t circularBufferGet(char *dataOut)
{
    if(circularBuffer.readIndex == circularBuffer.writeIndex && !circularBuffer.full)
    {
        return BUFFER_EMPTY_ERROR;
    }
    
    *dataOut = circularBuffer.data[circularBuffer.readIndex];
    circularBuffer.readIndex += 1; // increment index
    circularBuffer.readIndex &= CIRC_BUFFER_SIZE - 1; // efficient way to wrap index
    
    circularBuffer.full = false; //we just read something so it can't be full anymore
    
    return NO_ERR;
}

//
volatile int8_t turnCount = 0;

//Set a given drive state
void motorOut(int8_t driveState)
{

    //Lookup the output byte from the drive state.
    int8_t driveOut = driveTable[driveState & 0x07];

    //Turn off first
    if (~driveOut & 0x01) L1L = 0;
    if (~driveOut & 0x02) L1H = 1;
    if (~driveOut & 0x04) L2L = 0;
    if (~driveOut & 0x08) L2H = 1;
    if (~driveOut & 0x10) L3L = 0;
    if (~driveOut & 0x20) L3H = 1;

    //Then turn on
    if (driveOut & 0x01) L1L = 1;
    if (driveOut & 0x02) L1H = 0;
    if (driveOut & 0x04) L2L = 1;
    if (driveOut & 0x08) L2H = 0;
    if (driveOut & 0x10) L3L = 1;
    if (driveOut & 0x20) L3H = 0;
}

//Convert photointerrupter inputs to a rotor state
//Changed to read InterruptIn objects
inline int8_t readRotorState()
{
    return stateMap[I1_interrupt.read() + 2*I2_interrupt.read() + 4*I3_interrupt.read()];
}

//Basic synchronisation routine
int8_t motorHome()
{
    //Put the motor in drive state 0 and wait for it to stabilise
    motorOut(0);
    wait(1.0);

    //Get the rotor state
    return readRotorState();
}

//ISR activated when a character is received by serial port
// - reads character and puts it into the circular buffer
void serialRx_isr()
{
    //Read character and put into buffer
    char charIn = pc.getc();
    circularBufferPut(charIn);
    
    //If character is endline, set commandReady
    if(charIn == '\n' || charIn == '\r')
    {
        circularBuffer.commandReady = true;
    }
}

//Function to read a command
// - reads command string from the circular buffer
returnCode_t readCommand(char *commandString)
{
    //Intialise commandString
    for (int i = 0; i < CIRC_BUFFER_SIZE; i++)
    {
        commandString[i] = '\0';
    }
    
    //Check whether command is ready
    if(!circularBuffer.commandReady)
    {
        return NO_COMMAND_READY_ERROR;
    }
    
    //Read full command into commandString
    char tempChar = '\0';
    uint8_t commandStringLength = 0;
    circularBufferGet(&tempChar);
    
    while(tempChar != '\n' && tempChar != '\r')
    {
        commandString[commandStringLength] = tempChar; //add char to string        
        commandStringLength++;
        
        circularBufferGet(&tempChar); //get next char
    }
    //pc.printf("%s, length = %i\n", commandString, commandStringLength); //echo commandString
    
    //TODO potential bug - if the more characters have been read, incrementing writeIndex, during the command read
    if(circularBuffer.readIndex == circularBuffer.writeIndex && !circularBuffer.full)
    {
        circularBuffer.commandReady = false; //we've emptied the buffer
    }
    
    return NO_ERR;
}

//Check Rotation

//Set Rotation

//Set number of turns
// - count rotations
void rotateTurns()
{
    //TODO set this up to activate the irq version
}

//Control Motor Speed
// - calculate current speed
// - adjust speed to reach target speed
void rotateSpeed()
{

}

void rotorStateChange_basic_isr()
{
    //disable interrupts
    I1_interrupt.disable_irq();
    I2_interrupt.disable_irq();
    I3_interrupt.disable_irq();

    intState = readRotorState(); //try with interrupt.read()
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
    
    //enable interrupts
    I1_interrupt.enable_irq();
    I2_interrupt.enable_irq();
    I3_interrupt.enable_irq();
}

void rotorStateChange_turns_isr()
{
    //disable interrupts
    I1_interrupt.disable_irq();
    I2_interrupt.disable_irq();
    I3_interrupt.disable_irq();

    if(turnCount < 6*targetTurns)
    {
        intState = readRotorState(); //try with interrupt.read()
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        turnCount++;
    }
    
    //enable interrupts
    I1_interrupt.enable_irq();
    I2_interrupt.enable_irq();
    I3_interrupt.enable_irq();
}


//Main
int main()
{
    int8_t orState = 0;    //Rotot offset at motor state 0

    //Initialise the serial port
//    Serial pc(SERIAL_TX, SERIAL_RX);
    pc.baud(115200);
    pc.printf("Hello\n\r");
    pc.attach(&serialRx_isr, Serial::RxIrq);
//    int8_t intState = 0;
//    int8_t intStateOld = 0;

    //initialise circular buffer;
    circularBuffer.readIndex = 0;
    circularBuffer.writeIndex = 0;
    circularBuffer.full = false;
    for (int i = 0; i < CIRC_BUFFER_SIZE; i++)
    {
        circularBuffer.data[i] = '\0';
    }
    
    //TODO change mode to an enum/get rid of it completely
    const uint8_t mode = 0; //0 = basic, 1 = turns
    if(mode == 0)
    {
        // BASIC
        I1_interrupt.rise(&rotorStateChange_basic_isr);
        I2_interrupt.rise(&rotorStateChange_basic_isr);
        I3_interrupt.rise(&rotorStateChange_basic_isr);
        I1_interrupt.fall(&rotorStateChange_basic_isr);
        I2_interrupt.fall(&rotorStateChange_basic_isr);
        I3_interrupt.fall(&rotorStateChange_basic_isr);
    }
    else if(mode == 1)
    {
        // TURNS
        I1_interrupt.rise(&rotorStateChange_turns_isr);
        I2_interrupt.rise(&rotorStateChange_turns_isr);
        I3_interrupt.rise(&rotorStateChange_turns_isr);
        I1_interrupt.fall(&rotorStateChange_turns_isr);
        I2_interrupt.fall(&rotorStateChange_turns_isr);
        I3_interrupt.fall(&rotorStateChange_turns_isr);
    }
    else
    {
        
    }
    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    char commandString[CIRC_BUFFER_SIZE];
    for (int i = 0; i < CIRC_BUFFER_SIZE; i++)
    {
        commandString[i] = '\0';
    }

    while(1)
    {
        if(readCommand(commandString) == NO_ERR)
        {
            pc.printf("%s\n\r", commandString);
            parseCommand(commandString); //TODO actually do something about any errors returned
        }
        else
        {
            //TODO actually do something about any errors returned
        }
    }
}

