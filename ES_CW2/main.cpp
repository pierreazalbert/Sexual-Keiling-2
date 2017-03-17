#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "parser.h"
#include "circularBuffer.h"

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
const int8_t lead = 2;  //2 for forwards, -2 for backwards

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

InterruptIn CHA_interrupt(CHA);
InterruptIn CHB_interrupt(CHB);

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

CircularBuff circBuff; //construct circular buffer object

//Threads
Thread checkForAndProcessCommandsThread;

enum motorMode_t 
{
    STOP, //stop motor
    BASIC, //spin at full speed
    BACKWARDS, //basic but backwards
    ROTATE //rotate for set number of turns
};

motorMode_t motorMode = BACKWARDS;

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
    I1_interrupt.disable_irq();
    I2_interrupt.disable_irq();
    I3_interrupt.disable_irq();
    
    //Read character and put into buffer
    char charIn = pc.getc();
    circBuff.putChar(charIn);
    
    //If character is endline, set isCommandReady
    if(charIn == '\n' || charIn == '\r')
    {
        circBuff.setCommandReady(true);
    }
    
    I1_interrupt.enable_irq();
    I2_interrupt.enable_irq();
    I3_interrupt.enable_irq();
}

//Check Rotation

//Set Rotation
void rotateFullSpeed_basic()
{
    intState = readRotorState();
    motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
}

void rotateFullSpeed_backwards()
{
    intState = readRotorState();
    motorOut((intState-orState-lead+6)%6); //+6 to make sure the remainder is positive
}

//Set number of turns
// - count rotations
void rotateTurns()
{
    if(turnCount < 6*targetTurns)
    {
        intState = readRotorState();
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        turnCount++;
    }
    else
    {
        turnCount = 0;
        motorMode = STOP;
    }
}

//Control Motor Speed
// - calculate current speed
// - adjust speed to reach target speed
void rotateSpeed()
{

}

void rotorStateChange_isr()
{
    //disable interrupts
    I1_interrupt.disable_irq();
    I2_interrupt.disable_irq();
    I3_interrupt.disable_irq();
    
    switch(motorMode)
    {
        case STOP:
            break;
        
        case BASIC:
            rotateFullSpeed_basic();
            break;
            
        case BACKWARDS:
            rotateFullSpeed_backwards();
            break;
            
        case ROTATE:
            rotateTurns();
            break;
            
        default:
            motorMode = STOP;
            break;
    }
    
    //enable interrupts
    I1_interrupt.enable_irq();
    I2_interrupt.enable_irq();
    I3_interrupt.enable_irq();
}


//Function to read, parse and execute a command
returnCode_t readAndProcessCommand()
{
    char commandString[CIRC_BUFFER_SIZE];

    returnCode_t readReturn = circBuff.readCommand(commandString);
    if(readReturn == NO_ERR)
    {
        pc.printf("%s\n\r", commandString); //echo command
        
        if(commandString[0] == 'R' || commandString[0] == 'V')
        {
            // ***************************************** ROTATION(/SPEED) *****************************************
            dataSpeedOrRotateCommand_t dataSpeedOrRotateCommand;
            returnCode_t parseReturn = parseRotateOrSpeed(commandString, &dataSpeedOrRotateCommand);
            
            if(parseReturn != NO_ERR)
            {
                pc.printf("Error: parsing \"%s\" returned code %i \n\r\tcheck return_codes.h for return code details\n\r", commandString, parseReturn);
                return parseReturn;
            }
            if(dataSpeedOrRotateCommand.doRotate && dataSpeedOrRotateCommand.doSpeed)
            {
                pc.printf("Rotate %3.2f rotations at %3.2f rotations/sec\n\r", dataSpeedOrRotateCommand.rotateParam, dataSpeedOrRotateCommand.speedParam);
                //TODO do the command/find a way to return this to main
            }
            else if(dataSpeedOrRotateCommand.doRotate)
            {
                pc.printf("Rotate %3.2f rotations\n\r", dataSpeedOrRotateCommand.rotateParam);
                //TODO do the command/find a way to return this to main
                motorMode = ROTATE;
                rotateTurns();
            }
            else if(dataSpeedOrRotateCommand.doSpeed)
            {
                pc.printf("Spin at %3.2f rotations/sec\n\r", dataSpeedOrRotateCommand.speedParam);
                
                if(dataSpeedOrRotateCommand.speedParam > 0)
                {
                    motorMode = BASIC;
                    rotateFullSpeed_basic();
                }
                else if(dataSpeedOrRotateCommand.speedParam == 0)
                {
                    motorMode = STOP;
                }
                else //(dataSpeedOrRotateCommand.speedParam < 0)
                {
                    motorMode = BACKWARDS;
                    rotateFullSpeed_backwards();
                }
                
            }
            else
            {
                pc.printf("Warning: command returned NO_ERR, but didn't set doRotate or doSpeed\n\r");
            }
        }
        else if(commandString[0] == 'T')
        {
            // ***************************************** TUNE *****************************************
            dataTuneCommand_t dataTuneCommand;
            returnCode_t parseReturn = parseTune(commandString, &dataTuneCommand);
            
            if(parseReturn != NO_ERR)
            {
                pc.printf("Error: parsing \"%s\" returned code %i \n\r\tcheck return_codes.h for return code details\n\r", commandString, parseReturn);
                return parseReturn;
            }
            else
            {
                for(uint8_t seqIndex = 0; seqIndex < dataTuneCommand.seqLength; seqIndex++)
                {
                    pc.printf("Note %i: Tune %c %i for %i sec\n\r", seqIndex+1, dataTuneCommand.noteSeq[seqIndex].pitch, dataTuneCommand.noteSeq[seqIndex].pitchMod, dataTuneCommand.noteSeq[seqIndex].duration);
                }
                
                //TODO do the command/find a way to return this to main
            }
        }
        else
        {
            pc.printf("Unrecognised Command: %c\n\r", commandString[0]);
            return UNRECOGNISED_COMMAND_ERROR;
        }
    }
    else if(readReturn == NO_COMMAND_READY_ERROR)
    {
        return NO_COMMAND_READY_ERROR;
    }
    else
    {
        pc.printf("Error: reading command returned code %i \n\r\tcheck return_codes.h for return code details\n\r", commandString, readReturn);
        return readReturn;
    }
    return NO_ERR;
}

//Thread that will run indefinitely, reading and processing commands
void checkForAndProcessCommands_thread()
{
    while(1)
    {
        if(circBuff.getCommandReady())
        {           
            readAndProcessCommand();
        }
    }
}

//Main
int main()
{
    int8_t orState = 0;    //Rotot offset at motor state 0

    //Initialise the serial port
    pc.baud(115200);
    pc.printf("Hello\n\r");
    pc.attach(&serialRx_isr, Serial::RxIrq);
   
    I1_interrupt.rise(&rotorStateChange_isr);
    I2_interrupt.rise(&rotorStateChange_isr);
    I3_interrupt.rise(&rotorStateChange_isr);
    I1_interrupt.fall(&rotorStateChange_isr);
    I2_interrupt.fall(&rotorStateChange_isr);
    I3_interrupt.fall(&rotorStateChange_isr);

    
    //Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    //orState is subtracted from future rotor state inputs to align rotor and motor states

    
    checkForAndProcessCommandsThread.start(checkForAndProcessCommands_thread);

}

