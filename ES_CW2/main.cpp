/* 
    Getting started with threading
    • Starter code continuously checks photointerrupters
    • Convert to interrupts to allow other tasks to take place
    
    Define which tasks are taking place
    • Motor rotation?
    • Speed control?
    • Serial interface?
    
    What is their minimum initiation interval?
    • What are their dependencies (what can block their execution)?
    • What is their priority?
    • What data must be shared?
*/

// If doing the rotations and then trying to do fixed speed, then memory runs out
// doing it the other way around, no memory issues
// We use 'new' to allocate memory to the threads, could check if Thread.terminate()
// clears the memory, or if this is something we have to do manually

// RTX error maybe caused by going too fast - seems to occur when VX is used when X>=10


#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "parser.h"
#include "circular_buffer.h"
#include "tune.h"

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

#define NUM_STATES_LO_RES 6
#define NUM_RELATIVE_POSITION_STATES 468
#define SPEED_MEASURE_TIMEOUT_MS 100

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
volatile int8_t lead = 1;  //positive for ACW, negative for CW

//Status LED
DigitalOut led1(LED1);

//Photointerrupter inputs
// DigitalIn redundant as values can be read with IX_interrupt.read()
InterruptIn I1_interrupt(I1pin);
InterruptIn I2_interrupt(I2pin);
InterruptIn I3_interrupt(I3pin);

InterruptIn CHA_interrupt(CHA);
InterruptIn CHB_interrupt(CHB);

//Motor Drive outputs
PwmOut L1L(L1Lpin);
DigitalOut L1H(L1Hpin);
PwmOut L2L(L2Lpin);
DigitalOut L2H(L2Hpin);
PwmOut L3L(L3Lpin);
DigitalOut L3H(L3Hpin);

//Serial Port
RawSerial pc(SERIAL_TX, SERIAL_RX); // RTOS interrupts don't work with Serial, RawSerial is used instead
CircularBuff circBuff; //construct circular buffer object

volatile uint8_t intState = 0;
volatile uint8_t intStateOld = 0;
volatile uint8_t orState;

//Target parameters
volatile int target_turns = 0;
volatile float target_vel = 0.0f;

// Speed measurement
Timer vel_timer;
volatile float vel_timer_log = 1.0f;
volatile float meas_vel = 0.0f;

// Position measurement
volatile int turns = 0;

// Speed control
volatile float vel_period = 0.0f;

// PID controller coefficients
//Kc, Ti, Td, interval
#define PID_K (0.1f)
#define PID_T_I (0.0f)
#define PID_T_D (0.05f)
#define PID_RATE (0.1f)

typedef struct
{
    float K_p;
    float K_i;
    float K_d;
    
    float interval;
    
    float target;
    
    float error;
    float dError;
    float accError;
    
    float input_curr;  
    float input_prev;
    
    float output;
} PID_cust;    

void pid_init(PID_cust* controller, float K_p, float K_i, float K_d, float interval)
{
    controller->K_p = K_p;
    controller->K_i = K_i;
    controller->K_d = K_d;
    controller->interval = interval;
    
    controller->target = 0.0f;
    controller->error = 0.0f;
    controller->dError = 0.0f;
    controller->accError = 0.0f;
    controller->input_curr = 0.0f;
    controller->input_prev = 0.0f;
    controller->output = 0.0f;
    
    //pc.printf("pid initialised with parameters \n\r");
    //pc.printf("k_p %1.3f k_i %1.3f k_d %1.3f rate %1.3f \n\r", controller->K_p, controller->K_i, controller->K_d, controller->interval);
}

void pid_setTarget(PID_cust* controller, float target)
{
    controller->target = target;
}

void pid_setInput(PID_cust* controller, float input)
{
    if(input < 100.0f)
    {
        controller->input_curr = input;
    }
    else
    {
        controller->input_curr = controller->target;
    }
}

void pid_reset(PID_cust* controller)
{    
    controller->target = 0.0f;
    controller->error = 0.0f;
    controller->dError = 0.0f;
    controller->accError = 0.0f;
    controller->input_curr = 0.0f;
    controller->input_prev = 0.0f;
    controller->output = 0.0f;
}

float pid_run(PID_cust* controller)
{
    controller->error = controller->input_curr - controller->target;
 
    controller->accError += controller->error;
    
    controller->dError = (controller->input_curr - controller->input_prev) / controller->interval;
 
    controller->output = (controller->K_p*controller->error) - (controller->K_d*controller->dError); //+ (controller->K_i*controller->accError) 
 
    controller->input_prev = controller->input_curr;
    
    return controller->output;
}

PID_cust speedCTRL;

//Threads
Thread *speed_thread;
Thread *speed_controller_thread;
Thread *rotation_thread;
Thread *rotation_controller_thread;
Thread *play_tune_thread;

volatile bool isPlayingTune = false;
dataTuneCommand_t dataTuneCommand;


/* FUNCTION PROTOTYPES */

// Rotor General
int8_t motorHome();
inline int8_t readRotorState();
void motorOut(int8_t driveState);

// Measuring
void measureSpeed();

// Speed related
void fixedSpeed();
void fixedSpeedController();

// Rotation related
void fixedTurns();
void fixedTurnsController();

// Threading
void resetThreads();

// Serial Comms
void serialRx_isr();




//Main
int main()
{
    //Initialise the serial port
    pc.baud(115200);
    pc.printf("Hello\n\r");
    pc.attach(&serialRx_isr, Serial::RxIrq);
    
    I1_interrupt.rise(&measureSpeed);
    /*
    I1_interrupt.fall(&measureTurns);
    I2_interrupt.fall(&measureTurns);
    I3_interrupt.fall(&measureTurns);
    I1_interrupt.rise(&measureTurns);
    I2_interrupt.rise(&measureTurns);
    I3_interrupt.rise(&measureTurns);
    */
    
    // Speed controller
    pid_init(&speedCTRL, PID_K, PID_T_I, PID_T_D, PID_RATE);
    
    // allocate memory for threads                
    rotation_thread = new Thread(osPriorityNormal,400);
    //rotation_controller_thread = new Thread(osPriorityNormal,700);
    speed_thread = new Thread(osPriorityNormal,400);
    speed_controller_thread = new Thread(osPriorityNormal,400);
    play_tune_thread = new Thread(osPriorityNormal,400);
    
    
    // Run the motor synchronisation
    orState = motorHome();
    pc.printf("Rotor origin: %x\n\r",orState);
    // orState is subtracted from future rotor state inputs to align rotor and motor states
    intState = readRotorState();
    intStateOld = intState;

    // run forever, checking for input commands
    while(1)
    {
        if(circBuff.getCommandReady())
        {
            resetThreads();
            readAndProcessCommand();
        }
    }
    
}


// measure time between interrupts from I1
void measureSpeed()
{
    vel_timer_log = vel_timer.read();
    vel_timer.reset();
    vel_timer.start();
    
    meas_vel = 1.0f/vel_timer_log;
    //pc.printf("speed: %3.3f \n\r",meas_vel);
}

void playTune()
{    
    //pc.printf("play tune thread entered \n\r");
    
    while(1)
    {
        for(uint8_t noteIndex = 0; noteIndex < dataTuneCommand.seqLength; noteIndex++)
        {
            note_t *currentNote = &dataTuneCommand.noteSeq[noteIndex];
            float currentPeriod = (float)pitchToPeriod(currentNote->pitch); 
            int currentDuration = currentNote->duration; 
            //pc.printf("\tnote %i - pitch %i (%f) for %f sec\n\r", noteIndex+1, currentNote->pitch, currentPeriod, (float)currentDuration/BEATS_PER_SECOND);
    
            L1L.period_us(currentPeriod);
            L2L.period_us(currentPeriod);
            L3L.period_us(currentPeriod);
        
            Thread::wait(currentDuration*1000/BEATS_PER_SECOND);
        }
        //pc.printf("\trepeating tune \n\r");
    }
}


// Thread function for running at a fixed speed
void fixedSpeed()
{
    // structure is same as sample code, instead of a wait for pausing between 
    // each rotor state, Thread::wait is used instead
    
    while(1)
    {
        Thread::wait(1000.0f*vel_period);
        intState = readRotorState();
        //if (intState != intStateOld) {
            //intStateOld = intState;
        motorOut((intState-orState+lead+6)%6); //+6 to make sure the remainder is positive
        //}
        
        
        if(isPlayingTune)
        {
            // set duty cycle to 50%
            L1L.write(0.5f);
            L2L.write(0.5f);
            L3L.write(0.5f);
        }
    }    
}

// PID controller for the motor
void fixedSpeedController()
{
    // Controller takes the current speed and calculates the new speed towards
    // the target
    float speed_control;
    
    while(1)
    {
        //Update the process variable.
        pid_setInput(&speedCTRL, meas_vel);
        //Set the new output.
        speed_control = pid_run(&speedCTRL);
        
        /*
        pc.printf("input: %3.3f \n\r",speedCTRL.input_curr);
        pc.printf("target: %3.3f \n\r",speedCTRL.target);
        pc.printf("error: %3.3f \n\r",speedCTRL.error);
        pc.printf("speed_control: %3.6f \n\r",speed_control);
        pc.printf("period: %3.6f \n\r",vel_period);
        */
        if(abs(speed_control)<0.3f)
        {
            vel_period += 0.0005f*speed_control;
            vel_period = abs(vel_period);
        }
        Thread::wait(1000.0f*PID_RATE);
    }    
}

// Thread function for rotating to a number of turns
void fixedTurns()
{
    // structure is same as sample code, instead of a wait for pausing between 
    // each rotor state, Thread::wait is used instead
    //pc.printf("fixed turns thread entered \n\r");
    
    turns = 0;    
    intState = readRotorState();
    
    while(turns < target_turns)
    {
        //pc.printf("number of turns: %3.3f\n\r",turns);
        motorOut((intState-orState+lead+6)%6);
        turns++;
        
        if(lead > 0)
        {
            intState = (intState+7)%6; // intState + 1 + 6 ;)
        }
        else
        {
            intState = (intState+5)%6;
        }        
        
        Thread::wait(1000.0f*vel_period);
    }
}

                
// Thread function for rotating to a number of turns
void fixedTurnsController()
{
    // structure is same as sample code, instead of a wait for pausing between 
    // each rotor state, Thread::wait is used instead
    //pc.printf("fixed turns controller thread entered \n\r");
    float position_control;
    
    while(turns < target_turns)
    {
        // measured distance = turns
        
        //Update the process variable.
        //Set the new output.
        
        //vel_period = 4.0f*position_control/(target_turns*target_turns);
        
        Thread::wait(1000.0f*PID_RATE);
    }    
}


// check which threads are active or idle, and terminate them
void resetThreads()
{
    pc.printf("checking which threads are active... \n\r");
    
    pc.printf("speed_thread in state %d \n\r",speed_thread->get_state());
    if(speed_thread->get_state() != Thread::Deleted)
    {
        pc.printf("terminating speed_thread \n\r");
        speed_thread->terminate();
        //delete speed_thread;
    }
    
    pc.printf("speed_controller_thread in state %d \n\r",speed_controller_thread->get_state());
    if(speed_controller_thread->get_state() != Thread::Deleted)
    {
        pc.printf("terminating speed_controller_thread \n\r");
        speed_controller_thread->terminate();
        //delete speed_controller_thread;
    }
    
    pc.printf("rotation_thread in state %d \n\r",rotation_thread->get_state());
    if(rotation_thread->get_state() != Thread::Deleted)
    {
        pc.printf("terminating rotations_thread \n\r");
        rotation_thread->terminate();
        //delete rotation_thread;
    }
    /*
    pc.printf("rotation_controller_thread in state %d \n\r",rotation_controller_thread->get_state());
    if(rotation_controller_thread->get_state() != Thread::Deleted)
    {
        pc.printf("terminating rotation_control_thread \n\r");
        rotation_controller_thread->terminate();
        //delete rotation_controller_thread;
    }
    */
    pc.printf("play_tune_thread in state %d \n\r",play_tune_thread->get_state());
    if(play_tune_thread->get_state() != Thread::Deleted)
    {
        pc.printf("terminating play_tune_thread \n\r");
        play_tune_thread->terminate();
        //delete play_tune_thread;
    }
    
    // reset tune stuff
    isPlayingTune = false;
    L1L.write(1);
    L2L.write(1);
    L3L.write(1);    
}


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
                pc.printf("Rotate %3.2f turns at %3.2f revolutions per second\n\r", dataSpeedOrRotateCommand.rotateParam, dataSpeedOrRotateCommand.speedParam);
                
                target_turns = 6*dataSpeedOrRotateCommand.rotateParam;
                                
                if(target_turns < 0){
                    lead = -1;
                }
                else{
                    lead = 1;    
                }
                target_turns = abs(target_turns);
                                
                target_vel = dataSpeedOrRotateCommand.speedParam;
                vel_period = 1.0f/(6.0f * target_vel);
                
                // start motor thread
                rotation_thread->start(fixedTurns);
                
                // set pid controller and start thread to set it
                //pid_positionController.setSetPoint(target_turns);
                //rotation_controller_thread->start(fixedTurnsController);
                
            }
            else if(dataSpeedOrRotateCommand.doRotate)
            {
                pc.printf("Rotate %3.2f rotations\n\r", dataSpeedOrRotateCommand.rotateParam);
                
                target_turns = 6*dataSpeedOrRotateCommand.rotateParam;
                
                
                if(target_turns < 0){
                    lead = -1;
                }
                else{
                    lead = 1;    
                }
                target_turns = abs(target_turns);
                
                target_vel = 0.4f;
                vel_period = 1.0f/(6.0f * target_vel);
                
                // start motor thread
                rotation_thread->start(fixedTurns);
                
                // set pid controller and start thread to set it
                //pid_positionController.setSetPoint(target_turns);
                //rotation_controller_thread->start(fixedTurnsController);                
            }
            else if(dataSpeedOrRotateCommand.doSpeed)
            {
                pid_reset(&speedCTRL);
                
                target_vel = dataSpeedOrRotateCommand.speedParam;
                
                pc.printf("Spin at %3.2f rotations/sec\n\r", target_vel);
                
                if(target_vel < 0){
                    lead = -1;
                }
                else{
                    lead = 1;
                }
                
                target_vel = abs(target_vel);
                
                pid_setTarget(&speedCTRL, target_vel);
                
                vel_period = 1.0f/(6.0f * target_vel);
                
                // start motor speed thread
                speed_thread->start(fixedSpeed);                
                
                speed_controller_thread->start(fixedSpeedController);                
            }
            else
            {
                pc.printf("Warning: command returned NO_ERR, but didn't set doRotate or doSpeed\n\r");
            }
        }
        else if(commandString[0] == 'T')
        {
            // ***************************************** TUNE *****************************************
            returnCode_t parseReturn = parseTune(commandString, &dataTuneCommand);
                        
            if(parseReturn != NO_ERR)
            {
                pc.printf("Error: parsing \"%s\" returned code %i \n\r\tcheck return_codes.h for return code details\n\r", commandString, parseReturn);
                return parseReturn;
            }
            else if(dataTuneCommand.seqLength == 0)
            {
                pc.printf("Warning: No notes provided with T command\n\r");
            }
            else
            {
                // set parameters
                target_vel = 1.0f;
                vel_period = 1.0f/(6.0f * target_vel);
                isPlayingTune = true;
                
                //start threads
                speed_thread->start(fixedSpeed);
                play_tune_thread->start(playTune);
            }
        }
        // ************ TESTING COMMANDS ************
        else if (commandString[0] == 'H') // command tells which direction rotor is spinning
        {
            pc.printf(
            "'V' - Spin at a set speed e.g. 'V-10' to rotate in reverse at 10 rot/s\n\r"
            "'R' - Rotate a set number of rotations e.g. 'R5' to rotate 5 times\n\r"
            "'T' - To play a tune e.g. 'TA#4B2' to play A# for 4 sec and then B for 2 sec\n\r"
            "'H' - TESTING COMMAND to print list of possible commands\n\r"
            );
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