#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "parser.h"

//Function to parse rotate and/or speed command
// - parses string to acquire command/s and parameters
// - returns error code if parsing fails
returnCode_t parseRotateOrSpeed(const char *commandString, dataSpeedOrRotateCommand_t *dataSpeedOrRotateCommand)
{
    //Parse string for (R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,2})?)?(T([A-G][#\^]?[1-8]){1,16})?

    //Initialise command data
    dataSpeedOrRotateCommand->doRotate = false;
    dataSpeedOrRotateCommand->rotateParam = 0;
    dataSpeedOrRotateCommand->doSpeed = false;
    dataSpeedOrRotateCommand->speedParam = 0;
    
    switch(commandString[0])
    {
        case 'R':
        {         
            // ***************************************** ROTATION(/SPEED) *****************************************
            dataSpeedOrRotateCommand->doRotate = true;
            
            char speedType = '\0';
            float rotateParam = 0, speedParam = 0;
            uint8_t numVarsFilled = sscanf(&commandString[1], "%f%1[^.0-9]%f", &rotateParam, &speedType, &speedParam); //sscanf returns the number of vars filled
            
            switch(numVarsFilled)
            {
                case 0:
                    // COULDN'T READ ROTATE PARAM
                    return ROTATE_PARAM_ERROR;
                
                case 1:
                    // ROTATION ONLY
                    dataSpeedOrRotateCommand->rotateParam = rotateParam;
                    break;
                
                case 2:
                    // COULDN'T READ SPEED PARAM
                    return SPEED_WITH_ROTATE_PARAM_ERROR;
                
                case 3:
                    if(speedType != 'V')
                    {
                        // EXPECTED V AS SECOND COMMAND CODE, GOT SOMETHING ELSE
                        return UNRECOGNISED_COMMAND_ERROR;
                    }
                    // ROTATION & SPEED
                    dataSpeedOrRotateCommand->rotateParam = rotateParam;
                    dataSpeedOrRotateCommand->doSpeed = true;
                    dataSpeedOrRotateCommand->speedParam = abs(speedParam); //sign of V is ignored if R is specified
                    
                    break;
                
                default:
                    //some other read error
                    return GENERAL_PARAM_READ_ERROR;
                    
            } // end numVarsFilled switch
            
            break;
        } // end 'R' case
        case 'V':
        {
            // SPEED
            float speedParam = 0;
            uint8_t numVarsFilled = sscanf(&commandString[1], "%f", &speedParam); //sscanf returns the number of vars filled

            if(numVarsFilled == 0)
            {
                // COULDN'T READ SPEED PARAM
                return SPEED_PARAM_ERROR;
            }

            dataSpeedOrRotateCommand->doSpeed = true;
            dataSpeedOrRotateCommand->speedParam = speedParam;
            
            break;
        } // end 'V' case
        default:
        {
            // UNRECOGNISED COMMAND
            return UNRECOGNISED_COMMAND_ERROR;
        }
    }// end commandString[0] switch
    return NO_ERR;
}

//Function to parse tune command
// - parses string to acquire command/s and parameters
// - returns error code if parsing fails
returnCode_t parseTune(const char *commandString, dataTuneCommand_t *dataTuneCommand)
{
    //Parse string for (R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,2})?)?(T([A-G][#\^]?[1-8]){1,16})?
    
    if(commandString[0] == 'T')
    {
    
        // ***************************************** TUNE *****************************************
        int commandIndex = 1; //this is used to navigate through the entire command, we skip the first char as we know this is T
        uint8_t seqIndex = 0;
                
        while(commandString[commandIndex] != '\0') //until end of command
        {
            if(seqIndex >= MAX_NOTE_SEQUENCE_LENGTH)
            {
                return EXCEED_MAX_NOTE_SEQ_LENGTH;
            }
            
            // CHARACTER 1
            dataTuneCommand->noteSeq[seqIndex].pitch = charToPitch(commandString[commandIndex]);
            if(dataTuneCommand->noteSeq[seqIndex].pitch == INVALID_PITCH)
            {
                return INVALID_PITCH_NOTE_ERROR;
            }
            commandIndex++;  //move to next character
            
            // CHARACTER 2 - PITCHMOD OR DURATION
            if(commandString[commandIndex] == '^')
            {
                dataTuneCommand->noteSeq[seqIndex].pitch = (pitch_t)(dataTuneCommand->noteSeq[seqIndex].pitch + 1); //change to next pitch_t value
            }
            else if(commandString[commandIndex] == '#')
            {
                dataTuneCommand->noteSeq[seqIndex].pitch = (pitch_t)(dataTuneCommand->noteSeq[seqIndex].pitch - 1); //change to prev pitch_t value
            }
            else if(commandString[commandIndex] >= 0x31 && commandString[commandIndex] <= 0x38)
            {
                dataTuneCommand->noteSeq[seqIndex].duration = commandString[commandIndex] - 0x30; // convert UTF-8 to int
            }
            else if(commandString[commandIndex] == '\0')
            {
                // COMMAND ENDS EARLY
                return INCOMPLETE_COMMAND_ERROR;
            }
            else
            {
                // CHARACTER IS NOT VALID - MUST BE # OR ^ OR \d{1-8}
                return INVALID_PITCHMOD_NOTE_ERROR;
            }
            commandIndex++; //move to next character
            
            // CHARACTER 3 - DURATION, ONLY OCCURS IF PREVIOUS CHARACTER WAS # or ^
            if(commandString[commandIndex-1] == '^' || commandString[commandIndex-1] == '#')
            {
                if(commandString[commandIndex] >= 0x31 && commandString[commandIndex] <= 0x38)
                {
                    dataTuneCommand->noteSeq[seqIndex].duration = commandString[commandIndex] - 0x30; // convert UTF-8 to int
                }
                
                else if(commandString[commandIndex] == '\0')
                {
                    // COMMAND ENDS EARLY - # OR ^ MUST BE FOLLOWED BY DURATION
                    return INCOMPLETE_COMMAND_ERROR;
                }
                else
                {
                    // DURATION MUST BE DIGIT BETWEEN 1 AND 8
                    return INVALID_DURATION_NOTE_ERROR;
                }    
                commandIndex++;  //move to next character
            }
            
            seqIndex++; //move to next note in the sequence
        }// end of while
        
        dataTuneCommand->seqLength = seqIndex;
    }// end commandString[0] switch
    else
    { 
        // UNRECOGNISED COMMAND 
        return UNRECOGNISED_COMMAND_ERROR;
    }
    return NO_ERR;
}
