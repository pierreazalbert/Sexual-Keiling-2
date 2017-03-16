#include "mbed.h"
#include "rtos.h"
#include "return_codes.h"
#include "parser.h"

//Function to parse command
// - parses string to acquire command/s and parameters
// - currently just prints message and returns error code if parsing fails
returnCode_t parseCommand(const char *commandString)
{
    //Parse string for (R-?\d{1,3}(\.\d{1,2})?)?(V\d{1,3}(\.\d{1,2})?)?(T([A-G][#\^]?[1-8]){1,16})?
    
    switch(commandString[0])
    {
        case 'R':
        {         
            // ***************************************** ROTATION(/SPEED) *****************************************
            char speedType = '\0';
            float rotateParam = 0, speedParam = 0;
            uint8_t numVarsFilled = sscanf(&commandString[1], "%f%1[^.0-9]%f", &rotateParam, &speedType, &speedParam); //sscanf returns the number of vars filled
            
            switch(numVarsFilled)
            {
                case 0:
                    //pc.printf("Error: Rotate parameter could not be read\n\r");
                    return ROTATE_PARAM_ERROR;
                
                case 1:
                    // ***************************************** ROTATION ONLY *****************************************
                    //pc.printf("Rotate %3.2f degrees\n\r", rotateParam);
                    //TODO do rotation/return command somehow
                    break;
                
                case 2:
                    //pc.printf("Error: Speed parameter could not be read\n\r");
                    return SPEED_WITH_ROTATE_PARAM_ERROR;
                
                case 3:
                    if(speedType != 'V')
                    {
                        //pc.printf("Unrecognised Command (second command)\n\r");
                        return UNRECOGNISED_COMMAND_ERROR;
                    }
                    
                    // ***************************************** ROTATION & SPEED *****************************************
                    //pc.printf("Rotate %3.2f rotations at %3.2f rotations/sec\n\r", rotateParam, speedParam);
                    //TODO do rotation and speed/return command somehow
                    
                    break;
            } // end numVarsFilled switch
            
            break;
        } // end 'R' case
        case 'V':
        {
            // ***************************************** SPEED *****************************************
            float rotateParam = 0;
            uint8_t numVarsFilled = sscanf(&commandString[1], "%f", &rotateParam); //sscanf returns the number of vars filled
            
            if(numVarsFilled == 0)
            {
                //pc.printf("Error: Speed parameter could not be read\n\r");
                return SPEED_PARAM_ERROR;
            }
            
            //pc.printf("Spin at %3.2f rotations/sec\n\r", rotateParam);
            //TODO do speed/return command somehow

            break;
        } // end 'V' case
        case 'T':
        {
            // ***************************************** TUNE *****************************************
            int commandIndex = 1; //this is used to navigate through the entire command, we skip the first char as we know this is T
            
            note_t noteSeq[MAX_NOTE_SEQUENCE_LENGTH];
            uint8_t seqIndex = 0;
            
            while(commandString[commandIndex] != '\0') //until end of command
            {
                //Find the pitch
                if(commandString[commandIndex] < 0x41 || commandString[commandIndex] > 0x47) //if not between A and G inclusive
                {
                    //pc.printf("Error: Note parameter could not be read \n\r\t%c must be char between A and G \n\r", commandString[commandIndex]);
                    return INVALID_PITCH_NOTE_ERROR;
                }
                noteSeq[seqIndex].pitch = commandString[commandIndex];
                commandIndex++;  //move to next character
                
                //Find sharp/flat or duration
                if(commandString[commandIndex] == '^')
                {
                    noteSeq[seqIndex].pitchMod = FLAT;
                }
                else if(commandString[commandIndex] == '#')
                {
                    noteSeq[seqIndex].pitchMod = SHARP;                
                }
                else if(commandString[commandIndex] >= 0x31 && commandString[commandIndex] <= 0x38)
                {
                    noteSeq[seqIndex].pitchMod = STANDARD;
                    noteSeq[seqIndex].duration = commandString[commandIndex] - 0x30; // convert UTF-8 to int
                }
                else if(commandString[commandIndex] == '\0')
                {
                    //pc.printf("Error: Note parameter could not be read \n\r\tpitch must be followed by # or ^ or \\d{1-8} \n\r", commandString[commandIndex]);
                    return INCOMPLETE_NOTE_ERROR;
                }
                else
                {
                    //pc.printf("Error: Note parameter could not be read \n\r\t%c must be # or ^ or \\d{1-8} \n\r", commandString[commandIndex]);
                    return INVALID_PITCHMOD_NOTE_ERROR;
                }
                commandIndex++; //move to next character
                
                //Find duration if there was flat/sharp
                if(noteSeq[seqIndex].pitchMod != STANDARD)
                {
                    if(commandString[commandIndex] >= 0x31 && commandString[commandIndex] <= 0x38)
                    {
                        noteSeq[seqIndex].duration = commandString[commandIndex] - 0x30; // convert UTF-8 to int
                    }
                    
                    else if(commandString[commandIndex] == '\0')
                    {
                        //pc.printf("Error: Note parameter could not be read \n\r\t# or ^ must be followed by \\d{1-8} \n\r", commandString[commandIndex]);
                        return INCOMPLETE_NOTE_ERROR;
                    }
                    else
                    {
                        return INVALID_DURATION_NOTE_ERROR;
                    }    
                    commandIndex++;  //move to next character
                }
                
                //pc.printf("Note %i: Tune %c %i for %i sec\n\r", seqIndex, noteSeq[seqIndex].pitch, noteSeq[seqIndex].pitchMod, noteSeq[seqIndex].duration);
                seqIndex++; //move to next note in the sequence
            }
            
            //TODO do tune/return command somehow

            break;
        } // end 'T' case
        default:
        {
            //pc.printf("Unrecognised Command\n\r");
            return UNRECOGNISED_COMMAND_ERROR;
        }
    }// end commandString[0] switch
    return NO_ERR;
}