#ifndef PARSER_H
#define PARSER_H

//includes
#include "tune.h"

//definitions
#define MAX_NOTE_SEQUENCE_LENGTH 16

typedef struct
{
    note_t noteSeq[MAX_NOTE_SEQUENCE_LENGTH];
    uint8_t seqLength;
} dataTuneCommand_t;

typedef struct
{
    bool doRotate;
    float rotateParam;
    bool doSpeed;
    float speedParam;
} dataSpeedOrRotateCommand_t;

//function prototypes
returnCode_t parseRotateOrSpeed(const char *commandString, dataSpeedOrRotateCommand_t *dataSpeedOrRotateCommand);
returnCode_t parseTune(const char *commandString, dataTuneCommand_t *dataTuneCommand);
returnCode_t readAndProcessCommand();

#endif
