#ifndef PARSER_H
#define PARSER_H

//defines
#define MAX_NOTE_SEQUENCE_LENGTH 16

//types
enum notePitchMod_t
{
    STANDARD,
    SHARP,
    FLAT
};

typedef struct
{
    char pitch; //could also be uint if easier
    notePitchMod_t pitchMod; //sharp or flat or standard
    uint8_t duration;
} note_t;

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


#endif