#ifndef PARSER_H
#define PARSER_H

//defines
#define MAX_NOTE_SEQUENCE_LENGTH 16

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

//function prototype
returnCode_t parseCommand(const char *commandString);

#endif