#ifndef RETURN_CODES_H
#define RETURN_CODES_H

enum returnCode_t
{
    NO_ERR = 0,
    BUFFER_EMPTY_ERROR = -1,
    BUFFER_OVERFLOW_ERROR = -2,
    NO_COMMAND_READY_ERROR = -3,
    UNRECOGNISED_COMMAND_ERROR = -4,
    GENERAL_PARAM_READ_ERROR = -5,
    ROTATE_PARAM_ERROR = -6,
    SPEED_PARAM_ERROR = -7,
    SPEED_WITH_ROTATE_PARAM_ERROR = -8,
    INVALID_PITCH_NOTE_ERROR = -9,
    INVALID_PITCHMOD_NOTE_ERROR = -10,
    INVALID_DURATION_NOTE_ERROR = -11,
    INCOMPLETE_COMMAND_ERROR = -12,
    EXCEED_MAX_NOTE_SEQ_LENGTH = -13
    //add more as needed
};

#endif
