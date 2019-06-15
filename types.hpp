#ifndef TYPES_H
#define TYPES_H

enum class LightingFuncState
{
    Start,
    Step,
    End
};

/*
    Returns:
        [Start] = True if the starting animation is finished. False if it's still going.
        [Step]  = Doesn't matter.
        [End]   = True if the ending animation is finished. False if it's still going.
 */
typedef bool(*LightingFunc)(LightingFuncState);

#endif