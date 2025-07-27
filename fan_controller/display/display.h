#ifndef DISPLAY_H
#define DISPLAY_H

class Display
{
public:
    virtual void setup() {}
    virtual void service(int tempToShow, float fanPowerProportion, char targetSuffix) {}
};

#endif // DISPLAY_H