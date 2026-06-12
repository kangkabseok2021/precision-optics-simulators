#pragma once

class IDevice {
public:
    virtual ~IDevice() = default;
    virtual bool       init()     = 0;
    virtual bool       selfTest() = 0;
    virtual void       shutdown() = 0;
    virtual const char* name()   const = 0;
};
