#pragma once

#include <cstddef>
#include <cstdint>

class Stream
{
public:
    virtual ~Stream() {}
    virtual int available() { return 0; }
    virtual int read() { return -1; }
};

inline unsigned long millis() { return 0ul; }
