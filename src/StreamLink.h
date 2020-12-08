#ifndef __STREAM_LINK_H__
#define __STREAM_LINK_H__

#include <Stream.h>
#include "DataLink.h"


class StreamLink : public DataLink {
public:
    StreamLink();
    virtual ~StreamLink();

    void setStream(Stream *stream);
    
    bool available();
    void process();

private:
    Stream *_stream;
};

#endif // __STREAM_LINK_H__