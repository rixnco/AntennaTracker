#ifndef __PROTOCOL_DECODER_H__
#define __PROTOCOL_DECODER_H__


#include <string>


class ProtocolDecoder {
public:
    ProtocolDecoder(std::string name);
    virtual ~ProtocolDecoder();

    std::string getName();

    virtual void reset() = 0;

    virtual void process(uint8_t data) = 0;
private:
    std::string _name;
};




#endif // __PROTOCOL_DECODER_H__