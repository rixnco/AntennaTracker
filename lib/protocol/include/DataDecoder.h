#ifndef __DATA_DECODER_H__
#define __DATA_DECODER_H__


#include <string>


class DataDecoder {
public:
    DataDecoder(std::string name);
    virtual ~DataDecoder();

    std::string getName();

    virtual void reset() = 0;

    virtual void process(uint8_t data) = 0;
private:
    std::string _name;
};




#endif // __DATA_DECODER_H__