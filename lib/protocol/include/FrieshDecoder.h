#ifndef __FRIESH_DECODER_H__
#define __FRIESH_DECODER_H__

#include <DataDecoder.h>


#define FRIESH_MAX_FRAME_SIZE   64

enum FrieshError { FRIESH_ERROR_CRC, FRIESH_ERROR_OVERFLOW, FRIESH_ERROR_UNKNOWN_ID, FRIESH_ERROR_BAD_FORMAT };


class FrieshDecoder;

class FrieshListener {
public:
    virtual ~FrieshListener();
 
    virtual void onFrameDecoded(FrieshDecoder* decoder, uint32_t id);
    virtual void onFrameError(FrieshDecoder* decoder, FrieshError cause, uint32_t param);
     
    virtual void onSetHomeLocation(FrieshDecoder* decoder) = 0;
    virtual void onCalibrateCompass(FrieshDecoder* decoder) =  0;
    virtual void onStartTracking(FrieshDecoder* decoder) = 0;
    virtual void onStopTracking(FrieshDecoder* decoder) =  0;

};

class FrieshDecoder : public DataDecoder {
public:
    FrieshDecoder(bool checkCRC=true);
    virtual ~FrieshDecoder();

    void setCheckCRC(bool check);
    void setFrieshListener(FrieshListener* listener);

    virtual void reset();

     virtual void process(uint8_t data);

protected:
    bool decodeFrame();

    void fireFrameDecoded(uint32_t id);
    void fireFrameError(FrieshError cause, uint32_t param=0);
    void fireSetHomeLocation();
    void fireCalibrateCompass();
    void fireStartTracking();
    void fireStopTracking();

    enum State { IDLE, LENGTH, DATA, CRC };

    bool _checkCRC;
    State _state;
    uint8_t  _buffer[FRIESH_MAX_FRAME_SIZE+1];
    uint32_t _index;
    
    uint8_t  _length;
    uint8_t  _crc;    

    FrieshListener* _listener;
};




#endif // __FRIESH_DECODER_H__