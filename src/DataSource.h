#ifndef __DATA_SOURCE_H__
#define __DATA_SOURCE_H__

#include <Arduino.h>


class DataListener {
public:
    virtual ~DataListener();

    virtual void onDataReceived(uint8_t data) = 0;
};


class DataSource {
public:
    DataSource();
    virtual ~DataSource();

    virtual void close() = 0;
    void setDataListener(DataListener* listener);

protected:
    void doNotify(const uint8_t* data, size_t length);

    DataListener* _listener;
};




#endif // __DATA_PROVIDER_H__