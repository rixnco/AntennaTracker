#include "DataDecoder.h"


DataDecoder::DataDecoder(std::string name) : _name(name) {}

DataDecoder::~DataDecoder() {};

std::string DataDecoder::getName() { return _name; }


