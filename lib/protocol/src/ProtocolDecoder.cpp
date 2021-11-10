#include "ProtocolDecoder.h"


ProtocolDecoder::ProtocolDecoder(std::string name) : _name(name) {}

ProtocolDecoder::~ProtocolDecoder() {};

std::string ProtocolDecoder::getName() { return _name; }


