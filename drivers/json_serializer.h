#ifndef JSON_SERIALIZER_H
#define JSON_SERIALIZER_H

#include "data_structs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Function prototype for JSON serialization
const char* serializeTelemetryJSON(const telemetryRecord_t* rec);

#ifdef __cplusplus
}
#endif

#endif
