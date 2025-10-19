/******************************************************************************************
 *  json_serializer.cpp – Compact JSON encoder for LoRaWAN/TTN
 *  -----------------------------------------------------------
 *  Serializza telemetryRecord_t in formato JSON abbreviato e compatto (<242 B)
 *  Compatibile con The Things Network e ChirpStack.
 ******************************************************************************************/

#include "data_structs.h"
#include <stdio.h>

// Buffer temporaneo per la stringa JSON
constexpr size_t JSON_BUFFER_SIZE = 160;
static char jsonBuffer[JSON_BUFFER_SIZE];

/**
 * Serializza la struttura telemetryRecord_t in formato JSON abbreviato.
 * @param rec  Puntatore al record da convertire
 * @return     Puntatore al buffer JSON pronto per invio
 */
/**
 * Serializza la struttura telemetryRecord_t in formato JSON abbreviato.
 * @param rec  Puntatore al record da convertire
 * @return     Puntatore al buffer JSON pronto per invio
 */
const char* serializeTelemetryJSON(const telemetryRecord_t* rec)
{
    // Costruzione JSON compatto (chiavi abbreviate)
    // (Campo "id" rimosso perché non più presente nella struct)
    int len = snprintf(jsonBuffer, sizeof(jsonBuffer),
        "{"
        "\"ts\":%lu,"
        "\"lt\":%.5f,"
        "\"ln\":%.5f,"
        "\"tb\":%.1f,"
        "\"ta\":%.1f,"
        "\"h\":%.1f,"
        "\"r\":%u,"
        "\"g\":%u,"
        "\"st\":%u,"
        "\"w\":%u,"
        "\"s\":%u,"
        "\"b\":%.2f"
        "}",
        rec->timestamp,
        rec->latitude, rec->longitude,
        rec->temp_body, rec->temp_ambient,
        rec->heart_bpm,
        rec->ruminate, rec->grazing, rec->standing,
        rec->walking, rec->sitting,
        rec->batt_voltage
    );

    // Sicurezza buffer
    if (len < 0 || len >= (int)sizeof(jsonBuffer))
        jsonBuffer[sizeof(jsonBuffer) - 1] = '\0';

    return jsonBuffer;
}