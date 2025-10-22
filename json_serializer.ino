/******************************************************************************************
 *  json_serializer.ino – Serializzazione dei dati telemetrici in JSON compatto
 *  Versione adattata per architettura multi-INO (Cicerone Collar)
 *  ---------------------------------------------------------------
 *  Espone funzione:
 *      const char* serializeTelemetryJSON(const telemetryRecord_t* rec);
 ******************************************************************************************/

#include <Arduino.h>
#include "data_structs.h"

// ======================================================================================
// === FUNZIONE DI SERIALIZZAZIONE ===
// ======================================================================================

/**
 * Serializza la struttura telemetryRecord_t in una stringa JSON compatta
 * Restituisce un puntatore a buffer statico (valido fino alla prossima chiamata)
 */
const char* serializeTelemetryJSON(const telemetryRecord_t* rec)
{
    static char json[256];  // buffer riutilizzabile

    snprintf(json, sizeof(json),
        "{\"id\":\"C1\",\"ts\":%lu,"
        "\"lt\":%.6f,\"ln\":%.6f,"
        "\"tb\":%.2f,\"ta\":%.2f,"
        "\"h\":%.1f,"
        "\"r\":%u,\"g\":%u,\"st\":%u,\"w\":%u,\"s\":%u,"
        "\"b\":%.2f,\"hr\":%.1f}",
        rec->timestamp,
        rec->latitude, rec->longitude,
        rec->temp_body, rec->temp_ambient,
        rec->humidity,
        rec->ruminate, rec->grazing, rec->standing,
        rec->walking, rec->sitting,
        rec->batt_voltage, rec->heart_bpm
    );

    return json;
}

/******************************************************************************************
 *  END OF FILE
 ******************************************************************************************/
