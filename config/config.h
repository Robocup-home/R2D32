#ifndef CONFIG_H
#define CONFIG_H

#ifdef USE_ESP32_WIFI_CONFIG
    #include "custom/esp32_wifi_config.h"
#endif

#ifdef USE_ESP32S3_WIFI_CONFIG
    #include "custom/esp32s3_wifi_config.h"
#endif

#endif
