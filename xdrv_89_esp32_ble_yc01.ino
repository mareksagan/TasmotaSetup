/*
  xdrv_86_esp32_ble_yc01.ino - Native BLE driver for YC01/YIERYI/YINMIK 6-in-1 water monitor

  This driver talks to the YC01 over NimBLE directly, without using the generic
  Tasmota BLE engine (xdrv_79). It is intended for users who need a dedicated,
  predictable GATT polling loop for this single device.

  Protocol:
    - Advertised name: BLE-YC01
    - Service 0xFF01, characteristic 0xFF02 (read/write/notify)
    - Host must poll by reading 0xFF02; the meter auto-powers off after ~5 min
      without an active connection/reads.
    - Returned 18+ byte frame is decrypted with a bit-swap + invert algorithm,
      then parsed as big-endian signed 16-bit integers.

  Commands:
    YC01               - show status / active profile
    YC01Read           - force immediate read
    YC01Poll N         - set poll interval, 10..600 seconds (default 120)
    YC01Start          - write start-measurement payload
    YC01Stop           - write stop-measurement payload
    YC01Profile [name] - set profile or list profiles
    YC01Boost 0/1      - toggle boost thresholds for active profile
    YC01Mac <addr> [0/1] - set target MAC and BLE address type (public=0, random=1)

  Copyright (C) 2025  YC01 contributors
  Released under the same GPL v3 license as Tasmota.
*/

#ifdef ESP32
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#ifdef USE_YC01_ESP32

#define XDRV_89 89
#define D_CMND_YC01 "YC01"

#include <NimBLEDevice.h>

/*********************************************************************************************\
 * Constants and types
\*********************************************************************************************/

#define YC01_SVC_UUID        0xFF01
#define YC01_CHR_UUID        0xFF02
#define YC01_MAX_RETRY_SEC   300
#define YC01_OP_TIMEOUT_SEC  30
#define YC01_DEFAULT_POLL_S  120
#define YC01_MAX_MAC_STR_LEN 32

#ifndef YC01_DEFAULT_MAC
#define YC01_DEFAULT_MAC "414284588113"
#endif

enum YC01_Op_e {
  YC01_OP_READ = 0,
  YC01_OP_START,
  YC01_OP_STOP
};

enum YC01_TaskState_e {
  YC01_TS_IDLE = 0,
  YC01_TS_BUSY,
  YC01_TS_OK,
  YC01_TS_FAIL
};

struct yc01_reading_t {
  float ph;
  int16_t ec;
  int16_t tds;
  int16_t orp;
  float cl;
  float temp;
  uint8_t batt;
  float salt;
  int8_t rssi;
  bool valid;
};

struct yc01_profile_t {
  const char name[64];
  float ph_min, ph_max;
  int16_t ec_min, ec_max;
  int16_t temp_min, temp_max;
  float cl_max;
  int16_t orp_min, orp_max;
  int16_t salt_max;
  uint8_t has_boost;
  float boost_ph_min, boost_ph_max;
  int16_t boost_ec_min, boost_ec_max;
  int16_t boost_temp_min, boost_temp_max;
};

const yc01_profile_t YC01_PROFILES[] PROGMEM = {
  {"Generic", 5.8, 6.2, 1000, 2500, 20, 25, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"CherryTomato", 5.8, 6.0, 1400, 2000, 20, 25, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1800, 2800, 20, 26},
  {"BeefsteakTomato", 5.8, 6.0, 1400, 2000, 20, 25, 0.2, 300, 450, 750, 1, 6.0, 6.3, 2000, 3500, 20, 26},
  {"BellPepper_Poblano_BananaPepper", 5.8, 6.0, 1200, 1800, 22, 26, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1600, 2400, 22, 26},
  {"ChiliPepper_Jalapeno_Cayenne", 5.8, 6.0, 1200, 1900, 22, 26, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1600, 2600, 22, 27},
  {"Habanero", 5.8, 6.0, 1200, 2000, 22, 27, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1600, 2800, 22, 28},
  {"Eggplant", 5.8, 6.0, 1200, 1800, 22, 26, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1600, 2400, 22, 26},
  {"Okra", 5.8, 6.2, 1200, 1800, 22, 27, 0.2, 300, 450, 750, 1, 6.0, 6.5, 1400, 2200, 22, 28},
  {"BushBeans", 5.8, 6.2, 1000, 1400, 18, 24, 0.2, 300, 450, 500, 1, 6.0, 6.3, 1400, 2000, 18, 24},
  {"PoleBeans", 5.8, 6.2, 1000, 1400, 18, 24, 0.2, 300, 450, 500, 1, 6.0, 6.3, 1400, 2200, 18, 24},
  {"Peas_SnowPeas_SugarSnapPeas", 5.8, 6.2, 800, 1200, 15, 20, 0.2, 300, 450, 500, 1, 6.0, 6.3, 1200, 1800, 15, 20},
  {"Edamame", 5.8, 6.2, 1000, 1400, 18, 24, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1400, 2000, 18, 24},
  {"Cantaloupe_HoneydewMelon_MiniWatermelon", 5.8, 6.2, 1200, 1600, 22, 26, 0.2, 300, 450, 750, 1, 6.0, 6.3, 1600, 2400, 22, 26},
  {"Strawberry_EverbearingStrawberry_AlpineStrawberry", 5.5, 5.8, 1000, 1400, 15, 22, 0.1, 300, 450, 500, 1, 6.0, 6.2, 1400, 2000, 15, 22},
  {"Lettuce_ButterheadLettuce_RomaineLettuce", 6.0, 6.2, 1200, 2000, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Spinach_NewZealandSpinach", 6.0, 6.5, 1200, 1600, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Kale_CollardGreens", 6.0, 6.5, 1200, 1800, 18, 22, 0.2, 300, 450, 10000, 0, 0, 0, 0, 0, 0, 0},
  {"Arugula", 6.0, 6.2, 1500, 1800, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Endive_Escarole_Frisee_Radicchio", 6.0, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Mache", 6.0, 6.5, 800, 1200, 15, 20, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Microgreens", 5.5, 5.8, 400, 700, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"BokChoy_Senposai_YukinaSavoy", 6.0, 6.5, 1200, 1600, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Tatsoi_Komatsuna_Mibuna", 6.0, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"MustardGreens", 6.0, 6.2, 1200, 1600, 18, 22, 0.2, 300, 450, 10000, 0, 0, 0, 0, 0, 0, 0},
  {"Mizuna", 6.0, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 10000, 0, 0, 0, 0, 0, 0, 0},
  {"Watercress", 6.0, 6.5, 700, 1100, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Amaranth", 6.0, 6.5, 1200, 1600, 20, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"MalabarSpinach", 6.0, 6.5, 1200, 1600, 22, 26, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Purslane", 6.0, 6.5, 800, 1200, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Sorrel", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Celtuce", 6.0, 6.5, 1200, 1600, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"SweetPotatoVine", 6.0, 6.5, 1200, 1600, 20, 24, 0.2, 300, 450, 10000, 0, 0, 0, 0, 0, 0, 0},
  {"GarlicChives", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Basil_ThaiBasil_HolyBasil", 5.8, 6.2, 1200, 1800, 20, 24, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Parsley", 6.0, 6.5, 1000, 1600, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Cilantro", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Dill", 5.8, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Mint_Peppermint_Spearmint", 5.8, 6.2, 1200, 1600, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Chives", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Oregano", 6.0, 6.5, 1200, 1800, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Thyme_Sage_Tarragon_Marjoram", 6.0, 6.5, 1000, 1600, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Rosemary_BayLaurel", 6.0, 6.5, 1200, 1800, 18, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"LemonBalm", 5.8, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Lemongrass", 6.0, 6.5, 1400, 2000, 20, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Stevia", 6.0, 6.5, 1000, 1600, 20, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Shiso", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"VietnameseCoriander", 6.0, 6.5, 1000, 1400, 20, 24, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Culantro", 6.0, 6.5, 1000, 1400, 20, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Epazote", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Lovage", 6.0, 6.5, 1200, 1800, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"SummerSavory", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"WinterSavory", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Lavender", 6.0, 6.8, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Chamomile", 6.0, 6.5, 800, 1200, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Feverfew_Hyssop_Echinacea", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Fennel", 6.0, 6.5, 1200, 1800, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Nasturtium_Calendula_Borage_Pansy_Viola", 6.0, 6.5, 1000, 1400, 18, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Petunia", 5.5, 6.0, 1000, 1600, 18, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"GerberaDaisy", 5.8, 6.2, 1600, 2200, 18, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Zinnia", 5.8, 6.2, 1400, 2000, 20, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Snapdragon", 5.8, 6.2, 1200, 1800, 15, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Begonia_Impatiens", 5.5, 6.2, 800, 1400, 18, 24, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"SweetAlyssum", 5.8, 6.2, 800, 1200, 15, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Lobelia", 5.5, 6.2, 800, 1200, 15, 22, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Marigold", 6.0, 6.5, 1200, 1800, 20, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Dianthus", 6.0, 6.5, 1400, 2000, 18, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Cornflower", 6.0, 6.8, 1000, 1600, 15, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Portulaca", 5.8, 6.5, 800, 1400, 20, 28, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Ginger_Turmeric", 5.5, 6.5, 1400, 2000, 22, 28, 0.2, 300, 450, 8000, 0, 0, 0, 0, 0, 0, 0},
  {"Claytonia", 5.8, 6.5, 800, 1200, 10, 20, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"LandCress", 5.8, 6.5, 700, 1200, 10, 20, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"WelshOnion", 6.0, 6.5, 900, 1300, 10, 22, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Daylily", 6.0, 6.5, 800, 1400, 15, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"FavaBeans", 5.8, 6.2, 800, 1200, 10, 20, 0.2, 300, 450, 500, 1, 6.0, 6.3, 1200, 1800, 10, 20},
  {"BearsGarlic", 6.0, 6.5, 800, 1200, 8, 18, 0.2, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Lettuce_Spinach_Kale", 5.8, 6.2, 1000, 1600, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Lettuce_Arugula_Chard", 5.8, 6.2, 1000, 1600, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Lettuce_Basil_Parsley", 5.8, 6.2, 900, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Spinach_Kale_Chard", 6.0, 6.5, 1200, 1600, 18, 22, 0.1, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Microgreens", 5.8, 6.2, 500, 900, 18, 22, 0.1, 300, 450, 4000, 0, 0, 0, 0, 0, 0, 0},
  {"SaladGreens", 5.8, 6.2, 900, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Lettuce_Spinach_Arugula_Radish", 5.8, 6.2, 900, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"BokChoy_Tatsoi_Komatsuna", 6.0, 6.5, 1000, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Mizuna_Mibuna_Senposai", 6.0, 6.2, 1000, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Basil_Mint_Parsley", 5.8, 6.5, 1200, 1600, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Cilantro_Dill_Chives", 6.0, 6.5, 1000, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Rosemary_Thyme_Oregano", 6.0, 6.5, 1200, 1600, 18, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"LemonBalm_Chamomile_Mint", 5.8, 6.5, 1000, 1400, 18, 22, 0.1, 300, 450, 5000, 0, 0, 0, 0, 0, 0, 0},
  {"Mediterranean_Herb", 6.0, 6.5, 1200, 1600, 20, 24, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Tomato_Pepper_Eggplant", 6.0, 6.3, 1600, 2400, 22, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"CherryTomato_BellPepper", 6.0, 6.3, 1600, 2200, 22, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Pepper_Cucumber", 5.8, 6.2, 1400, 2200, 22, 25, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Eggplant_Zucchini_Pepper", 6.0, 6.3, 1600, 2200, 22, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Tomato_CherryTomato_Roma", 5.8, 6.3, 1400, 2400, 20, 26, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"BellPepper_ChiliPepper_Jalapeno", 6.0, 6.3, 1600, 2400, 22, 27, 0.2, 300, 450, 7500, 0, 0, 0, 0, 0, 0, 0},
  {"Radish_Turnip_Carrot", 5.8, 6.2, 1000, 1400, 18, 22, 0.2, 300, 450, 750, 0, 0, 0, 0, 0, 0, 0},
  {"GreenOnion_GarlicChives_Leek", 6.0, 6.5, 900, 1300, 18, 22, 0.1, 300, 450, 500, 0, 0, 0, 0, 0, 0, 0},
  {"Onion_Shallot_Chive", 6.0, 6.5, 900, 1300, 18, 22, 0.1, 300, 450, 500, 0, 0, 0, 0, 0, 0, 0},
  {"Petunia_Lobelia_Alyssum", 5.5, 6.2, 800, 1400, 15, 22, 0.2, 300, 450, 500, 0, 0, 0, 0, 0, 0, 0},
  {"BushBeans_PoleBeans_Edamame", 6.0, 6.3, 1200, 1600, 18, 24, 0.2, 300, 450, 750, 0, 0, 0, 0, 0, 0, 0},
  {"Watercress_Purslane_Sorrel", 6.0, 6.5, 800, 1200, 18, 22, 0.1, 300, 450, 500, 0, 0, 0, 0, 0, 0, 0},
  {"Radicchio_Mache", 6.0, 6.5, 800, 1200, 15, 20, 0.1, 300, 450, 400, 0, 0, 0, 0, 0, 0, 0},
  {"BeetGreens_TurnipGreens_CollardGreens", 6.0, 6.5, 1200, 1600, 18, 22, 0.1, 300, 450, 750, 0, 0, 0, 0, 0, 0, 0},
  {"Tomato_Basil", 5.8, 6.2, 1400, 2200, 20, 26, 0.2, 300, 450, 750, 0, 0, 0, 0, 0, 0, 0},
  {"Shiso_VietnameseCoriander_Culantro", 6.0, 6.5, 1000, 1400, 20, 24, 0.1, 300, 450, 500, 0, 0, 0, 0, 0, 0, 0},
  {"Lavender_Rosemary_Sage", 6.0, 6.8, 1000, 1600, 18, 24, 0.2, 300, 450, 750, 0, 0, 0, 0, 0, 0, 0}
};
const size_t YC01_PROFILE_COUNT = 106;

struct yc01_active_profile_t {
  char name[64];
  float ph_min, ph_max;
  int16_t ec_min, ec_max;
  int16_t tds_min, tds_max;
  int16_t orp_min, orp_max;
  float cl_min, cl_max;
  float temp_min, temp_max;
  float salt_min, salt_max;
  float ph_margin;
  float ec_margin;
  float tds_margin;
  float orp_margin;
  float cl_margin;
  float temp_margin;
  float salt_margin;
};

/*********************************************************************************************\
 * Globals
\*********************************************************************************************/

static yc01_reading_t yc01_last = {0};
static yc01_active_profile_t yc01_profile = {0};

static char yc01_mac_str[YC01_MAX_MAC_STR_LEN] = YC01_DEFAULT_MAC;
static uint8_t yc01_mac[6] = {0};
static uint8_t yc01_addr_type = 0;          // 0 = public, 1 = random
static uint16_t yc01_poll_s = YC01_DEFAULT_POLL_S;
static bool yc01_boost = false;

static volatile YC01_TaskState_e yc01_task_state = YC01_TS_IDLE;
static volatile YC01_Op_e yc01_pending_op = YC01_OP_READ;
static volatile bool yc01_request = false;

static uint32_t yc01_tick = 0;
static uint32_t yc01_last_ok = 0;
static uint32_t yc01_last_try = 0;
static uint8_t  yc01_fail_count = 0;
static uint32_t yc01_retry_at = 0;
static uint32_t yc01_watchdog = 0;
static bool yc01_nimble_initialized = false;

static SemaphoreHandle_t yc01_mutex = nullptr;

/*********************************************************************************************\
 * Helpers
\*********************************************************************************************/

static int16_t yc01_s16be(const uint8_t *d, size_t i) {
  uint16_t v = ((uint16_t)d[i] << 8) | d[i + 1];
  if (v > 32767) v -= 65536;
  return (int16_t)v;
}

static void yc01_decode(uint8_t *d, size_t len) {
  for (int i = (int)len - 1; i > 0; i--) {
    uint8_t t1 = d[i];
    uint8_t h1 = (t1 & 0x55) << 1;
    uint8_t l1 = (t1 & 0xAA) >> 1;
    uint8_t t0 = d[i - 1];
    uint8_t h0 = (t0 & 0x55) << 1;
    uint8_t l0 = (t0 & 0xAA) >> 1;
    d[i]     = 0xFF - (h1 | l0);
    d[i - 1] = 0xFF - (h0 | l1);
  }
}

static void yc01_parse(const uint8_t *d, size_t len) {
  if (len < 18) return;
  if (d[0] != 1 || d[2] != 15) return;

  yc01_reading_t r;
  r.ph   = yc01_s16be(d, 3) / 100.0f;
  r.ec   = yc01_s16be(d, 5);
  r.tds  = yc01_s16be(d, 7);
  r.orp  = yc01_s16be(d, 9);
  int16_t cl_raw = yc01_s16be(d, 11);
  r.cl   = (cl_raw < 0 ? 0.0f : cl_raw / 10.0f);
  r.temp = yc01_s16be(d, 13) / 10.0f;
  int32_t batt_raw = yc01_s16be(d, 15);
  int32_t batt = (100 * (batt_raw - 1950)) / (3190 - 1950);
  if (batt < 0) batt = 0;
  if (batt > 100) batt = 100;
  r.batt = (uint8_t)batt;
  r.salt = r.ec * 0.55f;
  r.rssi = -100;          // updated separately if we scan
  r.valid = true;

  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_last = r;
  yc01_last_ok = millis();
  xSemaphoreGive(yc01_mutex);
}

static bool yc01_parse_mac(const char *src, uint8_t *dst, uint8_t *type) {
  char buf[YC01_MAX_MAC_STR_LEN];
  strlcpy(buf, src, sizeof(buf));

  // Look for trailing address type: AABBCCDDEEFF/1
  char *slash = strchr(buf, '/');
  if (slash) {
    *slash = 0;
    if (type) *type = (uint8_t)atoi(slash + 1);
  } else {
    if (type) *type = 0;
  }

  // Remove separators
  char clean[13] = {0};
  int j = 0;
  for (int i = 0; buf[i] && j < 12; i++) {
    char c = buf[i];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
      clean[j++] = c;
    }
  }
  if (j != 12) return false;

  for (int i = 0; i < 6; i++) {
    char hex[3] = { clean[i * 2], clean[i * 2 + 1], 0 };
    dst[i] = (uint8_t)strtol(hex, nullptr, 16);
  }
  return true;
}

static void yc01_mac_to_str(const uint8_t *mac, char *out, size_t out_len) {
  snprintf(out, out_len, "%02X%02X%02X%02X%02X%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static const yc01_profile_t* yc01_find_profile(const char *name) {
  for (size_t i = 0; i < YC01_PROFILE_COUNT; i++) {
    char pname[64];
    strncpy_P(pname, YC01_PROFILES[i].name, sizeof(pname) - 1);
    pname[sizeof(pname) - 1] = 0;
    if (!strcasecmp(pname, name)) {
      return &YC01_PROFILES[i];
    }
  }
  return nullptr;
}

static void yc01_load_profile(const char *name) {
  const yc01_profile_t *p = yc01_find_profile(name);
  if (!p) {
    AddLog(LOG_LEVEL_INFO, PSTR("YC01: profile '%s' not found, using Generic"), name);
    p = yc01_find_profile("Generic");
  }
  if (!p) return;

  yc01_active_profile_t ap;
  memset(&ap, 0, sizeof(ap));

  strncpy_P(ap.name, p->name, sizeof(ap.name) - 1);
  ap.ph_min   = pgm_read_float(&p->ph_min);
  ap.ph_max   = pgm_read_float(&p->ph_max);
  ap.ec_min   = (int16_t)pgm_read_word(&p->ec_min);
  ap.ec_max   = (int16_t)pgm_read_word(&p->ec_max);
  ap.tds_min  = (int16_t)(ap.ec_min * 0.64f);
  ap.tds_max  = (int16_t)(ap.ec_max * 0.64f);
  ap.orp_min  = (int16_t)pgm_read_word(&p->orp_min);
  ap.orp_max  = (int16_t)pgm_read_word(&p->orp_max);
  ap.cl_min   = 0.0f;
  ap.cl_max   = pgm_read_float(&p->cl_max);
  ap.temp_min = (int16_t)pgm_read_word(&p->temp_min);
  ap.temp_max = (int16_t)pgm_read_word(&p->temp_max);
  ap.salt_min = 0.0f;
  ap.salt_max = (int16_t)pgm_read_word(&p->salt_max) / 10.0f;

  if (yc01_boost && pgm_read_byte(&p->has_boost)) {
    ap.ph_min   = pgm_read_float(&p->boost_ph_min);
    ap.ph_max   = pgm_read_float(&p->boost_ph_max);
    ap.ec_min   = (int16_t)pgm_read_word(&p->boost_ec_min);
    ap.ec_max   = (int16_t)pgm_read_word(&p->boost_ec_max);
    ap.tds_min  = (int16_t)(ap.ec_min * 0.64f);
    ap.tds_max  = (int16_t)(ap.ec_max * 0.64f);
    ap.temp_min = (int16_t)pgm_read_word(&p->boost_temp_min);
    ap.temp_max = (int16_t)pgm_read_word(&p->boost_temp_max);
  }

  ap.ph_margin   = 0.1f;
  ap.ec_margin   = (ap.ec_max - ap.ec_min) * 0.15f;
  if (ap.ec_margin < 50) ap.ec_margin = 50;
  ap.tds_margin  = ap.ec_margin * 0.64f;
  if (ap.tds_margin < 32) ap.tds_margin = 32;
  ap.orp_margin  = 50.0f;
  ap.cl_margin   = ap.cl_max * 0.5f;
  ap.temp_margin = 2.0f;
  ap.salt_margin = ap.salt_max * 0.15f;
  if (ap.salt_margin < 50) ap.salt_margin = 50;

  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_profile = ap;
  xSemaphoreGive(yc01_mutex);
}

static const char* yc01_color(float v, float min, float max, float margin) {
  if (v < min - margin || v > max + margin) return "red";
  if (v < min || v > max) return "orange";
  return "green";
}

/*********************************************************************************************\
 * BLE task
\*********************************************************************************************/

static void yc01_ble_cleanup(NimBLEClient *pClient) {
  if (pClient) {
    if (pClient->isConnected()) {
      pClient->disconnect();
    }
    NimBLEDevice::deleteClient(pClient);
  }
}

static bool yc01_ble_init_once(void) {
  if (yc01_nimble_initialized) return true;
  NimBLEDevice::init("YC01");
  yc01_nimble_initialized = true;
  return true;
}

static bool yc01_ble_perform_op(uint8_t op) {
  if (!yc01_ble_init_once()) return false;

  NimBLEClient *pClient = NimBLEDevice::createClient();
  if (!pClient) return false;

  pClient->setConnectionParams(12, 12, 0, 51);
  pClient->setConnectTimeout(15);

  bool ok = pClient->connect(NimBLEAddress(yc01_mac, yc01_addr_type), true);
  if (!ok) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: connect failed"));
    yc01_ble_cleanup(pClient);
    return false;
  }

  NimBLERemoteService *pSvc = pClient->getService(NimBLEUUID((uint16_t)YC01_SVC_UUID));
  if (!pSvc) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: service FF01 not found"));
    yc01_ble_cleanup(pClient);
    return false;
  }

  NimBLERemoteCharacteristic *pChr = pSvc->getCharacteristic(NimBLEUUID((uint16_t)YC01_CHR_UUID));
  if (!pChr) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: characteristic FF02 not found"));
    yc01_ble_cleanup(pClient);
    return false;
  }

  if (op == (uint8_t)YC01_OP_START) {
    uint8_t payload[] = {0x01, 0x01, 0x00};
    payload[2] = payload[0] ^ payload[1];
    if (!pChr->writeValue(payload, sizeof(payload), true)) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: start write failed"));
    }
  } else if (op == (uint8_t)YC01_OP_STOP) {
    uint8_t payload[] = {0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
    for (size_t i = 1; i < 5; i++) payload[5] ^= payload[i];
    if (!pChr->writeValue(payload, sizeof(payload), true)) {
      AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: stop write failed"));
    }
  }

  if (op == (uint8_t)YC01_OP_READ || op == (uint8_t)YC01_OP_START) {
    std::string value = pChr->readValue();
    if (value.length() >= 18) {
      uint8_t buf[32];
      memcpy(buf, value.data(), value.length() > 32 ? 32 : value.length());
      yc01_decode(buf, value.length());
      yc01_parse(buf, value.length());
    } else {
      AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: short read %d bytes"), value.length());
      yc01_ble_cleanup(pClient);
      return false;
    }
  }

  yc01_ble_cleanup(pClient);
  return true;
}

static void yc01_ble_task(void *pvParameters) {
  while (true) {
    bool do_request = false;
    YC01_Op_e op = YC01_OP_READ;

    xSemaphoreTake(yc01_mutex, portMAX_DELAY);
    if (yc01_request) {
      do_request = true;
      op = yc01_pending_op;
      yc01_request = false;
      yc01_task_state = YC01_TS_BUSY;
      yc01_watchdog = 0;
    }
    xSemaphoreGive(yc01_mutex);

    if (do_request) {
      bool ok = yc01_ble_perform_op((uint8_t)op);
      xSemaphoreTake(yc01_mutex, portMAX_DELAY);
      yc01_task_state = ok ? YC01_TS_OK : YC01_TS_FAIL;
      xSemaphoreGive(yc01_mutex);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void yc01_start_op(uint8_t op) {
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  if (yc01_task_state != YC01_TS_BUSY) {
    yc01_pending_op = (YC01_Op_e)op;
    yc01_request = true;
    yc01_task_state = YC01_TS_BUSY;
    yc01_watchdog = 0;
    yc01_last_try = millis();
  }
  xSemaphoreGive(yc01_mutex);
}

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

static void CmndYC01(void) {
  char macstr[13];
  yc01_mac_to_str(yc01_mac, macstr, sizeof(macstr));
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d,\"Poll\":%d,\"Profile\":\"%s\",\"Boost\":\"%s\",\"Valid\":\"%s\"}}"),
             macstr, yc01_addr_type, yc01_poll_s, yc01_profile.name,
             yc01_boost ? "on" : "off",
             yc01_last.valid ? "true" : "false");
}

static void CmndYC01Read(void) {
  yc01_start_op(YC01_OP_READ);
  ResponseCmndDone();
}

static void CmndYC01Poll(void) {
  int val = XdrvMailbox.payload;
  if (val < 10) val = 10;
  if (val > 600) val = 600;
  yc01_poll_s = (uint16_t)val;
  yc01_tick = 0;
  ResponseCmndNumber(yc01_poll_s);
}

static void CmndYC01Start(void) {
  yc01_start_op(YC01_OP_START);
  ResponseCmndDone();
}

static void CmndYC01Stop(void) {
  yc01_start_op(YC01_OP_STOP);
  ResponseCmndDone();
}

static void CmndYC01Profile(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Profile\":\"%s\",\"Boost\":\"%s\"}}"),
               yc01_profile.name, yc01_boost ? "on" : "off");
    return;
  }
  yc01_load_profile(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Profile\":\"%s\"}}"), yc01_profile.name);
}

static void CmndYC01Boost(void) {
  yc01_boost = XdrvMailbox.payload != 0;
  yc01_load_profile(yc01_profile.name);
  Response_P(PSTR("{\"YC01\":{\"Boost\":\"%s\"}}"), yc01_boost ? "on" : "off");
}

static void CmndYC01Mac(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    char macstr[13];
    yc01_mac_to_str(yc01_mac, macstr, sizeof(macstr));
    Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), macstr, yc01_addr_type);
    return;
  }

  uint8_t newmac[6];
  uint8_t newtype = 0;
  if (!yc01_parse_mac(XdrvMailbox.data, newmac, &newtype)) {
    ResponseCmndFailed();
    return;
  }
  memcpy(yc01_mac, newmac, 6);
  yc01_addr_type = newtype;
  char macstr[13];
  yc01_mac_to_str(yc01_mac, macstr, sizeof(macstr));
  strlcpy(yc01_mac_str, macstr, sizeof(yc01_mac_str));
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), macstr, yc01_addr_type);
}

const char kYC01_Commands[] PROGMEM = D_CMND_YC01 "|"
  "Read|"
  "Poll|"
  "Start|"
  "Stop|"
  "Profile|"
  "Boost|"
  "Mac";

void (*const YC01_Commands[])(void) PROGMEM = {
  &CmndYC01, &CmndYC01Read, &CmndYC01Poll, &CmndYC01Start, &CmndYC01Stop, &CmndYC01Profile, &CmndYC01Boost, &CmndYC01Mac
};

/*********************************************************************************************\
 * Tasmota hooks
\*********************************************************************************************/

static void YC01Init(void) {
  yc01_mutex = xSemaphoreCreateMutex();
  if (!yc01_parse_mac(yc01_mac_str, yc01_mac, &yc01_addr_type)) {
    yc01_parse_mac(YC01_DEFAULT_MAC, yc01_mac, &yc01_addr_type);
  }
  yc01_load_profile("Generic");

  xTaskCreatePinnedToCore(
    yc01_ble_task,
    "YC01BLE",
    8192,
    nullptr,
    1,
    nullptr,
#ifdef CONFIG_FREERTOS_UNICORE
    0
#else
    1
#endif
  );

  AddLog(LOG_LEVEL_INFO, PSTR("YC01: native NimBLE driver initialized, MAC %s"), yc01_mac_str);
}

static void YC01EverySecond(void) {
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);

  if (yc01_task_state == YC01_TS_BUSY) {
    yc01_watchdog++;
  }

  if (yc01_task_state == YC01_TS_OK) {
    yc01_fail_count = 0;
    yc01_retry_at = 0;
    yc01_task_state = YC01_TS_IDLE;
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: read OK"));
  } else if (yc01_task_state == YC01_TS_FAIL) {
    yc01_fail_count++;
    if (yc01_fail_count == 1) {
      yc01_retry_at = millis() + 60000;
    }
    yc01_task_state = YC01_TS_IDLE;
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: read fail #%d"), yc01_fail_count);
  } else if (yc01_task_state == YC01_TS_BUSY && yc01_watchdog > YC01_OP_TIMEOUT_SEC) {
    yc01_task_state = YC01_TS_FAIL;
    yc01_request = false;
    AddLog(LOG_LEVEL_INFO, PSTR("YC01: operation watchdog, aborting"));
  }

  bool can_poll = (yc01_task_state == YC01_TS_IDLE && !yc01_request);

  xSemaphoreGive(yc01_mutex);

  // Scheduled poll (outside mutex to avoid deadlock with yc01_start_op)
  if (can_poll) {
    if (yc01_retry_at && millis() >= yc01_retry_at) {
      yc01_retry_at = 0;
      yc01_start_op(YC01_OP_READ);
    } else {
      yc01_tick++;
      if (yc01_tick >= yc01_poll_s) {
        yc01_tick = 0;
        yc01_start_op(YC01_OP_READ);
      }
    }
  }
}

static void YC01JsonAppend(void) {
  if (!yc01_last.valid) return;

  char macstr[13];
  yc01_mac_to_str(yc01_mac, macstr, sizeof(macstr));

  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_reading_t r = yc01_last;
  xSemaphoreGive(yc01_mutex);

  ResponseAppend_P(PSTR(",\"YC01\":{\"MAC\":\"%s\",\"pH\":%*_f,\"EC\":%d,\"TDS\":%d,\"ORP\":%d,\"SALT\":%*_f,\"Temp\":%*_f,\"Cl\":%*_f,\"Batt\":%d}"),
                   macstr,
                   2, &r.ph,
                   r.ec,
                   r.tds,
                   r.orp,
                   1, &r.salt,
                   1, &r.temp,
                   2, &r.cl,
                   r.batt);
}

static void YC01WebSensor(void) {
  if (!yc01_last.valid) {
    WSContentSend_PD(PSTR("{s}YC01{m}<span style='color:red'>Disconnected</span>{e}"));
    return;
  }

  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_reading_t r = yc01_last;
  yc01_active_profile_t p = yc01_profile;
  xSemaphoreGive(yc01_mutex);

  WSContentSend_PD(PSTR("{s}YC01{m}<span style='color:green'>Connected</span>{e}"));
  WSContentSend_PD(PSTR("{s}pH{m}<span style='color:%s'>%*_f</span>{e}"),
                   yc01_color(r.ph, p.ph_min, p.ph_max, p.ph_margin), 2, &r.ph);
  WSContentSend_PD(PSTR("{s}EC{m}<span style='color:%s'>%d uS/cm</span>{e}"),
                   yc01_color((float)r.ec, p.ec_min, p.ec_max, p.ec_margin), r.ec);
  WSContentSend_PD(PSTR("{s}TDS{m}<span style='color:%s'>%d ppm</span>{e}"),
                   yc01_color((float)r.tds, p.tds_min, p.tds_max, p.tds_margin), r.tds);
  WSContentSend_PD(PSTR("{s}ORP{m}<span style='color:%s'>%d mV</span>{e}"),
                   yc01_color((float)r.orp, p.orp_min, p.orp_max, p.orp_margin), r.orp);
  WSContentSend_PD(PSTR("{s}SALT{m}<span style='color:%s'>%*_f ppm</span>{e}"),
                   yc01_color(r.salt, p.salt_min, p.salt_max, p.salt_margin), 1, &r.salt);
  WSContentSend_PD(PSTR("{s}Temp{m}<span style='color:%s'>%*_f °C</span>{e}"),
                   yc01_color(r.temp, p.temp_min, p.temp_max, p.temp_margin), 1, &r.temp);
  WSContentSend_PD(PSTR("{s}Chlorine{m}<span style='color:%s'>%*_f mg/L</span>{e}"),
                   yc01_color(r.cl, p.cl_min, p.cl_max, p.cl_margin), 2, &r.cl);
  WSContentSend_PD(PSTR("{s}Battery{m}<span style='color:%s'>%d %%</span>{e}"),
                   yc01_color((float)r.batt, 60.0f, 100.0f, 30.0f), r.batt);
}

/*********************************************************************************************\
 * Driver entry point
\*********************************************************************************************/

bool Xdrv89(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      YC01Init();
      break;
    case FUNC_EVERY_SECOND:
      YC01EverySecond();
      break;
    case FUNC_COMMAND:
      result = DecodeCommand(kYC01_Commands, YC01_Commands);
      break;
    case FUNC_JSON_APPEND:
      YC01JsonAppend();
      break;
    case FUNC_WEB_SENSOR:
      YC01WebSensor();
      break;
    case FUNC_ACTIVE:
      result = true;
      break;
  }
  return result;
}

#endif  // USE_YC01_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#endif  // ESP32
