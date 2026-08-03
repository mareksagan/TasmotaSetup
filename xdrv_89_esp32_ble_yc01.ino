/*
  xdrv_89_esp32_ble_yc01.ino - BLE YC01 water quality meter driver for Tasmota

  Native C++ driver for the YC01/YIERYI/YINMIK 6-in-1 water monitor.
  Uses direct NimBLE access. Requires tasmota32-bluetooth build.

  Protocol:
    GATT service 0xFF01, characteristic 0xFF02 (read/write/notify)
    Frame: b[0]=1, b[1]=2, b[2]=15, then BE s16 at offsets 3..15:
      pH   = s16(b[3..4]) / 100
      EC   = s16(b[5..6])
      TDS  = s16(b[7..8])
      ORP  = s16(b[9..10])
      Cl   = max(0, s16(b[11..12])) / 10
      Temp = s16(b[13..14]) / 10
      Batt = clamp(100*(s16(b[15..16])-1950)/1240, 0, 100)
      SALT = EC * 0.55

  Console commands:
    YC01               - show status
    YC01Read           - force immediate read
    YC01Poll N         - set poll interval (30-600s, default 240)
    YC01Profile [name] - set profile or list available
    YC01Boost 0/1      - toggle boost mode
    YC01Mac <addr> [t] - show/set MAC and address type
*/

#ifdef ESP32
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#ifdef USE_YC01_ESP32

#define XDRV_89 89

#include <NimBLEDevice.h>

struct yc01_reading_t {float ph;
  int16_t ec;
  int16_t tds;
  int16_t orp;
  float cl;
  float temp;
  uint8_t batt;
  float salt;
  int16_t rssi;
  bool valid;};

struct yc01_ranges_t {float fmin[8];
  float fmax[8];
  float fmar[8];};

enum YC01_OpState_e {YC01_IDLE = 0,YC01_BUSY,YC01_OK,YC01_FAIL};

#define YC01_SVC_UUID       0xFF01
#define YC01_CHR_UUID       0xFF02
#define YC01_DEFAULT_POLL_S 120
#define YC01_MIN_POLL_S     30
#define YC01_MAX_POLL_S     600
#define YC01_OP_TIMEOUT_S   15
#define YC01_DEFAULT_MAC    "414284588113"
#define YC01_MAX_MAC_LEN    32

#ifndef YC01_DEFAULT_MAC
#define YC01_DEFAULT_MAC "414284588113"
#endif

static const char* const YC01_PF_NAMES[] = {"Generic","CherryTomato","BeefsteakTomato","BellPepper_Poblano_BananaPepper","ChiliPepper_Jalapeno_Cayenne","Habanero","Eggplant","Okra","BushBeans","PoleBeans","Peas_SnowPeas_SugarSnapPeas","Edamame","Cantaloupe_HoneydewMelon_MiniWatermelon","Strawberry_EverbearingStrawberry_AlpineStrawberry","Lettuce_ButterheadLettuce_RomaineLettuce","Spinach_NewZealandSpinach","Kale_CollardGreens","Arugula","Endive_Escarole_Frisee_Radicchio","Mache","Microgreens","BokChoy_Senposai_YukinaSavoy","Tatsoi_Komatsuna_Mibuna","MustardGreens","Mizuna","Watercress","Amaranth","MalabarSpinach","Purslane","Sorrel","Celtuce","SweetPotatoVine","GarlicChives","Basil_ThaiBasil_HolyBasil","Parsley","Cilantro","Dill","Mint_Peppermint_Spearmint","Chives","Oregano","Thyme_Sage_Tarragon_Marjoram","Rosemary_BayLaurel","LemonBalm","Lemongrass","Stevia","Shiso","VietnameseCoriander","Culantro","Epazote","Lovage","SummerSavory","WinterSavory","Lavender","Chamomile","Feverfew_Hyssop_Echinacea","Fennel","Nasturtium_Calendula_Borage_Pansy_Viola","Petunia","GerberaDaisy","Zinnia","Snapdragon","Begonia_Impatiens","SweetAlyssum","Lobelia","Marigold","Dianthus","Cornflower","Portulaca","Ginger_Turmeric","Claytonia","LandCress","WelshOnion","Daylily","FavaBeans","BearsGarlic","Lettuce_Spinach_Kale","Lettuce_Arugula_Chard","Lettuce_Basil_Parsley","Spinach_Kale_Chard","Microgreens1","SaladGreens","Lettuce_Spinach_Arugula_Radish","BokChoy_Tatsoi_Komatsuna1","Mizuna_Mibuna_Senposai","Basil_Mint_Parsley","Cilantro_Dill_Chives","Rosemary_Thyme_Oregano","LemonBalm_Chamomile_Mint","Mediterranean_Herb","Tomato_Pepper_Eggplant","CherryTomato_BellPepper","Pepper_Cucumber","Eggplant_Zucchini_Pepper","Tomato_CherryTomato_Roma","BellPepper_ChiliPepper_Jalapeno","Radish_Turnip_Carrot","GreenOnion_GarlicChives_Leek","Onion_Shallot_Chive","Petunia_Lobelia_Alyssum","BushBeans_PoleBeans_Edamame","Watercress_Purslane_Sorrel","Radicchio_Mache1","BeetGreens_TurnipGreens_CollardGreens","Tomato_Basil","Shiso_VietnameseCoriander_Culantro","Lavender_Rosemary_Sage"};

static const float YC01_PF_DATA[][16] = {
  {5.8,6.2,1000,2500,20,25,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.0,1400,2000,20,25,0.2,300,450,750,6.0,6.3,1800,2800,20,26},
  {5.8,6.0,1400,2000,20,25,0.2,300,450,750,6.0,6.3,2000,3500,20,26},
  {5.8,6.0,1200,1800,22,26,0.2,300,450,750,6.0,6.3,1600,2400,22,26},
  {5.8,6.0,1200,1900,22,26,0.2,300,450,750,6.0,6.3,1600,2600,22,27},
  {5.8,6.0,1200,2000,22,27,0.2,300,450,750,6.0,6.3,1600,2800,22,28},
  {5.8,6.0,1200,1800,22,26,0.2,300,450,750,6.0,6.3,1600,2400,22,26},
  {5.8,6.2,1200,1800,22,27,0.2,300,450,750,6.0,6.5,1400,2200,22,28},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,500,6.0,6.3,1400,2000,18,24},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,500,6.0,6.3,1400,2200,18,24},
  {5.8,6.2,800,1200,15,20,0.2,300,450,500,6.0,6.3,1200,1800,15,20},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,750,6.0,6.3,1400,2000,18,24},
  {5.8,6.2,1200,1600,22,26,0.2,300,450,750,6.0,6.3,1600,2400,22,26},
  {5.5,5.8,1000,1400,15,22,0.1,300,450,500,6.0,6.2,1400,2000,15,22},
  {6.0,6.2,1200,2000,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,10000,0,0,0,0,0,0},
  {6.0,6.2,1500,1800,18,22,0.2,300,450,5000,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,800,1200,15,20,0.2,300,450,7500,0,0,0,0,0,0},
  {5.5,5.8,400,700,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.2,1200,1600,18,22,0.2,300,450,10000,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,10000,0,0,0,0,0,0},
  {6.0,6.5,700,1100,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,20,24,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,22,26,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,1200,1800,20,24,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,1200,1600,18,22,0.2,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,24,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1400,2000,20,24,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,20,24,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.8,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.8,1000,1400,18,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,20,26,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,1400,2000,22,28,0.2,300,450,8000,0,0,0,0,0,0},
  {5.8,6.5,800,1200,10,20,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.5,700,1200,10,20,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,900,1300,10,22,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.5,800,1400,15,24,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.2,800,1200,10,20,0.2,300,450,500,6.0,6.3,1200,1800,10,20},
  {6.0,6.5,800,1200,8,18,0.2,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,1000,1600,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,1000,1600,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,7500,0,0,0,0,0,0},
  {5.8,6.2,500,900,18,22,0.1,300,450,4000,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {5.8,6.5,1200,1600,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,24,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.5,1000,1400,18,22,0.1,300,450,5000,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,20,24,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.3,1600,2400,22,26,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.3,1600,2200,22,26,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.2,1400,2200,22,25,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.3,1600,2200,22,26,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.3,1400,2400,20,26,0.2,300,450,7500,0,0,0,0,0,0},
  {6.0,6.3,1600,2400,22,27,0.2,300,450,7500,0,0,0,0,0,0},
  {5.8,6.2,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0},
  {6.0,6.5,900,1300,18,22,0.1,300,450,500,0,0,0,0,0,0},
  {6.0,6.5,900,1300,18,22,0.1,300,450,500,0,0,0,0,0,0},
  {5.5,6.2,800,1400,15,22,0.2,300,450,500,0,0,0,0,0,0},
  {6.0,6.3,1200,1600,18,24,0.2,300,450,750,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.1,300,450,500,0,0,0,0,0,0},
  {6.0,6.5,800,1200,15,20,0.1,300,450,400,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,750,0,0,0,0,0,0},
  {5.8,6.2,1400,2200,20,26,0.2,300,450,750,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,20,24,0.1,300,450,500,0,0,0,0,0,0},
  {6.0,6.8,1000,1600,18,24,0.2,300,450,750,0,0,0,0,0,0}
};

static const size_t YC01_PF_COUNT = sizeof(YC01_PF_NAMES) / sizeof(YC01_PF_NAMES[0]);

static yc01_reading_t yc01_last = {0};
static yc01_ranges_t yc01_ranges = {};

static char yc01_mac[YC01_MAX_MAC_LEN] = YC01_DEFAULT_MAC;
static uint8_t yc01_addr_type = 0;
static uint16_t yc01_poll_s = YC01_DEFAULT_POLL_S;
static bool yc01_boost = false;
static bool yc01_connected = false;
static int16_t yc01_rssi = 0;
static int16_t yc01_orp_offset = 0;

static volatile YC01_OpState_e yc01_state = YC01_IDLE;
static volatile bool yc01_request = false;
static uint32_t yc01_tick = 0;
static uint32_t yc01_watchdog = 0;
static uint8_t yc01_fails = 0;
static uint32_t yc01_last_try = 0;
static int yc01_last_ok = -1;
static uint32_t yc01_retry_at = 0;
static uint8_t yc01_retry_cnt = 0;

static String yc01_profile_name = "Generic";
static SemaphoreHandle_t yc01_mutex = nullptr;

static int16_t yc01_s16be(const uint8_t *d, size_t i) {uint16_t v = ((uint16_t)d[i] << 8) | d[i + 1];
  if (v > 32767) v -= 65536;
  return (int16_t)v;}

static void yc01_decode(uint8_t *d, size_t len) {
  for (int i = (int)len - 1; i > 0; i--) {uint8_t t1 = d[i];
    uint8_t h1 = (t1 & 0x55) << 1;
    uint8_t l1 = (t1 & 0xAA) >> 1;
    uint8_t t0 = d[i - 1];
    uint8_t h0 = (t0 & 0x55) << 1;
    uint8_t l0 = (t0 & 0xAA) >> 1;
    d[i]     = 0xFF - (h1 | l0);
    d[i - 1] = 0xFF - (h0 | l1);}
}

static void yc01_parse_frame(const uint8_t *d, size_t len, yc01_reading_t *r);
static void yc01_parse_frame(const uint8_t *d, size_t len, yc01_reading_t *r) {if (len < 18) return;
  if (d[0] != 1 || d[2] != 15) return;
  r->ph   = yc01_s16be(d,3) / 100.0f;
  r->ec   = yc01_s16be(d,5);
  r->tds  = yc01_s16be(d,7);
  r->orp  = yc01_s16be(d,9);
  int16_t cl_raw = yc01_s16be(d,11);
  r->cl   = (cl_raw < 0 ? 0.0f : cl_raw / 10.0f);
  r->temp = yc01_s16be(d,13) / 10.0f;
  int32_t batt_raw = yc01_s16be(d,15);
  int32_t batt = (100 * (batt_raw - 1950)) / (3190 - 1950);
  if (batt < 0) batt = 0;
  if (batt > 100) batt = 100;
  r->batt = (uint8_t)batt;
  r->salt = r->ec * 0.55f;
  r->valid = true;}

static void yc01_parse_mac(const char *src, uint8_t *dst, uint8_t *type);
static void yc01_parse_mac(const char *src, uint8_t *dst, uint8_t *type) {
  char buf[YC01_MAX_MAC_LEN];
  strlcpy(buf, src, sizeof(buf));
  char *slash = strchr(buf, '/');
  if (slash) {*slash = 0;
    if (type) *type = (uint8_t)atoi(slash + 1);} else {if (type) *type = 0;}
  char clean[13] = {0};
  int j = 0;
  for (int i = 0; buf[i] && j < 12; i++) {
    char c = buf[i];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {clean[j++] = c;}
  }
  if (j != 12) return;
  for (int i = 0; i < 6; i++) {
    char hex[3] = {clean[i * 2],clean[i * 2 + 1],0};
    dst[i] = (uint8_t)strtol(hex, nullptr, 16);
  }
}

static void yc01_mac_to_str(const uint8_t *mac, char *out, size_t out_len) {snprintf(out,out_len,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);}

static void yc01_apply_profile(const float *v);
static void yc01_apply_profile(const float *v) {
  yc01_ranges.fmin[0] = v[0];
  yc01_ranges.fmax[0] = v[1];
  yc01_ranges.fmin[1] = v[2];
  yc01_ranges.fmax[1] = v[3];
  yc01_ranges.fmin[2] = v[2] * 0.64f;
  yc01_ranges.fmax[2] = v[3] * 0.64f;
  yc01_ranges.fmin[3] = v[7];
  yc01_ranges.fmax[3] = v[8];
  yc01_ranges.fmin[4] = 0.0f;
  yc01_ranges.fmax[4] = v[6];
  yc01_ranges.fmin[5] = v[4];
  yc01_ranges.fmax[5] = v[5];
  yc01_ranges.fmin[6] = 0.0f;
  yc01_ranges.fmax[6] = v[9] / 10.0f;
  yc01_ranges.fmin[7] = 60.0f;
  yc01_ranges.fmax[7] = 100.0f;
  for (int i = 0; i < 8; i++) {yc01_ranges.fmar[i] = (yc01_ranges.fmax[i] - yc01_ranges.fmin[i]) * 0.15f;
    if (i == 0 && yc01_ranges.fmar[i] < 0.1f) yc01_ranges.fmar[i] = 0.1f;
    if (i == 1 && yc01_ranges.fmar[i] < 200.0f) yc01_ranges.fmar[i] = 200.0f;
    if (i == 2 && yc01_ranges.fmar[i] < 128.0f) yc01_ranges.fmar[i] = 128.0f;
    if (i == 3 && yc01_ranges.fmar[i] < 50.0f) yc01_ranges.fmar[i] = 50.0f;
    if (i == 5 && yc01_ranges.fmar[i] < 2.0f) yc01_ranges.fmar[i] = 2.0f;
    if (i == 6 && yc01_ranges.fmar[i] < 125.0f) yc01_ranges.fmar[i] = 125.0f;}
}

static void yc01_apply_boost(const float *v);
static void yc01_apply_boost(const float *v) {if (v[10] <= 0.0f) return;
  yc01_ranges.fmin[0] = v[10];
  yc01_ranges.fmax[0] = v[11];
  yc01_ranges.fmin[1] = v[12];
  yc01_ranges.fmax[1] = v[13];
  yc01_ranges.fmin[2] = v[12] * 0.64f;
  yc01_ranges.fmax[2] = v[13] * 0.64f;
  yc01_ranges.fmin[5] = v[14];
  yc01_ranges.fmax[5] = v[15];}

static const char* yc01_color(float v, int idx) {if (idx < 0 || idx > 7) return "green";
  float mn = yc01_ranges.fmin[idx];
  float mx = yc01_ranges.fmax[idx];
  float mar = yc01_ranges.fmar[idx];
  if (v < mn - mar || v > mx + mar) return "red";
  if (v < mn || v > mx) return "orange";
  return "green";}

static int yc01_find_profile(const char *name) {
  for (size_t i = 0; i < YC01_PF_COUNT; i++) {if (strcmp(YC01_PF_NAMES[i],name) == 0) return (int)i;}
  return -1;
}

static void yc01_load_profile(const char *name);
static void yc01_load_profile(const char *name) {
  int idx = yc01_find_profile(name);
  if (idx < 0) {AddLog(LOG_LEVEL_INFO,PSTR("YC01: profile '%s' not found,using Generic"),name);
    idx = 0;}
  yc01_profile_name = YC01_PF_NAMES[idx];
  yc01_apply_profile(YC01_PF_DATA[idx]);
  if (yc01_boost && YC01_PF_DATA[idx][10] > 0) {yc01_apply_boost(YC01_PF_DATA[idx]);} else if (yc01_boost) {yc01_boost = false;}
}

static SemaphoreHandle_t yc01_ble_sem = nullptr;

static int yc01_ble_op_complete(BLE_ESP32::generic_sensor_t *op) {
  if (op->context != (void*)0x01) return 0;
  if (op->readlen >= 18) {
    uint8_t buf[32];
    size_t len = op->readlen > 32 ? 32 : op->readlen;
    memcpy(buf, op->dataRead, len);
    yc01_decode(buf, len);
    yc01_reading_t r;
    yc01_parse_frame(buf, len, &r);
    if (r.valid) {
      r.rssi = yc01_rssi;
      r.orp -= yc01_orp_offset;
      xSemaphoreTake(yc01_mutex, portMAX_DELAY);
      yc01_last = r;
      yc01_connected = true;
      xSemaphoreGive(yc01_mutex);
    }
  } else if (op->state < 0) {
    xSemaphoreTake(yc01_mutex, portMAX_DELAY);
    yc01_connected = false;
    xSemaphoreGive(yc01_mutex);
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: BLE op failed state=%d"), op->state);
  }
  xSemaphoreGive(yc01_ble_sem);
  return 1;
}

static bool yc01_ble_do_read(void) {
  if (!NimBLEDevice::isInitialized()) {
    AddLog(LOG_LEVEL_INFO, PSTR("YC01: BLE not enabled, skipping read"));
    return false;
  }
  BLE_ESP32::generic_sensor_t *op = nullptr;
  int res = BLE_ESP32::newOperation(&op);
  if (!res || !op) return false;
  uint8_t mac[6];
  yc01_parse_mac(yc01_mac, mac, nullptr);
  op->addr = NimBLEAddress(mac, yc01_addr_type);
  op->serviceUUID = NimBLEUUID((uint16_t)YC01_SVC_UUID);
  op->characteristicUUID = NimBLEUUID((uint16_t)YC01_CHR_UUID);
  op->readlen = 1;
  op->completecallback = (void*)&yc01_ble_op_complete;
  op->context = (void*)0x01;
  res = BLE_ESP32::extQueueOperation(&op);
  if (!res) {
    BLE_ESP32::freeOperation(&op);
    return false;
  }
  xSemaphoreTake(yc01_ble_sem, 0);
  if (xSemaphoreTake(yc01_ble_sem, pdMS_TO_TICKS(40000)) != pdTRUE) return false;
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  bool valid = yc01_last.valid;
  xSemaphoreGive(yc01_mutex);
  return valid;
}

static void yc01_ble_task(void *pv);
static void yc01_ble_task(void *pv) {
  while (true) {
    bool do_req = false;
    xSemaphoreTake(yc01_mutex, portMAX_DELAY);
    if (yc01_request) {do_req = true;
      yc01_request = false;
      yc01_state = YC01_BUSY;
      yc01_watchdog = 0;
      yc01_last_try = millis();}
    xSemaphoreGive(yc01_mutex);
    if (do_req) {bool ok = yc01_ble_do_read();
      xSemaphoreTake(yc01_mutex,portMAX_DELAY);
      yc01_state = ok ? YC01_OK : YC01_FAIL;
      xSemaphoreGive(yc01_mutex);}
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

static void yc01_start_op(void);
static void yc01_start_op(void) {
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  if (yc01_state != YC01_BUSY) {yc01_request = true;
    yc01_state = YC01_BUSY;
    yc01_watchdog = 0;
    yc01_last_try = millis();}
  xSemaphoreGive(yc01_mutex);
}

static void CmndYC01(void);
static void CmndYC01Read(void);
static void CmndYC01Poll(void);
static void CmndYC01Profile(void);
static void CmndYC01Boost(void);
static void CmndYC01Mac(void);
static void YC01Init(void);
static void YC01EverySecond(void);
static void YC01JsonAppend(void);
static void YC01WebSensor(void);

static void CmndYC01(void) {
  char macstr[13];
  uint8_t mac[6];
  yc01_parse_mac(yc01_mac, mac, nullptr);
  yc01_mac_to_str(mac, macstr, sizeof(macstr));
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d,\"Poll\":%d,\"Profile\":\"%s\",\"Boost\":\"%s\",\"Valid\":\"%s\"}}"),
             macstr, yc01_addr_type, yc01_poll_s, yc01_profile_name.c_str(),
             yc01_boost ? "on" : "off",
             yc01_last.valid ? "true" : "false");
}

static void CmndYC01Read(void) {yc01_start_op();
  ResponseCmndDone();}

static void CmndYC01Poll(void) {int val = XdrvMailbox.payload;
  if (val < YC01_MIN_POLL_S) val = YC01_MIN_POLL_S;
  if (val > YC01_MAX_POLL_S) val = YC01_MAX_POLL_S;
  yc01_poll_s = (uint16_t)val;
  yc01_tick = 0;
  ResponseCmndNumber(yc01_poll_s);}

static void CmndYC01Profile(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01Profile\":["));
    int line_len = 0;
    for (size_t i = 0; i < YC01_PF_COUNT; i++) {
      if (i > 0) {ResponseAppend_P(PSTR(","));
        line_len += 1;}
      int name_len = strlen(YC01_PF_NAMES[i]);
      if (line_len > 0 && line_len + name_len > 100) {ResponseAppend_P(PSTR("\n"));
        line_len = 0;}
      ResponseAppend_P(PSTR("\"%s\""), YC01_PF_NAMES[i]);
      line_len += name_len + 2;
    }
    ResponseAppend_P(PSTR("\n]}"));
    return;
  }
  yc01_load_profile(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Profile\":\"%s\"}}"), yc01_profile_name.c_str());
}

static void CmndYC01Boost(void) {
  bool want_boost = XdrvMailbox.payload != 0;
  if (want_boost) {
    int idx = yc01_find_profile(yc01_profile_name.c_str());
    if (idx >= 0 && YC01_PF_DATA[idx][10] <= 0.0f) {
      Response_P(PSTR("{\"YC01\":{\"Boost\":\"off\",\"Error\":\"boost not available for profile '%s\"}}"),
                 yc01_profile_name.c_str());
      return;
    }
  }
  yc01_boost = want_boost;
  yc01_load_profile(yc01_profile_name.c_str());
  Response_P(PSTR("{\"YC01\":{\"Boost\":\"%s\"}}"), yc01_boost ? "on" : "off");
}

static void CmndYC01Mac(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    char macstr[13];
    uint8_t mac[6];
    yc01_parse_mac(yc01_mac, mac, nullptr);
    yc01_mac_to_str(mac, macstr, sizeof(macstr));
    Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), macstr, yc01_addr_type);
    return;
  }
  uint8_t newmac[6];
  uint8_t newtype = 0;
  yc01_parse_mac(XdrvMailbox.data, newmac, &newtype);
  yc01_mac_to_str(newmac, yc01_mac, sizeof(yc01_mac));
  yc01_addr_type = newtype;
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), yc01_mac, yc01_addr_type);
}

const char kYC01_Commands[] PROGMEM = "YC01|"
  "|"
  "Read|"
  "Poll|"
  "Profile|"
  "Boost|"
  "Mac";

void (*const YC01_Commands[])(void) PROGMEM = {&CmndYC01,&CmndYC01Read,&CmndYC01Poll,&CmndYC01Profile,&CmndYC01Boost,&CmndYC01Mac};

static void YC01Init(void) {
  yc01_mutex = xSemaphoreCreateMutex();
  yc01_ble_sem = xSemaphoreCreateBinary();
  yc01_last_ok = -1;
  yc01_tick = yc01_poll_s - 10;
  if (yc01_find_profile(yc01_profile_name.c_str()) < 0) {yc01_profile_name = "Generic";}
  yc01_load_profile(yc01_profile_name.c_str());
  xTaskCreatePinnedToCore(yc01_ble_task, "YC01BLE", 8192, nullptr, 1, nullptr,
#ifdef CONFIG_FREERTOS_UNICORE
    0
#else
    1
#endif
  );
  AddLog(LOG_LEVEL_INFO, PSTR("YC01: driver v1.0 started, MAC %s, profile %s"),
         yc01_mac, yc01_profile_name.c_str());
}

static void YC01EverySecond(void) {
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  if (yc01_state == YC01_BUSY) {yc01_watchdog++;}
  if (yc01_state == YC01_OK) {yc01_fails = 0;
    yc01_retry_cnt = 0;
    yc01_last_ok = millis() / 1000;
    yc01_state = YC01_IDLE;} else if (yc01_state == YC01_FAIL) {
    yc01_fails++;
    if (yc01_retry_cnt < 2) {yc01_retry_cnt++;
      yc01_retry_at = millis() / 1000 + 3;}
    yc01_state = YC01_IDLE;
  }
  bool can_poll = (yc01_state == YC01_IDLE && !yc01_request);
  xSemaphoreGive(yc01_mutex);
  if (can_poll) {
    if (yc01_retry_cnt > 0 && millis() / 1000 >= yc01_retry_at) {yc01_retry_at = millis() / 1000 + 3;
      yc01_start_op();
      return;}
    yc01_tick++;
    if (yc01_tick >= yc01_poll_s) {yc01_tick = 0;
      yc01_start_op();}
  }
}

static void YC01JsonAppend(void) {
  if (!yc01_last.valid) return;
  char macstr[13];
  uint8_t mac[6];
  yc01_parse_mac(yc01_mac, mac, nullptr);
  yc01_mac_to_str(mac, macstr, sizeof(macstr));
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_reading_t r = yc01_last;
  xSemaphoreGive(yc01_mutex);
  ResponseAppend_P(PSTR(",\"YC01\":{\"MAC\":\"%s\",\"pH\":%*_f,\"EC\":%d,\"TDS\":%d,\"ORP\":%d,\"SALT\":%*_f,\"Temp\":%*_f,\"Cl\":%*_f,\"Batt\":%d}"),
                   macstr, 2, &r.ph, r.ec, r.tds, r.orp, 1, &r.salt, 1, &r.temp, 2, &r.cl, r.batt);
}

static void yc01_format_profile(const char *name, char *out, size_t out_len) {
  size_t j = 0;
  bool prev_lower = false;
  for (size_t i = 0; name[i] && j < out_len - 1; i++) {
    char c = name[i];
    if (c == '_') {
      if (j < out_len - 3) {out[j++] = ' '; out[j++] = '/'; out[j++] = ' ';}
      prev_lower = false;
    } else if (c >= 'A' && c <= 'Z' && prev_lower) {
      if (j < out_len - 1) {out[j++] = ' '; out[j++] = c;}
      prev_lower = true;
    } else {
      out[j++] = c;
      prev_lower = (c >= 'a' && c <= 'z');
    }
  }
  out[j] = '\0';
}

static void YC01WebSensor(void) {
  if (!yc01_last.valid) {
    WSContentSend_PD(PSTR("{s}YC01{m}<span style='color:red'>Disconnected</span>{e}"));
    return;
  }
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_reading_t r = yc01_last;
  xSemaphoreGive(yc01_mutex);
  WSContentSend_PD(PSTR("{s}YC01{m}<span style='color:green'>Connected</span>{e}"));
  char pf_fmt[48];
  yc01_format_profile(yc01_profile_name.c_str(), pf_fmt, sizeof(pf_fmt));
  WSContentSend_PD(PSTR("{s}Profile{m}%s{e}"), pf_fmt);
  WSContentSend_PD(PSTR("{s}pH{m}<span style='color:%s'>%*_f</span>{e}"),
                   yc01_color(r.ph, 0), 2, &r.ph);
  WSContentSend_PD(PSTR("{s}EC{m}<span style='color:%s'>%d uS/cm</span>{e}"),
                   yc01_color((float)r.ec, 1), r.ec);
  WSContentSend_PD(PSTR("{s}TDS{m}<span style='color:%s'>%d ppm</span>{e}"),
                   yc01_color((float)r.tds, 2), r.tds);
  WSContentSend_PD(PSTR("{s}ORP{m}<span style='color:%s'>%d mV</span>{e}"),
                   yc01_color((float)r.orp, 3), r.orp);
  WSContentSend_PD(PSTR("{s}SALT{m}<span style='color:%s'>%*_f ppm</span>{e}"),
                   yc01_color(r.salt, 6), 1, &r.salt);
  WSContentSend_PD(PSTR("{s}Temp{m}<span style='color:%s'>%*_f °C</span>{e}"),
                   yc01_color(r.temp, 5), 1, &r.temp);
  WSContentSend_PD(PSTR("{s}Chlorine{m}<span style='color:%s'>%*_f mg/L</span>{e}"),
                   yc01_color(r.cl, 4), 2, &r.cl);
  WSContentSend_PD(PSTR("{s}Battery{m}<span style='color:%s'>%d %%</span>{e}"),
                   yc01_color((float)r.batt, 7), r.batt);
}

bool Xdrv89(uint32_t function) {
  bool result = false;
  switch (function) {case FUNC_INIT:         YC01Init(); break;
    case FUNC_EVERY_SECOND: YC01EverySecond(); break;
    case FUNC_COMMAND:      result = DecodeCommand(kYC01_Commands,YC01_Commands); break;
    case FUNC_JSON_APPEND:  YC01JsonAppend(); break;
    case FUNC_WEB_SENSOR:   YC01WebSensor(); break;
    case FUNC_ACTIVE:       result = true; break;}
  return result;
}

#endif  // USE_YC01_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#endif  // ESP32
