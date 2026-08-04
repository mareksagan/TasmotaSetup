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
#include <HttpClientLight.h>
#include <WiFiClient.h>

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

#define YC01_SVC_UUID       0xFF01
#define YC01_CHR_UUID       0xFF02
#define YC01_DEFAULT_POLL_S 240
#define YC01_MIN_POLL_S     30
#define YC01_MAX_POLL_S     600
#define YC01_OP_TIMEOUT_S   15
#define YC01_DEFAULT_MAC    "414284588113"
#define YC01_MAX_MAC_LEN    32

#ifndef YC01_DEFAULT_MAC
#define YC01_DEFAULT_MAC "414284588113"
#endif

static const char* const YC01_PF_NAMES[] = {"Generic","CherryTomato","BeefsteakTomato","BellPepper_Poblano_BananaPepper","ChiliPepper_Jalapeno_Cayenne","Habanero","Eggplant","Okra","BushBeans","PoleBeans","Peas_SnowPeas_SugarSnapPeas","Edamame","Cantaloupe_HoneydewMelon_MiniWatermelon","Strawberry_EverbearingStrawberry_AlpineStrawberry","Lettuce_ButterheadLettuce_RomaineLettuce","Spinach_NewZealandSpinach","Kale_CollardGreens","Arugula","Endive_Escarole_Frisee_Radicchio","Mache","Microgreens","BokChoy_Senposai_YukinaSavoy","Tatsoi_Komatsuna_Mibuna","MustardGreens","Mizuna","Watercress","Amaranth","MalabarSpinach","Purslane","Sorrel","Celtuce","SweetPotatoVine","GarlicChives","Basil_ThaiBasil_HolyBasil","Parsley","Cilantro","Dill","Mint_Peppermint_Spearmint","Chives","Oregano","Thyme_Sage_Tarragon_Marjoram","Rosemary_BayLaurel","LemonBalm","Lemongrass","Stevia","Shiso","VietnameseCoriander","Culantro","Epazote","Lovage","SummerSavory","WinterSavory","Lavender","Chamomile","Feverfew_Hyssop_Echinacea","Fennel","Nasturtium_Calendula_Borage_Pansy_Viola","Petunia","GerberaDaisy","Zinnia","Snapdragon","Begonia_Impatiens","SweetAlyssum","Lobelia","Marigold","Dianthus","Cornflower","Portulaca","Ginger_Turmeric","Claytonia","LandCress","WelshOnion","Daylily","FavaBeans","BearsGarlic","BasilMintParsleyMix","BeetGreensTurnipGreensCollardGreensMix","Beetroot","BellPepperChiliPepperJalapenoMix","BokChoyTatsoiKomatsunaMix","Broccoli_Cauliflower_Cabbage_BrusselsSprouts_Kohlrabi","BushBeansPoleBeansEdamameMix","Carrot_SmallVariety","Celery","CherryTomatoBellPepperMix","CilantroDillChivesMix","Cucumber","EggplantZucchiniPepperMix","GreenOnionGarlicChivesLeekMix","GreenOnion_OnionGreens_ShallotGreens","LavenderRosemarySageMix","Leek","LemonBalmChamomileMintMix","LettuceArugulaChardMix","LettuceBasilParsleyMix","LettuceSpinachArugulaRadishMix","LettuceSpinachKaleMix","MediterraneanHerbMix","MicrogreensMix","MizunaMibunaSenposaiMix","OnionShallotChiveMix","PepperCucumberMix","PetuniaLobeliaAlyssumMix","Potato_SmallAeroponic","RadicchioMacheMix","Radish","RadishTurnipCarrotMix","RosemaryThymeOreganoMix","SaladGreensMix","ShisoVietnameseCorianderCulantroMix","SpinachKaleChardMix","SwissChard_NapaCabbage_TurnipGreens_BeetGreens","TomatoBasilMixCompanion","TomatoCherryTomatoRomaMix","TomatoPepperEggplantMix","Tomato_RomaTomato_HeirloomTomato","Turnip","WatercressPurslaneSorrelMix","Zucchini_SummerSquash_PattypanSquash"};

static const float YC01_PF_DATA[][17] = {
{5.8,6.2,1000,2500,20,25,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.0,1400,2000,20,25,0.2,300,450,750,1,6.0,6.3,1800.0,2800.0,20.0,26.0},
  {5.8,6.0,1400,2000,20,25,0.2,300,450,750,1,6.0,6.3,2000.0,3500.0,20.0,26.0},
  {5.8,6.0,1200,1800,22,26,0.2,300,450,750,1,6.0,6.3,1600.0,2400.0,22.0,26.0},
  {5.8,6.0,1200,1900,22,26,0.2,300,450,750,1,6.0,6.3,1600.0,2600.0,22.0,27.0},
  {5.8,6.0,1200,2000,22,27,0.2,300,450,750,1,6.0,6.3,1600.0,2800.0,22.0,28.0},
  {5.8,6.0,1200,1800,22,26,0.2,300,450,750,1,6.0,6.3,1600.0,2400.0,22.0,26.0},
  {5.8,6.2,1200,1800,22,27,0.2,300,450,750,1,6.0,6.5,1400.0,2200.0,22.0,28.0},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,500,1,6.0,6.3,1400.0,2000.0,18.0,24.0},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,500,1,6.0,6.3,1400.0,2200.0,18.0,24.0},
  {5.8,6.2,800,1200,15,20,0.2,300,450,500,1,6.0,6.3,1200.0,1800.0,15.0,20.0},
  {5.8,6.2,1000,1400,18,24,0.2,300,450,750,1,6.0,6.3,1400.0,2000.0,18.0,24.0},
  {5.8,6.2,1200,1600,22,26,0.2,300,450,750,1,6.0,6.3,1600.0,2400.0,22.0,26.0},
  {5.5,5.8,1000,1400,15,22,0.1,300,450,500,1,6.0,6.2,1400.0,2000.0,15.0,22.0},
  {6.0,6.2,1200,2000,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.2,1500,1800,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,800,1200,15,20,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,5.8,400,700,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.2,1200,1600,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.5,700,1100,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,20,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,22,26,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,20,24,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1200,1800,20,24,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1000,1400,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1200,1600,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1400,2000,20,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1600,20,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,20,24,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,20,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.8,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,6.0,1000,1600,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,1600,2200,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,1400,2000,20,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,1200,1800,15,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,6.2,800,1400,18,24,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,800,1200,15,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.5,6.2,800,1200,15,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,20,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1400,2000,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.8,1000,1600,15,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.5,800,1400,20,28,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,6.5,1400,2000,22,28,0.2,300,450,800,0,0,0,0,0,0,0},
  {5.8,6.5,800,1200,10,20,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.5,700,1200,10,20,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,900,1300,10,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,800,1400,15,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,800,1200,10,20,0.2,300,450,500,1,6.0,6.3,1200,1800,10,20},
  {6.0,6.5,800,1200,8,18,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.5,1200,1600,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,24,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.3,1600,2400,22,27,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1400,2000,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.3,1200,1600,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,6.2,1000,1400,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1800,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.3,1600,2200,22,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.5,5.8,1200,1600,20,24,0.2,300,450,750,1,5.8,6.0,1600.0,2400.0,20.0,25.0},
  {6.0,6.3,1600,2200,22,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,900,1300,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,900,1300,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.8,1000,1600,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.5,1000,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1000,1600,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1000,1600,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,20,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,500,900,18,22,0.1,300,450,400,0,0,0,0,0,0,0},
  {6.0,6.2,1000,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,900,1300,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.2,1400,2200,22,25,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.5,6.2,800,1400,15,22,0.2,300,450,500,0,0,0,0,0,0,0},
  {5.5,6.0,1600,2400,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.5,800,1200,15,20,0.1,300,450,400,0,0,0,0,0,0,0},
  {5.5,6.2,800,1200,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {5.8,6.2,1000,1400,18,22,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,24,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.2,900,1400,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1000,1400,20,24,0.1,300,450,500,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.1,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.5,1200,1600,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {5.8,6.2,1400,2200,20,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.3,1400,2400,20,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {6.0,6.3,1600,2400,22,26,0.2,300,450,750,0,0,0,0,0,0,0},
  {5.8,6.0,1400,2000,20,25,0.2,300,450,750,1,6.0,6.3,2000.0,3200.0,20.0,26.0},
  {6.0,6.5,1000,1600,18,22,0.2,300,450,1000,0,0,0,0,0,0,0},
  {6.0,6.5,800,1200,18,22,0.1,300,450,500,0,0,0,0,0,0,0},
  {5.8,6.0,1200,1800,20,25,0.2,300,450,750,1,6.0,6.3,1600.0,2400.0,20.0,25.0}
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

static String yc01_profile_name = "Generic";
static SemaphoreHandle_t yc01_mutex = nullptr;
static uint8_t yc01_mac_bytes[6] = {0};
static String yc01_location = "";
static String yc01_action_msg = "";
static String yc01_notes_msg = "";
static int yc01_pump_relay = 0;
static int yc01_heater_relay = 0;
static bool yc01_pump_auto = false;
static bool yc01_heater_auto = false;
static bool yc01_pump_state = false;
static bool yc01_heater_state = false;
static float yc01_lat = 0.0f;
static float yc01_lon = 0.0f;
static int yc01_sunrise_min = 360;
static int yc01_sunset_min = 1080;
static int yc01_outside_temp = 99;

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

static int yc01_read_ok_cnt = 0;
static SemaphoreHandle_t yc01_sem = nullptr;
static volatile bool yc01_op_trigger = false;

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
      r.rssi = 0;
      r.orp -= yc01_orp_offset;
      xSemaphoreTake(yc01_mutex, portMAX_DELAY);
      yc01_last = r;
      yc01_connected = true;
      xSemaphoreGive(yc01_mutex);
      yc01_read_ok_cnt++;
      if (yc01_read_ok_cnt <= 3 || (yc01_read_ok_cnt % 10 == 0)) {
        AddLog(LOG_LEVEL_INFO, PSTR("YC01: read OK #%d pH=%.2f EC=%d TDS=%d Temp=%.1f Batt=%d"), yc01_read_ok_cnt, r.ph, r.ec, r.tds, r.temp, r.batt);
      }
      xSemaphoreGive(yc01_sem);
      return 1;
    }
  } else if (op->state < 0) {
    xSemaphoreTake(yc01_mutex, portMAX_DELAY);
    yc01_connected = false;
    xSemaphoreGive(yc01_mutex);
    AddLog(LOG_LEVEL_DEBUG, PSTR("YC01: BLE op failed state=%d"), op->state);
  }
  xSemaphoreGive(yc01_sem);
  return 1;
}

static void yc01_queue_read(void) {
  BLE_ESP32::generic_sensor_t *op = nullptr;
  int res = BLE_ESP32::newOperation(&op);
  if (!res || !op) return;
  uint8_t mac[6];
  yc01_parse_mac(yc01_mac, mac, nullptr);
  op->addr = NimBLEAddress(mac, yc01_addr_type);
  op->serviceUUID = NimBLEUUID((uint16_t)YC01_SVC_UUID);
  op->characteristicUUID = NimBLEUUID((uint16_t)YC01_CHR_UUID);
  op->readlen = 1;
  op->completecallback = (void*)&yc01_ble_op_complete;
  op->context = (void*)0x01;
  yc01_op_trigger = true;
  BLE_ESP32::extQueueOperation(&op);
}

static void yc01_ble_task(void *pv);
static void yc01_ble_task(void *pv) {
  while (true) {
    if (NimBLEDevice::isInitialized()) {
      yc01_queue_read();
      xSemaphoreTake(yc01_sem, pdMS_TO_TICKS(30000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
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

static void CmndYC01Read(void) {
  yc01_op_trigger = true;
  ResponseCmndDone();}

static void CmndYC01Poll(void) {int val = XdrvMailbox.payload;
  if (val < YC01_MIN_POLL_S) val = YC01_MIN_POLL_S;
  if (val > YC01_MAX_POLL_S) val = YC01_MAX_POLL_S;
  yc01_poll_s = (uint16_t)val;
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
  String warn = "";
  if (yc01_location.length() > 0) {
    int idx = yc01_find_profile(XdrvMailbox.data);
    if (idx > 0) {
      float tmin = YC01_PF_DATA[idx][4];
      float tmax = YC01_PF_DATA[idx][5];
      warn = " Note: check season for this profile (temp " + String(tmin, 0) + "-" + String(tmax, 0) + " C) in " + yc01_location.c_str() + ".";
    }
  }
  yc01_load_profile(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Profile\":\"%s\"%s}}"), yc01_profile_name.c_str(), warn.c_str());
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

static void CmndYC01Location(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Location\":\"%s\"}}"), yc01_location.c_str());
    return;
  }
  yc01_location = XdrvMailbox.data;
  yc01_geocode_city(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Location\":\"%s\",\"Lat\":%.4f,\"Lon\":%.4f,\"Sunrise\":%d,\"Sunset\":%d}}"),
             yc01_location.c_str(), yc01_lat, yc01_lon, yc01_sunrise_min, yc01_sunset_min);
}

const char kYC01_Commands[] PROGMEM = "YC01|"
  "|"
  "Read|"
  "Poll|"
  "Profile|"
  "Boost|"
  "Mac|"
  "Location|"
  "LatLon|"
  "Pump|"
  "Heater";

void (*const YC01_Commands[])(void) PROGMEM = {&CmndYC01,&CmndYC01Read,&CmndYC01Poll,&CmndYC01Profile,&CmndYC01Boost,&CmndYC01Mac,&CmndYC01Location,&CmndYC01LatLon,&CmndYC01Pump,&CmndYC01Heater};

static void YC01Init(void) {
  yc01_mutex = xSemaphoreCreateMutex();
  yc01_sem = xSemaphoreCreateBinary();
  if (yc01_find_profile(yc01_profile_name.c_str()) < 0) {yc01_profile_name = "Generic";}
  yc01_load_profile(yc01_profile_name.c_str());
  yc01_parse_mac(yc01_mac, yc01_mac_bytes, nullptr);
  AddLog(LOG_LEVEL_INFO, PSTR("YC01: driver v1.0 started, MAC %s, profile %s"), yc01_mac, yc01_profile_name.c_str());
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
  ResponseAppend_P(PSTR(",\"YC01\":{\"MAC\":\"%s\",\"pH\":%*_f,\"EC\":%d,\"TDS\":%d,\"ORP\":%d,\"SALT\":%*_f,\"Temp\":%*_f,\"Cl\":%*_f,\"Batt\":\"%s\"}"),
                   macstr, 2, &r.ph, r.ec, r.tds, r.orp, 1, &r.salt, 1, &r.temp, 2, &r.cl, r.batt >= 84 ? "High" : "Low");
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
  yc01_calc_action();
  WSContentSend_PD(PSTR("{s}YC01{m}<span style='color:green'>Connected</span>{e}"));
  char pf_fmt[48];
  yc01_format_profile(yc01_profile_name.c_str(), pf_fmt, sizeof(pf_fmt));
  WSContentSend_PD(PSTR("{s}Profile{m}%s{e}"), pf_fmt);
  WSContentSend_PD(PSTR("{s}Blooming boost{m}%s{e}"), yc01_boost ? "Yes" : "No");
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
  const char* batt_color = r.batt >= 84 ? "green" : "red";
  const char* batt_txt = r.batt >= 84 ? "High" : "Low";
  WSContentSend_PD(PSTR("{s}Battery charge{m}<span style='color:%s'>%s</span>{e}"),
                   batt_color, batt_txt);
  WSContentSend_PD(PSTR("{s}Action{m}%s{e}"), yc01_action_msg.c_str());
  if (yc01_notes_msg.length() > 0) {
    WSContentSend_PD(PSTR("{s}Notes{m}%s{e}"), yc01_notes_msg.c_str());
  }
  if (yc01_pump_relay > 0) {
    char pump_txt[32];
    snprintf(pump_txt, sizeof(pump_txt), "Pump (relay %d): %s%s", yc01_pump_relay, yc01_pump_state ? "ON" : "OFF", yc01_pump_auto ? " [auto]" : "");
    WSContentSend_PD(PSTR("{s}%s{m}%s{e}"), yc01_pump_auto ? "Pump auto" : "Pump", pump_txt);
  }
  if (yc01_heater_relay > 0) {
    char heat_txt[32];
    snprintf(heat_txt, sizeof(heat_txt), "Heater (relay %d): %s%s", yc01_heater_relay, yc01_heater_state ? "ON" : "OFF", yc01_heater_auto ? " [auto]" : "");
    WSContentSend_PD(PSTR("{s}%s{m}%s{e}"), yc01_heater_auto ? "Heater auto" : "Heater", heat_txt);
  }
}

static void yc01_calc_action(void) {
  if (!yc01_last.valid) {
    yc01_action_msg = "<span style='color:gray'>No data</span>";
    return;
  }
  yc01_reading_t r = yc01_last;
  float ph_lo = yc01_ranges.fmin[0];
  float ph_hi = yc01_ranges.fmax[0];
  float ec_lo = yc01_ranges.fmin[1];
  float ec_hi = yc01_ranges.fmax[1];
  float ph_diff_hi = r.ph - ph_hi;
  float ph_diff_lo = ph_lo - r.ph;
  String issues = "";
  if (ph_diff_hi > 1.0f) issues += "pH HIGH: add pH down (phosphoric acid)";
  else if (ph_diff_hi > 0.3f) issues += "pH high: +1ml pH down/10L";
  else if (ph_diff_lo > 1.0f) issues += "pH LOW: add KOH solution";
  else if (ph_diff_lo > 0.3f) issues += "pH low: +1ml pH up/10L";
  if ((float)r.ec > ec_hi * 1.2f) { if (issues.length()) issues += "<br>"; issues += "EC HIGH: add water"; }
  else if ((float)r.ec < ec_lo * 0.6f) { if (issues.length()) issues += "<br>"; issues += "EC LOW: +A+B (sep!)"; }
  else if ((float)r.ec < ec_lo * 0.85f) { if (issues.length()) issues += "<br>"; issues += "EC low: +A+B (sep!)"; }
  if (r.temp > 24.0f) { if (issues.length()) issues += "<br>"; issues += "HOT! root rot risk"; }
  else if (r.temp < 16.0f) { if (issues.length()) issues += "<br>"; issues += "COLD! heat to 18C"; }
  else if (r.temp > yc01_ranges.fmax[5] + 3.0f) { if (issues.length()) issues += "<br>"; issues += "temp high"; }
  else if (r.temp < yc01_ranges.fmin[5] - 3.0f) { if (issues.length()) issues += "<br>"; issues += "temp low"; }
  if (issues.length() == 0) {
    yc01_action_msg = "<span style='color:green'>All OK</span>";
    yc01_notes_msg = "";
  } else {
    yc01_action_msg = "<span style='color:orange;font-size:smaller'>" + issues + "</span>";
    if (issues.indexOf("A+B") >= 0) {
      yc01_notes_msg = "<small>NEVER mix A+B! Add A first, then B</small>";
    } else {
      yc01_notes_msg = "";
    }
  }
}

static int yc01_time_min_of_day(void) {
  return RtcTime.minute + RtcTime.hour * 60;
}

static void yc01_set_relay(int relay, bool on) {
  if (relay < 1) return;
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "POWER%d %s", relay, on ? "ON" : "OFF");
  ExecuteCommand(cmd, SRC_IGNORE);
}

static void yc01_calc_sun_times(void) {
  if (yc01_lat == 0.0f && yc01_lon == 0.0f) return;
  int day_of_year = RtcTime.day_of_year;
  float lat_rad = yc01_lat * 0.0174533f;
  float decl = 23.45f * sinf(0.0174533f * (360.0f/365.0f) * (day_of_year - 81));
  float decl_rad = decl * 0.0174533f;
  float cos_h = -tanf(lat_rad) * tanf(decl_rad);
  if (cos_h > 1.0f) cos_h = 1.0f;
  if (cos_h < -1.0f) cos_h = -1.0f;
  float h = acosf(cos_h) / 0.0174533f;
  float noon = 720.0f - yc01_lon * 4.0f;
  yc01_sunrise_min = (int)(noon - h * 4.0f);
  yc01_sunset_min = (int)(noon + h * 4.0f);
}

static String yc01_http_get(const char *url) {
  HTTPClientLight http;
  http.setURL(url);
  http.setTimeout(5000);
  int httpCode = http.GET();
  if (httpCode != 200) {
    http.end();
    return "";
  }
  String payload = http.getString();
  http.end();
  return payload;
}

static void yc01_geocode_city(const char *city) {
  char url[128];
  snprintf(url, sizeof(url), "http://geocoding-api.open-meteo.com/v1/search?name=%s&count=1", city);
  String json = yc01_http_get(url);
  if (json.length() == 0) return;
  int lat_idx = json.indexOf("\"latitude\":");
  int lon_idx = json.indexOf("\"longitude\":");
  if (lat_idx < 0 || lon_idx < 0) return;
  int lat_start = lat_idx + 11;
  int lat_end = json.indexOf(",", lat_start);
  int lon_start = lon_idx + 12;
  int lon_end = json.indexOf(",", lon_start);
  yc01_lat = json.substring(lat_start, lat_end).toFloat();
  yc01_lon = json.substring(lon_start, lon_end).toFloat();
  yc01_calc_sun_times();
}

static void yc01_fetch_weather(void) {
  if (yc01_lat == 0.0f && yc01_lon == 0.0f) return;
  char url[160];
  snprintf(url, sizeof(url), "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current_weather=true", yc01_lat, yc01_lon);
  String json = yc01_http_get(url);
  if (json.length() == 0) return;
  int t_idx = json.indexOf("\"temperature\":");
  if (t_idx < 0) return;
  int t_start = t_idx + 14;
  int t_end = json.indexOf(",", t_start);
  yc01_outside_temp = json.substring(t_start, t_end).toFloat();
}

static void yc01_autocontrol(void) {
  static int weather_tick = 0;
  static int pump_cycle_sec = 0;
  if (yc01_lat != 0.0f) {
    yc01_calc_sun_times();
    weather_tick++;
    if (weather_tick >= 1800) {
      weather_tick = 0;
      yc01_fetch_weather();
    }
  }
  int now = yc01_time_min_of_day();
  bool daylight = (now >= yc01_sunrise_min + 30 && now <= yc01_sunset_min - 30);
  xSemaphoreTake(yc01_mutex, portMAX_DELAY);
  yc01_reading_t r = yc01_last;
  xSemaphoreGive(yc01_mutex);
  if (yc01_pump_auto && yc01_pump_relay > 0) {
    bool emergency_stop = false;
    if (yc01_last.valid) {
      if (r.temp > 24.0f) emergency_stop = true;
      if (r.ph < 4.0f || r.ph > 9.0f) emergency_stop = true;
    }
    if (!emergency_stop) {
      pump_cycle_sec++;
      bool want_pump;
      if (daylight) {
        int cycle_pos = pump_cycle_sec % (15*60 + 45*60);
        want_pump = (cycle_pos < 15*60);
      } else {
        int cycle_pos = pump_cycle_sec % (15*60 + 150*60);
        want_pump = (cycle_pos < 15*60);
      }
      if (want_pump != yc01_pump_state) {
        yc01_pump_state = want_pump;
        yc01_set_relay(yc01_pump_relay, want_pump);
      }
    } else if (yc01_pump_state) {
      yc01_pump_state = false;
      yc01_set_relay(yc01_pump_relay, false);
    }
  }
  if (yc01_heater_auto && yc01_heater_relay > 0 && yc01_last.valid) {
    bool want_heater = false;
    if (r.temp < 18.0f) want_heater = true;
    if (r.temp >= 21.0f) want_heater = false;
    if (yc01_outside_temp != 99 && yc01_outside_temp < 10.0f && r.temp < 20.0f) want_heater = true;
    if (want_heater != yc01_heater_state) {
      yc01_heater_state = want_heater;
      yc01_set_relay(yc01_heater_relay, want_heater);
    }
  }
}

static void CmndYC01Pump(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"PumpRelay\":%d,\"PumpAuto\":%s,\"PumpState\":%s}}"),
               yc01_pump_relay, yc01_pump_auto ? "true" : "false", yc01_pump_state ? "ON" : "OFF");
    return;
  }
  int val = atoi(XdrvMailbox.data);
  yc01_pump_relay = val;
  yc01_pump_auto = true;
  Response_P(PSTR("{\"YC01\":{\"PumpRelay\":%d,\"PumpAuto\":true}}"), val);
}

static void CmndYC01Heater(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"HeaterRelay\":%d,\"HeaterAuto\":%s,\"HeaterState\":%s}}"),
               yc01_heater_relay, yc01_heater_auto ? "true" : "false", yc01_heater_state ? "ON" : "OFF");
    return;
  }
  int val = atoi(XdrvMailbox.data);
  yc01_heater_relay = val;
  yc01_heater_auto = true;
  Response_P(PSTR("{\"YC01\":{\"HeaterRelay\":%d,\"HeaterAuto\":true}}"), val);
}

static void CmndYC01LatLon(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Lat\":%.4f,\"Lon\":%.4f}}"), yc01_lat, yc01_lon);
    return;
  }
  char buf[48];
  strlcpy(buf, XdrvMailbox.data, sizeof(buf));
  char *sp = strchr(buf, ' ');
  if (!sp) { ResponseCmndFailed(); return; }
  *sp = 0;
  yc01_lat = atof(buf);
  yc01_lon = atof(sp + 1);
  yc01_calc_sun_times();
  Response_P(PSTR("{\"YC01\":{\"Lat\":%.4f,\"Lon\":%.4f,\"Sunrise\":%d,\"Sunset\":%d}}"),
             yc01_lat, yc01_lon, yc01_sunrise_min, yc01_sunset_min);
}

bool Xdrv89(uint32_t function) {
  bool result = false;
  switch (function) {case FUNC_INIT:         YC01Init(); break;
    case FUNC_EVERY_SECOND: yc01_autocontrol(); break;
    case FUNC_COMMAND:      result = DecodeCommand(kYC01_Commands,YC01_Commands); break;
    case FUNC_JSON_APPEND:  YC01JsonAppend(); break;
    case FUNC_WEB_SENSOR:   YC01WebSensor(); break;
    case FUNC_ACTIVE:       result = true; break;}
  return result;
}

#endif  // USE_YC01_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#endif  // ESP32
