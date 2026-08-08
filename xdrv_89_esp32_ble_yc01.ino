/*
  xdrv_89_esp32_ble_yc01.ino - BLE water quality meter driver for hydroponics

  Native C++ driver for BLE water quality meters (YC01/YIERYI/YINMIK 6-in-1).
  Uses Tasmota BLE engine. Requires tasmota32-bluetooth build.

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

  Console commands (prefix HPN):
    HPN               - show status
    HPNRead           - force immediate read
    HPNPoll N         - set poll interval 30-600s (default 240)
    HPNProfile [name] - set profile or list available
    HPNBloom 0/1      - toggle bloom mode
    HPNMac <addr> [t] - show/set MAC and address type
*/

#ifdef ESP32
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#ifdef USE_HPN_ESP32

#define XDRV_89 89

#include <NimBLEDevice.h>
#include <HttpClientLight.h>
#include <WiFiClient.h>

struct hpn_reading_t {float ph;
  int16_t ec;
  int16_t tds;
  int16_t orp;
  float cl;
  float temp;
  uint8_t batt;
  float salt;
  int16_t rssi;
  bool valid;};

struct hpn_ranges_t {float fmin[8];
  float fmax[8];
  float fmar[8];};

#define HPN_SVC_UUID       0xFF01
#define HPN_CHR_UUID       0xFF02
#define HPN_DEFAULT_POLL_S 240
#define HPN_MIN_POLL_S     30
#define HPN_MAX_POLL_S     600
#define HPN_OP_TIMEOUT_S   15
#define HPN_DEFAULT_MAC    "414284588113"
#define HPN_MAX_MAC_LEN    32

#ifndef HPN_DEFAULT_MAC
#define HPN_DEFAULT_MAC "414284588113"
#endif

static const char* const HPN_PF_NAMES[] = {"Generic","CherryTomato","BeefsteakTomato","BellPepper_Poblano_BananaPepper","ChiliPepper_Jalapeno_Cayenne","Habanero","Eggplant","Okra","BushBeans","PoleBeans","Peas_SnowPeas_SugarSnapPeas","Edamame","Cantaloupe_HoneydewMelon_MiniWatermelon","Strawberry_EverbearingStrawberry_AlpineStrawberry","Lettuce_ButterheadLettuce_RomaineLettuce","Spinach_NewZealandSpinach","Kale_CollardGreens","Arugula","Endive_Escarole_Frisee_Radicchio","Mache","Microgreens","BokChoy_Senposai_YukinaSavoy","Tatsoi_Komatsuna_Mibuna","MustardGreens","Mizuna","Watercress","Amaranth","MalabarSpinach","Purslane","Sorrel","Celtuce","SweetPotatoVine","GarlicChives","Basil_ThaiBasil_HolyBasil","Parsley","Cilantro","Dill","Mint_Peppermint_Spearmint","Chives","Oregano","Thyme_Sage_Tarragon_Marjoram","Rosemary_BayLaurel","LemonBalm","Lemongrass","Stevia","Shiso","VietnameseCoriander","Culantro","Epazote","Lovage","SummerSavory","WinterSavory","Lavender","Chamomile","Feverfew_Hyssop_Echinacea","Fennel","Nasturtium_Calendula_Borage_Pansy_Viola","Petunia","GerberaDaisy","Zinnia","Snapdragon","Begonia_Impatiens","SweetAlyssum","Lobelia","Marigold","Dianthus","Cornflower","Portulaca","Ginger_Turmeric","Claytonia","LandCress","WelshOnion","Daylily","FavaBeans","BearsGarlic","BasilMintParsley","BeetGreensTurnipGreensCollardGreens","Beetroot","BellPepperChiliPepperJalapeno","BokChoyTatsoiKomatsuna","Broccoli_Cauliflower_Cabbage_BrusselsSprouts_Kohlrabi","BushBeansPoleBeansEdamame","Carrot_SmallVariety","Celery","CherryTomatoBellPepper","CilantroDillChives","Cucumber","EggplantZucchiniPepper","GreenOnionGarlicChivesLeek","GreenOnion_OnionGreens_ShallotGreens","LavenderRosemarySage","Leek","LemonBalmChamomileMint","LettuceArugulaChard","LettuceBasilParsley","LettuceSpinachArugulaRadish","LettuceSpinachKale","MediterraneanHerb","Microgreens","MizunaMibunaSenposai","OnionShallotChive","PepperCucumber","PetuniaLobeliaAlyssum","Potato_SmallAeroponic","RadicchioMache","Radish","RadishTurnipCarrot","RosemaryThymeOregano","SaladGreens","ShisoVietnameseCorianderCulantro","SpinachKaleChard","SwissChard_NapaCabbage_TurnipGreens_BeetGreens","TomatoBasilCompanion","TomatoCherryTomatoRoma","TomatoPepperEggplant","Tomato_RomaTomato_HeirloomTomato","Turnip","WatercressPurslaneSorrel","Zucchini_SummerSquash_PattypanSquash"};

static const float HPN_PF_DATA[][17] = {
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

static const size_t HPN_PF_COUNT = sizeof(HPN_PF_NAMES) / sizeof(HPN_PF_NAMES[0]);

static hpn_reading_t hpn_last = {0};
static hpn_ranges_t hpn_ranges = {};

static char hpn_mac[HPN_MAX_MAC_LEN] = HPN_DEFAULT_MAC;
static uint8_t hpn_addr_type = 0;
static uint16_t hpn_poll_s = HPN_DEFAULT_POLL_S;
static bool hpn_bloom = false;
static bool hpn_connected = false;
static int16_t hpn_rssi = 0;
static int16_t hpn_orp_offset = 0;

static String hpn_profile_name = "Generic";
static SemaphoreHandle_t hpn_mutex = nullptr;
static uint8_t hpn_mac_bytes[6] = {0};
static String hpn_location = "";
static String hpn_action_msg = "";
static String hpn_notes_msg = "";
static float hpn_volume_l = 20.0;
static int hpn_water_type = 1;
static float hpn_kh = 4.0f;
static float hpn_ph_cal = 1.0f;
static float hpn_ec_cal = 1.0f;
static int hpn_pump_relay = 0;
static int hpn_heater_relay = 0;
static bool hpn_pump_auto = false;
static bool hpn_heater_auto = false;
static bool hpn_pump_state = false;
static bool hpn_heater_state = false;
static float hpn_lat = 0.0f;
static float hpn_lon = 0.0f;
static int hpn_sunrise_min = 360;
static int hpn_sunset_min = 1080;
static int hpn_outside_temp = 99;

static int16_t hpn_s16be(const uint8_t *d, size_t i) {uint16_t v = ((uint16_t)d[i] << 8) | d[i + 1];
  if (v > 32767) v -= 65536;
  return (int16_t)v;}

static void hpn_decode(uint8_t *d, size_t len) {
  for (int i = (int)len - 1; i > 0; i--) {uint8_t t1 = d[i];
    uint8_t h1 = (t1 & 0x55) << 1;
    uint8_t l1 = (t1 & 0xAA) >> 1;
    uint8_t t0 = d[i - 1];
    uint8_t h0 = (t0 & 0x55) << 1;
    uint8_t l0 = (t0 & 0xAA) >> 1;
    d[i]     = 0xFF - (h1 | l0);
    d[i - 1] = 0xFF - (h0 | l1);}
}

static void hpn_parse_frame(const uint8_t *d, size_t len, hpn_reading_t *r);
static void hpn_parse_frame(const uint8_t *d, size_t len, hpn_reading_t *r) {if (len < 18) return;
  if (d[0] != 1 || d[2] != 15) return;
  r->ph   = hpn_s16be(d,3) / 100.0f;
  r->ec   = hpn_s16be(d,5);
  r->tds  = hpn_s16be(d,7);
  r->orp  = hpn_s16be(d,9);
  int16_t cl_raw = hpn_s16be(d,11);
  r->cl   = (cl_raw < 0 ? 0.0f : cl_raw / 10.0f);
  r->temp = hpn_s16be(d,13) / 10.0f;
  int32_t batt_raw = hpn_s16be(d,15);
  int32_t batt = (100 * (batt_raw - 1950)) / (3190 - 1950);
  if (batt < 0) batt = 0;
  if (batt > 100) batt = 100;
  r->batt = (uint8_t)batt;
  r->salt = r->ec * 0.55f;
  r->valid = true;}

static void hpn_parse_mac(const char *src, uint8_t *dst, uint8_t *type);
static void hpn_parse_mac(const char *src, uint8_t *dst, uint8_t *type) {
  char buf[HPN_MAX_MAC_LEN];
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

static void hpn_mac_to_str(const uint8_t *mac, char *out, size_t out_len) {snprintf(out,out_len,"%02X%02X%02X%02X%02X%02X",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);}

static void hpn_apply_profile(const float *v);
static void hpn_apply_profile(const float *v) {
  hpn_ranges.fmin[0] = v[0];
  hpn_ranges.fmax[0] = v[1];
  hpn_ranges.fmin[1] = v[2];
  hpn_ranges.fmax[1] = v[3];
  hpn_ranges.fmin[2] = v[2] * 0.64f;
  hpn_ranges.fmax[2] = v[3] * 0.64f;
  hpn_ranges.fmin[3] = v[7];
  hpn_ranges.fmax[3] = v[8];
  hpn_ranges.fmin[4] = 0.0f;
  hpn_ranges.fmax[4] = v[6];
  hpn_ranges.fmin[5] = v[4];
  hpn_ranges.fmax[5] = v[5];
  hpn_ranges.fmin[6] = 0.0f;
  hpn_ranges.fmax[6] = v[9] / 10.0f;
  hpn_ranges.fmin[7] = 60.0f;
  hpn_ranges.fmax[7] = 100.0f;
  for (int i = 0; i < 8; i++) {hpn_ranges.fmar[i] = (hpn_ranges.fmax[i] - hpn_ranges.fmin[i]) * 0.15f;
    if (i == 0 && hpn_ranges.fmar[i] < 0.1f) hpn_ranges.fmar[i] = 0.1f;
    if (i == 1 && hpn_ranges.fmar[i] < 200.0f) hpn_ranges.fmar[i] = 200.0f;
    if (i == 2 && hpn_ranges.fmar[i] < 128.0f) hpn_ranges.fmar[i] = 128.0f;
    if (i == 3 && hpn_ranges.fmar[i] < 50.0f) hpn_ranges.fmar[i] = 50.0f;
    if (i == 5 && hpn_ranges.fmar[i] < 2.0f) hpn_ranges.fmar[i] = 2.0f;
    if (i == 6 && hpn_ranges.fmar[i] < 125.0f) hpn_ranges.fmar[i] = 125.0f;}
}

static void hpn_apply_boost(const float *v);
static void hpn_apply_boost(const float *v) {if (v[10] <= 0.0f) return;
  hpn_ranges.fmin[0] = v[10];
  hpn_ranges.fmax[0] = v[11];
  hpn_ranges.fmin[1] = v[12];
  hpn_ranges.fmax[1] = v[13];
  hpn_ranges.fmin[2] = v[12] * 0.64f;
  hpn_ranges.fmax[2] = v[13] * 0.64f;
  hpn_ranges.fmin[5] = v[14];
  hpn_ranges.fmax[5] = v[15];}

static const char* hpn_color(float v, int idx) {if (idx < 0 || idx > 7) return "green";
  float mn = hpn_ranges.fmin[idx];
  float mx = hpn_ranges.fmax[idx];
  float mar = hpn_ranges.fmar[idx];
  if (v < mn - mar || v > mx + mar) return "red";
  if (v < mn || v > mx) return "orange";
  return "green";}

static int hpn_find_profile(const char *name) {
  for (size_t i = 0; i < HPN_PF_COUNT; i++) {if (strcmp(HPN_PF_NAMES[i],name) == 0) return (int)i;}
  return -1;
}

static void hpn_load_profile(const char *name);
static void hpn_load_profile(const char *name) {
  int idx = hpn_find_profile(name);
  if (idx < 0) {AddLog(LOG_LEVEL_INFO,PSTR("HPN: profile '%s' not found,using Generic"),name);
    idx = 0;}
  hpn_profile_name = HPN_PF_NAMES[idx];
  hpn_apply_profile(HPN_PF_DATA[idx]);
  if (hpn_bloom && HPN_PF_DATA[idx][10] > 0) {hpn_apply_boost(HPN_PF_DATA[idx]);} else if (hpn_bloom) {hpn_bloom = false;}
}

static int hpn_read_ok_cnt = 0;
static SemaphoreHandle_t hpn_sem = nullptr;
static volatile bool hpn_op_trigger = false;

static int hpn_ble_op_complete(BLE_ESP32::generic_sensor_t *op) {
  if (op->context != (void*)0x01) return 0;
  if (op->readlen >= 18) {
    uint8_t buf[32];
    size_t len = op->readlen > 32 ? 32 : op->readlen;
    memcpy(buf, op->dataRead, len);
    hpn_decode(buf, len);
    hpn_reading_t r;
    hpn_parse_frame(buf, len, &r);
    if (r.valid) {
      r.rssi = 0;
      r.orp -= hpn_orp_offset;
      xSemaphoreTake(hpn_mutex, portMAX_DELAY);
      hpn_last = r;
      hpn_connected = true;
      xSemaphoreGive(hpn_mutex);
      hpn_read_ok_cnt++;
      if (hpn_read_ok_cnt <= 3 || (hpn_read_ok_cnt % 10 == 0)) {
        AddLog(LOG_LEVEL_INFO, PSTR("HPN: read OK #%d pH=%.2f EC=%d TDS=%d Temp=%.1f Batt=%d"), hpn_read_ok_cnt, r.ph, r.ec, r.tds, r.temp, r.batt);
      }
      xSemaphoreGive(hpn_sem);
      return 1;
    }
  } else if (op->state < 0) {
    xSemaphoreTake(hpn_mutex, portMAX_DELAY);
    hpn_connected = false;
    xSemaphoreGive(hpn_mutex);
    AddLog(LOG_LEVEL_DEBUG, PSTR("HPN: BLE op failed state=%d"), op->state);
  }
  xSemaphoreGive(hpn_sem);
  return 1;
}

static void hpn_queue_read(void) {
  BLE_ESP32::generic_sensor_t *op = nullptr;
  int res = BLE_ESP32::newOperation(&op);
  if (!res || !op) return;
  uint8_t mac[6];
  hpn_parse_mac(hpn_mac, mac, nullptr);
  op->addr = NimBLEAddress(mac, hpn_addr_type);
  op->serviceUUID = NimBLEUUID((uint16_t)HPN_SVC_UUID);
  op->characteristicUUID = NimBLEUUID((uint16_t)HPN_CHR_UUID);
  op->readlen = 1;
  op->completecallback = (void*)&hpn_ble_op_complete;
  op->context = (void*)0x01;
  hpn_op_trigger = true;
  BLE_ESP32::extQueueOperation(&op);
}

static void hpn_ble_task(void *pv);
static void hpn_ble_task(void *pv) {
  while (true) {
    if (NimBLEDevice::isInitialized()) {
      hpn_queue_read();
      xSemaphoreTake(hpn_sem, pdMS_TO_TICKS(30000));
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

static void CmndHPN(void);
static void CmndHPNRead(void);
static void CmndHPNPoll(void);
static void CmndHPNProfile(void);
static void CmndHPNBloom(void);
static void CmndHPNMac(void);
static void HPNInit(void);
static void HPNEverySecond(void);
static void HPNJsonAppend(void);
static void HPNWebSensor(void);

static void CmndHPN(void) {
  char macstr[13];
  uint8_t mac[6];
  hpn_parse_mac(hpn_mac, mac, nullptr);
  hpn_mac_to_str(mac, macstr, sizeof(macstr));
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d,\"Poll\":%d,\"Profile\":\"%s\",\"Bloom\":\"%s\",\"Volume\":%.1f,\"KH\":%.1f,\"Location\":\"%s\"}}"),
             macstr, hpn_addr_type, hpn_poll_s, hpn_profile_name.c_str(),
             hpn_bloom ? "on" : "off", hpn_volume_l, hpn_kh, hpn_location.c_str());
  AddLog(LOG_LEVEL_INFO, PSTR("HPN: Commands:"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPN                 - this help + status"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNRead             - force immediate sensor read"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNPoll N           - set poll interval 30-600s (default 240)"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNProfile [name]   - set profile or list all"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNBloom 0/1        - toggle bloom mode for current profile"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNMac <addr> [type] - show/set MAC and address type"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNLocation <city>  - set city (geocodes lat/lon for sun times)"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNLatLon <lat> <lon> - set coordinates directly"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HYDROPump <relay>   - set pump relay (1-4) + auto mode"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNHeater <relay>   - set heater relay (1-4) + auto mode"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNVolume <liters>  - set reservoir volume (default 20)"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNWater <type>     - Distilled|SoftTap|MediumTap|HardTap"));
  AddLog(LOG_LEVEL_INFO, PSTR("  HPNCal <param> <val> - KH|PH|EC calibration (e.g. KH 4.0)"));
}

static void CmndHPNRead(void) {
  hpn_op_trigger = true;
  ResponseCmndDone();}

static void CmndHPNPoll(void) {int val = XdrvMailbox.payload;
  if (val < HPN_MIN_POLL_S) val = HPN_MIN_POLL_S;
  if (val > HPN_MAX_POLL_S) val = HPN_MAX_POLL_S;
  hpn_poll_s = (uint16_t)val;
  ResponseCmndNumber(hpn_poll_s);}

static void CmndHPNProfile(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01Profile\":["));
    int line_len = 0;
    for (size_t i = 0; i < HPN_PF_COUNT; i++) {
      if (i > 0) {ResponseAppend_P(PSTR(","));
        line_len += 1;}
      int name_len = strlen(HPN_PF_NAMES[i]);
      if (line_len > 0 && line_len + name_len > 100) {ResponseAppend_P(PSTR("\n"));
        line_len = 0;}
      ResponseAppend_P(PSTR("\"%s\""), HPN_PF_NAMES[i]);
      line_len += name_len + 2;
    }
    ResponseAppend_P(PSTR("\n]}"));
    return;
  }
  String warn = "";
  if (hpn_location.length() > 0) {
    int idx = hpn_find_profile(XdrvMailbox.data);
    if (idx > 0) {
      float tmin = HPN_PF_DATA[idx][4];
      float tmax = HPN_PF_DATA[idx][5];
      warn = " Note: check season for this profile (temp " + String(tmin, 0) + "-" + String(tmax, 0) + " C) in " + hpn_location.c_str() + ".";
    }
  }
  hpn_load_profile(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Profile\":\"%s\"%s}}"), hpn_profile_name.c_str(), warn.c_str());
}

static void CmndHPNBloom(void) {
  bool want_bloom = XdrvMailbox.payload != 0;
  if (want_bloom) {
    int idx = hpn_find_profile(hpn_profile_name.c_str());
    if (idx >= 0 && HPN_PF_DATA[idx][10] <= 0.0f) {
      Response_P(PSTR("{\"YC01\":{\"Bloom\":\"off\",\"Error\":\"bloom not available for profile '%s\"}}"),
                 hpn_profile_name.c_str());
      return;
    }
  }
  hpn_bloom = want_bloom;
  hpn_load_profile(hpn_profile_name.c_str());
  Response_P(PSTR("{\"YC01\":{\"Bloom\":\"%s\"}}"), hpn_bloom ? "on" : "off");
}

static void CmndHPNMac(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    char macstr[13];
    uint8_t mac[6];
    hpn_parse_mac(hpn_mac, mac, nullptr);
    hpn_mac_to_str(mac, macstr, sizeof(macstr));
    Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), macstr, hpn_addr_type);
    return;
  }
  uint8_t newmac[6];
  uint8_t newtype = 0;
  hpn_parse_mac(XdrvMailbox.data, newmac, &newtype);
  hpn_mac_to_str(newmac, hpn_mac, sizeof(hpn_mac));
  hpn_addr_type = newtype;
  Response_P(PSTR("{\"YC01\":{\"MAC\":\"%s\",\"Type\":%d}}"), hpn_mac, hpn_addr_type);
}

static void CmndHPNLocation(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Location\":\"%s\"}}"), hpn_location.c_str());
    return;
  }
  hpn_location = XdrvMailbox.data;
  hpn_geocode_city(XdrvMailbox.data);
  Response_P(PSTR("{\"YC01\":{\"Location\":\"%s\",\"Lat\":%.4f,\"Lon\":%.4f,\"Sunrise\":%d,\"Sunset\":%d}}"),
             hpn_location.c_str(), hpn_lat, hpn_lon, hpn_sunrise_min, hpn_sunset_min);
}

const char kHPN_Commands[] PROGMEM = "HPN|"
  "|"
  "Read|"
  "Poll|"
  "Profile|"
  "Bloom|"
  "Mac|"
  "Location|"
  "LatLon|"
  "Pump|"
  "Heater|"
  "Volume|"
  "Water|"
  "Cal";

void (*const HPN_Commands[])(void) PROGMEM = {&CmndHPN,&CmndHPNRead,&CmndHPNPoll,&CmndHPNProfile,&CmndHPNBloom,&CmndHPNMac,&CmndHPNLocation,&CmndHPNLatLon,&CmndHPNPump,&CmndHPNHeater,&CmndHPNVolume,&CmndHPNWater,&CmndHPNCal};

static void HPNInit(void) {
  hpn_mutex = xSemaphoreCreateMutex();
  hpn_sem = xSemaphoreCreateBinary();
  if (hpn_find_profile(hpn_profile_name.c_str()) < 0) {hpn_profile_name = "Generic";}
  hpn_load_profile(hpn_profile_name.c_str());
  hpn_parse_mac(hpn_mac, hpn_mac_bytes, nullptr);
  AddLog(LOG_LEVEL_INFO, PSTR("HPN: driver v1.0 started, MAC %s, profile %s"), hpn_mac, hpn_profile_name.c_str());
  xTaskCreatePinnedToCore(hpn_ble_task, "YC01BLE", 8192, nullptr, 1, nullptr,
#ifdef CONFIG_FREERTOS_UNICORE
    0
#else
    1
#endif
  );
  AddLog(LOG_LEVEL_INFO, PSTR("HPN: driver v1.0 started, MAC %s, profile %s"),
         hpn_mac, hpn_profile_name.c_str());
}

static void HPNEverySecond(void) {
}

static void HPNJsonAppend(void) {
  if (!hpn_last.valid) return;
  char macstr[13];
  uint8_t mac[6];
  hpn_parse_mac(hpn_mac, mac, nullptr);
  hpn_mac_to_str(mac, macstr, sizeof(macstr));
  xSemaphoreTake(hpn_mutex, portMAX_DELAY);
  hpn_reading_t r = hpn_last;
  xSemaphoreGive(hpn_mutex);
  ResponseAppend_P(PSTR(",\"YC01\":{\"MAC\":\"%s\",\"pH\":%*_f,\"EC\":%d,\"TDS\":%d,\"ORP\":%d,\"SALT\":%*_f,\"Temp\":%*_f,\"Cl\":%*_f,\"Batt\":\"%s\"}"),
                   macstr, 2, &r.ph, r.ec, r.tds, r.orp, 1, &r.salt, 1, &r.temp, 2, &r.cl, r.batt >= 84 ? "High" : "Low");
}

static void hpn_format_profile(const char *name, char *out, size_t out_len) {
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

static void HPNWebSensor(void) {
  if (!hpn_last.valid) {
    WSContentSend_PD(PSTR("{s}BLE{m}<span style='color:red'>Disconnected</span>{e}"));
    return;
  }
  xSemaphoreTake(hpn_mutex, portMAX_DELAY);
  hpn_reading_t r = hpn_last;
  xSemaphoreGive(hpn_mutex);
  hpn_calc_action();
  WSContentSend_PD(PSTR("{s}BLE{m}<span style='color:green'>Connected</span>{e}"));
  char pf_fmt[48];
  hpn_format_profile(hpn_profile_name.c_str(), pf_fmt, sizeof(pf_fmt));
  WSContentSend_PD(PSTR("{s}Profile{m}%s{e}"), pf_fmt);
  WSContentSend_PD(PSTR("{s}Bloom{m}%s{e}"), hpn_bloom ? "Yes" : "No");
  char buf16[16];
  dtostrfd(hpn_volume_l, 0, buf16);
  WSContentSend_PD(PSTR("{s}Volume{m}%sL{e}"), buf16);
  dtostrfd(hpn_kh, 1, buf16);
  WSContentSend_PD(PSTR("{s}KH{m}%s dKH{e}"), buf16);
  WSContentSend_PD(PSTR("{s}Location{m}%s{e}"), hpn_location.length() > 0 ? hpn_location.c_str() : "Not set");
  WSContentSend_PD(PSTR("{s}pH{m}<span style='color:%s'>%*_f</span>{e}"),
                   hpn_color(r.ph, 0), 2, &r.ph);
  WSContentSend_PD(PSTR("{s}EC{m}<span style='color:%s'>%d \u00B5S/cm</span>{e}"),
                   hpn_color((float)r.ec, 1), r.ec);
  WSContentSend_PD(PSTR("{s}TDS{m}<span style='color:%s'>%d ppm</span>{e}"),
                   hpn_color((float)r.tds, 2), r.tds);
  WSContentSend_PD(PSTR("{s}ORP{m}<span style='color:%s'>%d mV</span>{e}"),
                   hpn_color((float)r.orp, 3), r.orp);
  WSContentSend_PD(PSTR("{s}SALT{m}<span style='color:%s'>%*_f ppm</span>{e}"),
                   hpn_color(r.salt, 6), 1, &r.salt);
  char tbuf[96];
  char val[16];
  dtostrfd(r.temp, 1, val);
  const char* tcolor = "green";
  const char* label = "Normal";
  if (r.temp > hpn_ranges.fmax[5] + 5.0f) { tcolor = "red"; label = "Rotting"; }
  else if (r.temp > hpn_ranges.fmax[5] + 2.0f) { tcolor = "orange"; label = "Warm"; }
  else if (r.temp < hpn_ranges.fmin[5] - 5.0f) { tcolor = "blue"; label = "Lockout"; }
  else if (r.temp < hpn_ranges.fmin[5] - 2.0f) { tcolor = "blue"; label = "Cold"; }
  snprintf(tbuf, sizeof(tbuf), "%s (%s \u00B0C)", label, val);
  WSContentSend_PD(PSTR("{s}Heat{m}<span style='color:%s'>%s</span>{e}"),
                   tcolor, tbuf);
  WSContentSend_PD(PSTR("{s}Chlorine{m}<span style='color:%s'>%*_f mg/L</span>{e}"),
                   hpn_color(r.cl, 4), 2, &r.cl);
  const char* batt_color = r.batt >= 84 ? "green" : "red";
  const char* batt_txt = r.batt >= 84 ? "Charged" : "Discharging";
  WSContentSend_PD(PSTR("{s}Battery{m}<span style='color:%s'>%s</span>{e}"),
                   batt_color, batt_txt);
  WSContentSend_PD(PSTR("{s}Add{m}%s{e}"), hpn_action_msg.c_str());
  if (hpn_notes_msg.length() > 0) {
    WSContentSend_PD(PSTR("{s}Notes{m}%s{e}"), hpn_notes_msg.c_str());
  }
  if (hpn_pump_relay > 0) {
    char pump_txt[32];
    snprintf(pump_txt, sizeof(pump_txt), "Pump (relay %d): %s%s", hpn_pump_relay, hpn_pump_state ? "ON" : "OFF", hpn_pump_auto ? " [auto]" : "");
    WSContentSend_PD(PSTR("{s}%s{m}%s{e}"), hpn_pump_auto ? "Pump auto" : "Pump", pump_txt);
  }
  if (hpn_heater_relay > 0) {
    char heat_txt[32];
    snprintf(heat_txt, sizeof(heat_txt), "Heater (relay %d): %s%s", hpn_heater_relay, hpn_heater_state ? "ON" : "OFF", hpn_heater_auto ? " [auto]" : "");
    WSContentSend_PD(PSTR("{s}%s{m}%s{e}"), hpn_heater_auto ? "Heater auto" : "Heater", heat_txt);
  }
}

static void hpn_fmt_dose(float ml, char *out, size_t out_len) {
  if (ml >= 1000.0f) {
    float liters = ml / 1000.0f;
    if (liters == (int)liters) dtostrfd(liters, 0, out);
    else dtostrfd(liters, 2, out);
    strcat(out, "L");
  } else {
    if (ml == (int)ml) dtostrfd(ml, 0, out);
    else dtostrfd(ml, 1, out);
    strcat(out, "ml");
  }
}

static void hpn_calc_action(void) {
  if (!hpn_last.valid) {
    hpn_action_msg = "<span style='color:gray'>No data</span>";
    return;
  }
  hpn_reading_t r = hpn_last;
  float ph_lo = hpn_ranges.fmin[0];
  float ph_hi = hpn_ranges.fmax[0];
  float ec_lo = hpn_ranges.fmin[1];
  float ec_hi = hpn_ranges.fmax[1];
  float ph_diff_hi = r.ph - ph_hi;
  float ph_diff_lo = ph_lo - r.ph;
  String issues = "";
  float vol = hpn_volume_l > 0 ? hpn_volume_l : 20.0f;
  char vol_str[16];
  dtostrfd(vol, 0, vol_str);
  if (ph_diff_hi > 0.3f) { float ml = ph_diff_hi * hpn_volume_l * hpn_kh * 0.15f * hpn_ph_cal; char buf[16]; hpn_fmt_dose(ml, buf, sizeof(buf)); issues += buf; issues += " vinegar"; }
  else if (ph_diff_lo > 0.3f) { float ml = ph_diff_lo * hpn_volume_l * hpn_kh * 0.25f * hpn_ph_cal; char buf[16]; hpn_fmt_dose(ml, buf, sizeof(buf)); issues += buf; issues += " NaHCO3"; }
  float ec_diff = 0;
  if ((float)r.ec > ec_hi * 1.2f) {
    float water_L = hpn_volume_l * ((float)r.ec / ec_hi - 1.0f);
    char buf[16]; dtostrfd(water_L, 1, buf);
    if (issues.length()) issues += "<br>";
    issues += buf; issues += "L water";
  } else if ((float)r.ec < ec_lo * 0.85f) {
    ec_diff = ec_lo - (float)r.ec;
    float ml_A = ec_diff * hpn_volume_l * 0.5f * hpn_ec_cal;
    float ml_B = ec_diff * hpn_volume_l * 0.5f * hpn_ec_cal;
    char bufA[16], bufB[16];
    hpn_fmt_dose(ml_A, bufA, sizeof(bufA));
    hpn_fmt_dose(ml_B, bufB, sizeof(bufB));
    if (issues.length()) issues += "<br>";
    issues += bufA; issues += " A, "; issues += bufB; issues += " B";
  }
  if (issues.length() == 0) {
    hpn_action_msg = "<span style='color:green'>No action needed</span>";
    hpn_notes_msg = "";
  } else {
    hpn_action_msg = "<span style='color:orange'>" + issues + "</span>";
    if (issues.indexOf("A+B") >= 0) {
      hpn_notes_msg = "A \u2192 0.5h \u2192 B";
    } else {
      hpn_notes_msg = "";
    }
  }
}

static int hpn_time_min_of_day(void) {
  return RtcTime.minute + RtcTime.hour * 60;
}

static void hpn_set_relay(int relay, bool on) {
  if (relay < 1) return;
  char cmd[16];
  snprintf(cmd, sizeof(cmd), "POWER%d %s", relay, on ? "ON" : "OFF");
  ExecuteCommand(cmd, SRC_IGNORE);
}

static void hpn_calc_sun_times(void) {
  if (hpn_lat == 0.0f && hpn_lon == 0.0f) return;
  int day_of_year = RtcTime.day_of_year;
  float lat_rad = hpn_lat * 0.0174533f;
  float decl = 23.45f * sinf(0.0174533f * (360.0f/365.0f) * (day_of_year - 81));
  float decl_rad = decl * 0.0174533f;
  float cos_h = -tanf(lat_rad) * tanf(decl_rad);
  if (cos_h > 1.0f) cos_h = 1.0f;
  if (cos_h < -1.0f) cos_h = -1.0f;
  float h = acosf(cos_h) / 0.0174533f;
  float noon = 720.0f - hpn_lon * 4.0f;
  hpn_sunrise_min = (int)(noon - h * 4.0f);
  hpn_sunset_min = (int)(noon + h * 4.0f);
}

static String hpn_http_get(const char *url) {
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

static void hpn_reverse_geocode(float lat, float lon) {
  char url[160];
  snprintf(url, sizeof(url), "https://api.bigdatacloud.net/data/reverse-geocode-client?latitude=%.4f&longitude=%.4f&localityLanguage=en", lat, lon);
  String json = hpn_http_get(url);
  if (json.length() == 0) return;
  int city_idx = json.indexOf("\"city\":");
  if (city_idx < 0) city_idx = json.indexOf("\"locality\":");
  if (city_idx < 0) return;
  int start = json.indexOf("\"", city_idx + 8) + 1;
  int end = json.indexOf("\"", start);
  if (end <= start) return;
  hpn_location = json.substring(start, end);
  AddLog(LOG_LEVEL_INFO, PSTR("HPN: reverse geocoded to '%s'"), hpn_location.c_str());
}

static void hpn_geocode_city(const char *city) {
  char url[128];
  snprintf(url, sizeof(url), "http://geocoding-api.open-meteo.com/v1/search?name=%s&count=1&language=en", city);
  String json = hpn_http_get(url);
  if (json.length() == 0) { AddLog(LOG_LEVEL_DEBUG, PSTR("HPN: geocode failed")); return; }
  if (json.indexOf("\"results\":") < 0) { AddLog(LOG_LEVEL_DEBUG, PSTR("HPN: city not found")); return; }
  int lat_idx = json.indexOf("\"latitude\":");
  int lon_idx = json.indexOf("\"longitude\":");
  if (lat_idx < 0 || lon_idx < 0) return;
  float lat = json.substring(lat_idx + 11, json.indexOf(",", lat_idx + 11)).toFloat();
  float lon = json.substring(lon_idx + 12, json.indexOf(",", lon_idx + 12)).toFloat();
  if (lat == 0.0f && lon == 0.0f) { AddLog(LOG_LEVEL_DEBUG, PSTR("HPN: invalid coords")); return; }
  hpn_lat = lat;
  hpn_lon = lon;
  hpn_calc_sun_times();
  AddLog(LOG_LEVEL_INFO, PSTR("HPN: location set to %.4f, %.4f (sunrise %d:%02d, sunset %d:%02d)"),
         hpn_lat, hpn_lon, hpn_sunrise_min/60, hpn_sunrise_min%60, hpn_sunset_min/60, hpn_sunset_min%60);
}

static void hpn_fetch_weather(void) {
  if (hpn_lat == 0.0f || hpn_lon == 0.0f) return;
  char url[160];
  snprintf(url, sizeof(url), "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f&current_weather=true", hpn_lat, hpn_lon);
  String json = hpn_http_get(url);
  if (json.length() == 0) return;
  int t_idx = json.indexOf("\"temperature\":");
  if (t_idx < 0) return;
  hpn_outside_temp = json.substring(t_idx + 14, json.indexOf(",", t_idx + 14)).toFloat();
  AddLog(LOG_LEVEL_DEBUG, PSTR("HPN: outside temp %.1fC"), hpn_outside_temp);
}

static void hpn_autocontrol(void) {
  static int weather_tick = 0;
  static int pump_cycle_sec = 0;
  if (hpn_lat != 0.0f) {
    hpn_calc_sun_times();
    weather_tick++;
    if (weather_tick >= 1800) {
      weather_tick = 0;
      hpn_fetch_weather();
    }
  }
  int now = hpn_time_min_of_day();
  bool daylight = (now >= hpn_sunrise_min + 30 && now <= hpn_sunset_min - 30);
  xSemaphoreTake(hpn_mutex, portMAX_DELAY);
  hpn_reading_t r = hpn_last;
  xSemaphoreGive(hpn_mutex);
  if (hpn_pump_auto && hpn_pump_relay > 0) {
    bool emergency_stop = false;
    if (hpn_last.valid) {
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
      if (want_pump != hpn_pump_state) {
        hpn_pump_state = want_pump;
        hpn_set_relay(hpn_pump_relay, want_pump);
      }
    } else if (hpn_pump_state) {
      hpn_pump_state = false;
      hpn_set_relay(hpn_pump_relay, false);
    }
  }
  if (hpn_heater_auto && hpn_heater_relay > 0 && hpn_last.valid) {
    bool want_heater = false;
    if (r.temp < 18.0f) want_heater = true;
    if (r.temp >= 21.0f) want_heater = false;
    if (hpn_outside_temp != 99 && hpn_outside_temp < 10.0f && r.temp < 20.0f) want_heater = true;
    if (want_heater != hpn_heater_state) {
      hpn_heater_state = want_heater;
      hpn_set_relay(hpn_heater_relay, want_heater);
    }
  }
}

static void CmndHPNPump(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"PumpRelay\":%d,\"PumpAuto\":%s,\"PumpState\":%s}}"),
               hpn_pump_relay, hpn_pump_auto ? "true" : "false", hpn_pump_state ? "ON" : "OFF");
    return;
  }
  int val = atoi(XdrvMailbox.data);
  hpn_pump_relay = val;
  hpn_pump_auto = true;
  Response_P(PSTR("{\"YC01\":{\"PumpRelay\":%d,\"PumpAuto\":true}}"), val);
}

static void CmndHPNHeater(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"HeaterRelay\":%d,\"HeaterAuto\":%s,\"HeaterState\":%s}}"),
               hpn_heater_relay, hpn_heater_auto ? "true" : "false", hpn_heater_state ? "ON" : "OFF");
    return;
  }
  int val = atoi(XdrvMailbox.data);
  hpn_heater_relay = val;
  hpn_heater_auto = true;
  Response_P(PSTR("{\"YC01\":{\"HeaterRelay\":%d,\"HeaterAuto\":true}}"), val);
}

static void CmndHPNCal(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"KH\":%.1f,\"PHCal\":%.2f,\"ECCal\":%.2f}}"), hpn_kh, hpn_ph_cal, hpn_ec_cal);
    return;
  }
  char buf[48];
  strlcpy(buf, XdrvMailbox.data, sizeof(buf));
  char *sp = strchr(buf, ' ');
  if (!sp) { ResponseCmndFailed(); return; }
  *sp = 0;
  String param = sp + 1;
  if (buf[0] == 'k' || buf[0] == 'K') { hpn_kh = param.toFloat(); if (hpn_kh < 0) hpn_kh = 0; if (hpn_kh > 15) hpn_kh = 15; Response_P(PSTR("{\"YC01\":{\"KH\":%.1f}}"), hpn_kh); }
  else if (buf[0] == 'p' || buf[0] == 'P') { hpn_ph_cal = param.toFloat(); if (hpn_ph_cal < 0.1f) hpn_ph_cal = 0.1f; Response_P(PSTR("{\"YC01\":{\"PHCal\":%.2f}}"), hpn_ph_cal); }
  else if (buf[0] == 'e' || buf[0] == 'E') { hpn_ec_cal = param.toFloat(); if (hpn_ec_cal < 0.1f) hpn_ec_cal = 0.1f; Response_P(PSTR("{\"YC01\":{\"ECCal\":%.2f}}"), hpn_ec_cal); }
  else { ResponseCmndFailed(); }
}

static void CmndHPNWater(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    const char* names[4] = {"Distilled", "SoftTap", "MediumTap", "HardTap"};
    Response_P(PSTR("{\"YC01\":{\"Water\":\"%s\",\"KH\":%.1f}}"), names[hpn_water_type], hpn_kh);
    return;
  }
  String s = XdrvMailbox.data;
  s.toLowerCase();
  float kh_vals[4] = {0.5f, 3.0f, 6.0f, 10.0f};
  int val = 1;
  if (s.startsWith("dis")) val = 0;
  else if (s.startsWith("soft")) val = 1;
  else if (s.startsWith("med")) val = 2;
  else if (s.startsWith("hard")) val = 3;
  hpn_water_type = val;
  hpn_kh = kh_vals[val];
  const char* names[4] = {"Distilled", "SoftTap", "MediumTap", "HardTap"};
  Response_P(PSTR("{\"YC01\":{\"Water\":\"%s\",\"KH\":%.1f}}"), names[val], hpn_kh);
}

static void CmndHPNVolume(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Volume\":%.1f}}"), hpn_volume_l);
    return;
  }
  hpn_volume_l = atof(XdrvMailbox.data);
  if (hpn_volume_l < 1.0f) hpn_volume_l = 20.0f;
  Response_P(PSTR("{\"YC01\":{\"Volume\":%.1f}}"), hpn_volume_l);
}

static void CmndHPNLatLon(void) {
  if (strlen(XdrvMailbox.data) == 0) {
    Response_P(PSTR("{\"YC01\":{\"Lat\":%.4f,\"Lon\":%.4f,\"Location\":\"%s\"}}"), hpn_lat, hpn_lon, hpn_location.c_str());
    return;
  }
  char buf[48];
  strlcpy(buf, XdrvMailbox.data, sizeof(buf));
  char *sp = strchr(buf, ' ');
  if (!sp) { ResponseCmndFailed(); return; }
  *sp = 0;
  hpn_lat = atof(buf);
  hpn_lon = atof(sp + 1);
  hpn_calc_sun_times();
  if (hpn_location.length() == 0) hpn_reverse_geocode(hpn_lat, hpn_lon);
  Response_P(PSTR("{\"YC01\":{\"Lat\":%.4f,\"Lon\":%.4f,\"Sunrise\":%d,\"Sunset\":%d,\"Location\":\"%s\"}}"),
             hpn_lat, hpn_lon, hpn_sunrise_min, hpn_sunset_min, hpn_location.c_str());
}

bool Xdrv89(uint32_t function) {
  bool result = false;
  switch (function) {case FUNC_INIT:         HPNInit(); break;
    case FUNC_EVERY_SECOND: hpn_autocontrol(); break;
    case FUNC_COMMAND:      result = DecodeCommand(kHPN_Commands,HPN_Commands); break;
    case FUNC_JSON_APPEND:  HPNJsonAppend(); break;
    case FUNC_WEB_SENSOR:   HPNWebSensor(); break;
    case FUNC_ACTIVE:       result = true; break;}
  return result;
}

#endif  // USE_HPN_ESP32
#endif  // CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
#endif  // ESP32
