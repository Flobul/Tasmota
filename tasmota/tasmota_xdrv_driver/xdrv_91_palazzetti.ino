/*
  xdrv_91_palazzetti.ino - Palazzetti support for Tasmota

  Copyright (C) 2024  Flobul

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#ifdef USE_PALAZZETTI

#define XDRV_91                   91
#define D_NAME_PALAZZETTI         "Palazzetti"
#define JSON_RESPONSE_SIZE        1536
#define PLZ_UDP_PORT              54549
#define PLZ_SETTINGS_TIMESYNC     1     //0=OFF; 1=ON
#define PLZ_SETTINGS_MQTT_TOPIC   0     //0=RAW value in one topic (ex: TELE/T1 = 19.20);
                                        //1=JSON in category topic (ex: TELE/TMPS = "{\"INFO\":"DATA":{"T1":23.20}...}");
                                        //2=RAW value in category topic (ex: TELE/TMPS/T1 = 19.20)
#define PLZ_ENDPOINT              "/cgi-bin/sendmsg.lua"
#include <Palazzetti.h>
#include <TasmotaSerial.h>
#include <time.h>
#include <WiFiUdp.h>

Palazzetti pala;
TasmotaSerial *plzSerial = nullptr;
WiFiUDP udpServer;
uint8_t pala_rx_pin = NOT_A_PIN;
uint8_t pala_tx_pin = NOT_A_PIN;
bool plz_connected = false;
bool isGetRequest;
bool isUdpRequest;
uint8_t tcnt = 0;
uint8_t plzInterval = 15;
const uint16_t PLZ_SETTINGS_VERSION = 0x0100;
unsigned long _lastAllStatusRefreshMillis = 0;
unsigned long lastAttemptTime = 0;
const unsigned long retryInterval = 60000;
time_t getTimeStamp() {
    return time(nullptr);
}

const char HTTP_PALAZZETTI_TITLE_S[]     PROGMEM = "{s}%s{m}%s ";
const char HTTP_PALAZZETTI_SIMPLE[]      PROGMEM = "{s}%s{m}%d %s{e}";
const char HTTP_PALAZZETTI_POURCENT[]    PROGMEM = "{s}%s{m}%d " D_UNIT_PERCENT" {e}";
const char HTTP_PALAZZETTI_TIME[]        PROGMEM = "{s}%s{m}%d " D_UNIT_HOUR " %d " D_UNIT_MINUTE "{e}";
const char HTTP_PALAZZETTI_TEMPERATURE[] PROGMEM = "{s}%s{m}%s " D_UNIT_DEGREE "%c{e}";
const char HTTP_PALAZZETTI_THERMOSTAT[]  PROGMEM = "{s}%s{m}%s " D_UNIT_DEGREE "%c ";
const char HTTP_PALAZZETTI_TITLE_D[]     PROGMEM = "{s}%s{m}%d ";
const char HTTP_PALAZZETTI_POWER_BTN[]   PROGMEM = "<button style=\"background:#%s\" onclick=\"fetch('" PLZ_ENDPOINT "?cmd=%s')\">%s</button>";
const char HTTP_PALAZZETTI_SLIDER_BTN[]  PROGMEM = "<td><button onclick=\"fetch('" PLZ_ENDPOINT "?cmd=%s');\">-</button></td>"
                                                       "<td><input type='range' min='%d' max='%d' step='1' value='%s' onchange=\"fetch('" PLZ_ENDPOINT "?cmd=%s+this.value');\" /></td>"
                                                       "<td><button onclick=\"fetch('" PLZ_ENDPOINT "?cmd=%s');\">+</button>{e}";
const char kPalazzetti_Commands[] PROGMEM = "Sendmsg|Init|Interval|Timesync|Mqtt";

enum Palazzetti_Commands {
    CMND_PALAZZETTI_SENDMSG,
    CMND_PALAZZETTI_INIT,
    CMND_PALAZZETTI_INTERVAL,
    CMND_PALAZZETTI_TIMESYNC,
    CMND_PALAZZETTI_MQTT
};

struct {
    Palazzetti::CommandResult cmdRes;
    char data[1024];  // Buffer global pour construire le JSON ("DATA": {"T1": 10.1,"T2":...})
    char info[256];   // Buffer temporaire pour chaque bloc de JSON ("INFO":{"CMD":...)
    char full[1280];  // Taille ajustée pour contenir les deux ({"INFO":{"CMD":"GET+SETP","DATA":...}})
    char msg[128];  // message lors d'erreur
    char cmd[16];  // cmd envoyée
    time_t ts;
} plzIJson;

struct {
    uint32_t crc32;                                     // To detect file changes
    uint16_t version;                                   // To detect driver function changes
    uint8_t  timesync;
    uint8_t  interval;
    uint8_t  mqtttopic;
} PalazzettiSettings;

struct Plz {
    uint16_t STATUS;
    uint16_t LSTATUS;
    uint16_t FSTATUS;
    float T1;
    float T2;
    float T3;
    float T4;
    float T5;
    uint16_t F1V;
    uint16_t F2V;
    uint16_t F1RPM;
    uint16_t F2L;
    uint16_t F2LF;
    bool isF3SF4SValid;
    float F3S;
    float F4S;
    bool isF3LF4LValid;
    uint16_t F3L;
    uint16_t F4L;
    bool isPWRValid;
    uint16_t IGN;
    uint16_t POWERTIMEh;
    uint16_t POWERTIMEm;
    uint16_t HEATTIMEh;
    uint16_t HEATTIMEm;
    uint16_t SERVICETIMEh;
    uint16_t SERVICETIMEm;
    uint16_t ONTIMEh;
    uint16_t ONTIMEm;
    uint16_t OVERTMPERRORS;
    uint16_t IGNERRORS;
    uint16_t PQT;
    char STOVE_DATETIME[20];
    byte STOVE_WDAY;
    float SETP;
    byte PWR;
    float FDR;
    bool isF2LValid;
    uint16_t FANLMINMAX[6];
    byte SLNT;
    uint16_t DPT;
    uint16_t DP;
    byte IN_I01;
    byte IN_I02;
    byte IN_I03;
    byte IN_I04;
    byte OUT_O01;
    byte OUT_O02;
    byte OUT_O03;
    byte OUT_O04;
    byte OUT_O05;
    byte OUT_O06;
    byte OUT_O07;
    char SN[28];
    uint16_t MOD;
    uint16_t VER;
    uint16_t CORE;
    char FWDATE[11];
    byte CHRSTATUS;
    float PCHRSETP[6];
    byte PSTART[6][2];
    byte PSTOP[6][2];
    byte DM[7][3];
    int MBTYPE;
    char APLTS[20];
    uint16_t APLWDAY;
    bool isMFSTATUSValid;
    uint16_t MFSTATUS;
    byte PUMP;
    byte IN;
    byte OUT;
    bool isSNValid;
    byte SNCHK;
    uint16_t FLUID;
    uint16_t SPLMIN;
    uint16_t SPLMAX;
    byte UICONFIG;
    byte HWTYPE;
    byte DSPTYPE;
    byte DSPFWVER;
    byte CONFIG;
    byte PELLETTYPE;
    uint16_t PSENSTYPE;
    byte PSENSLMAX;
    byte PSENSLTSH;
    byte PSENSLMIN;
    byte MAINTPROBE;
    byte STOVETYPE;
    byte FAN2TYPE;
    byte FAN2MODE;
    byte BLEMBMODE;
    byte BLEDSPMODE;
    byte CHRONOTYPE;
    byte AUTONOMYTYPE;
    byte NOMINALPWR;
    byte params[0x6A];
    uint16_t hiddenParams[0x6F];
} Plz;

#ifdef USE_WEBSERVER

#endif // USE_WEBSERVER

#define D_PLZ_POWER "Puissance"
#define D_PLZ_POWER_ON "Allumer"
#define D_PLZ_POWER_OFF "Éteindre"
#define D_PLZ_SET_POINT "Consigne"
#define D_PLZ_T1 "Température Ambiante / Départ"
#define D_PLZ_T2 "Température Sécurité / Retour"
#define D_PLZ_T3 "Température Fumées"
#define D_PLZ_T4 "Température MultiFire CC"
#define D_PLZ_T5 "Température Ambiante / Accumulateur"
#define D_PLZ_MAIN_FAN "Ventilateur central"
#define D_PLZ_FAN4 "Ventilateur droit"
#define D_PLZ_FAN3 "Ventilateur gauche"

#define D_PLZ_STATUS_0 "Eteint"
#define D_PLZ_STATUS_1 "Arrêté par minuterie"
#define D_PLZ_STATUS_2 "Essai d’allumage"
#define D_PLZ_STATUS_3 "Chargement granulés"
#define D_PLZ_STATUS_4 "Allumage"
#define D_PLZ_STATUS_5 "Contrôle combustion"
#define D_PLZ_STATUS_6 "En marche"
#define D_PLZ_STATUS_7 "En marche - Modulation"
#define D_PLZ_STATUS_8 "-"
#define D_PLZ_STATUS_9 "Stand-by"
#define D_PLZ_STATUS_10 "Extinction"
#define D_PLZ_STATUS_11 "Nettoyage brasier"
#define D_PLZ_STATUS_12 "Attente refroidissement"
#define D_PLZ_STATUS_51 "Eco mode"
#define D_PLZ_STATUS_241 "Erreur Nettoyage"
#define D_PLZ_STATUS_243 "Erreur Grille"
#define D_PLZ_STATUS_244 "NTC2 ALARM"
#define D_PLZ_STATUS_245 "NTC3 ALARM"
#define D_PLZ_STATUS_247 "Erreur Porte"
#define D_PLZ_STATUS_248 "Erreur Dépression"
#define D_PLZ_STATUS_249 "NTC1 ALARM"
#define D_PLZ_STATUS_250 "TC1 ALARM"
#define D_PLZ_STATUS_252 "Erreur évacuation Fumées"
#define D_PLZ_STATUS_253 "Pas de pellets"
#define D_PLZ_STATUS_501 "Éteint"
#define D_PLZ_STATUS_502 "Allumage"
#define D_PLZ_STATUS_503 "Contrôle combustion"
#define D_PLZ_STATUS_504 "En marche"
#define D_PLZ_STATUS_505 "Bûches terminées"
#define D_PLZ_STATUS_506 "Refroidissement"
#define D_PLZ_STATUS_507 "Nettoyage brasier"
#define D_PLZ_STATUS_509 "Allumage avec granulés"
#define D_PLZ_STATUS_1239 "Porte ouverte"
#define D_PLZ_STATUS_1240 "Température trop haute"
#define D_PLZ_STATUS_1241 "Avertissement nettoyage"
#define D_PLZ_STATUS_1243 "Erreur combustion"
#define D_PLZ_STATUS_1244 "Erreur sonde granulés ou eau de retour"
#define D_PLZ_STATUS_1245 "Erreur T05 sonde non raccordée ou bien défectueuse"
#define D_PLZ_STATUS_1247 "Porte ou couvercle de charge ouverts"
#define D_PLZ_STATUS_1248 "Erreur pressostat de sécurité"
#define D_PLZ_STATUS_1249 "Mauvais fonctionnement sonde principale"
#define D_PLZ_STATUS_1250 "Mauvais fonctionnement sonde fumées"
#define D_PLZ_STATUS_1252 "Température trop haute des fumées en sortie"
#define D_PLZ_STATUS_1253 "Granulés terminés ou non allumage"
#define D_PLZ_STATUS_1508 "Erreur générique - Voir Notice"

#define D_PLZ_STATUS_UNKNOWN "Erreur inconnu"
#define D_PLZ_STOVETYPE_UNKNOWN "Inconnu";
#define D_PLZ_STOVETYPE_AIR "Circulation d'air";
#define D_PLZ_STOVETYPE_WATER "Hydraulique";
#define D_PLZ_STOVETYPE_MULTIFIRE_AIR "Mixte bois/granulés air";
#define D_PLZ_STOVETYPE_MULTIFIRE_IDRO "Mixte bois/granulés hydraulique";
#define D_PLZ_STOVETYPE_KITCHEN_AIR "Cuisinière";
#define D_PLZ_STOVETYPE_KITCHEN_IDRO "Cuisinière hydrolique";
#define D_PLZ_HIGH "HAUT";
#define D_PLZ_PROP "Proportionel";

const char JSON_STR_TEMPLATE[] PROGMEM = "\"%s\":\"%s\",";
const char JSON_INT_TEMPLATE[] PROGMEM = "\"%s\":%d,";
const char JSON_ELE_TEMPLATE[] PROGMEM = "%d,";
const char JSON_BOOL_TEMPLATE[] PROGMEM = "\"%s\":%s,";
const char JSON_OBJ_TEMPLATE[] PROGMEM = "\"%s\":{";
const char JSON_ARRAY_TEMPLATE[] PROGMEM = "\"%s\":[";
const char JSON_INFO_TEMPLATE[] PROGMEM = "\"INFO\":{";
const char JSON_OBJ_CLOSE[] PROGMEM = "}";
const char JSON_ARRAY_CLOSE[] PROGMEM = "]";
const char JSON_COMMA[] PROGMEM = ",";
const char JSON_OBJ_CLOSE_COMMA[] PROGMEM = "},";
const char JSON_CMD[] PROGMEM = "CMD";
const char JSON_RSP[] PROGMEM = "RSP";
const char JSON_TS[] PROGMEM = "TS";
const char JSON_MSG[] PROGMEM = "MSG";

const char* commandResultToString() {
    switch (plzIJson.cmdRes) {
        case Palazzetti::CommandResult::OK:
            return "OK";
        case Palazzetti::CommandResult::ERROR:
            return "ERROR";
        case Palazzetti::CommandResult::COMMUNICATION_ERROR:
            return "COMM_ERROR";
        case Palazzetti::CommandResult::BUSY:
            return "BUSY";
        case Palazzetti::CommandResult::UNSUPPORTED:
            return "UNSUPPORTED";
        case Palazzetti::CommandResult::PARSER_ERROR:
            return "PARSER_ERROR";
        default:
            return "UNKNOWN";
    }
}

const char* getPlzStatusMessage(uint16_t statusCode) {
    switch (statusCode) {
        case 0:   return D_PLZ_STATUS_0;
        case 1:   return D_PLZ_STATUS_1;
        case 2:   return D_PLZ_STATUS_2;
        case 3:   return D_PLZ_STATUS_3;
        case 4:   return D_PLZ_STATUS_4;
        case 5:   return D_PLZ_STATUS_5;
        case 6:   return D_PLZ_STATUS_6;
        case 7:   return D_PLZ_STATUS_7;
        case 8:   return D_PLZ_STATUS_8;
        case 9:   return D_PLZ_STATUS_9;
        case 10:  return D_PLZ_STATUS_10;
        case 11:  return D_PLZ_STATUS_11;
        case 12:  return D_PLZ_STATUS_12;
        case 51:  return D_PLZ_STATUS_51;
        case 241: return D_PLZ_STATUS_241;
        case 243: return D_PLZ_STATUS_243;
        case 244: return D_PLZ_STATUS_244;
        case 245: return D_PLZ_STATUS_245;
        case 247: return D_PLZ_STATUS_247;
        case 248: return D_PLZ_STATUS_248;
        case 249: return D_PLZ_STATUS_249;
        case 250: return D_PLZ_STATUS_250;
        case 252: return D_PLZ_STATUS_252;
        case 253: return D_PLZ_STATUS_253;
        case 501: return D_PLZ_STATUS_501;
        case 502: return D_PLZ_STATUS_502;
        case 503: return D_PLZ_STATUS_503;
        case 504: return D_PLZ_STATUS_504;
        case 505: return D_PLZ_STATUS_505;
        case 506: return D_PLZ_STATUS_506;
        case 507: return D_PLZ_STATUS_507;
        case 509: return D_PLZ_STATUS_509;
        case 1239: return D_PLZ_STATUS_1239;
        case 1240: return D_PLZ_STATUS_1240;
        case 1241: return D_PLZ_STATUS_1241;
        case 1243: return D_PLZ_STATUS_1243;
        case 1244: return D_PLZ_STATUS_1244;
        case 1245: return D_PLZ_STATUS_1245;
        case 1247: return D_PLZ_STATUS_1247;
        case 1248: return D_PLZ_STATUS_1248;
        case 1249: return D_PLZ_STATUS_1249;
        case 1250: return D_PLZ_STATUS_1250;
        case 1252: return D_PLZ_STATUS_1252;
        case 1253: return D_PLZ_STATUS_1253;
        case 1508: return D_PLZ_STATUS_1508;
        default:  return D_PLZ_STATUS_UNKNOWN;
    }
}

const char* getPlzStoveType(uint16_t stoveType) {
    switch (stoveType) {
        case 0:   return D_PLZ_STOVETYPE_UNKNOWN;
        case 1:   return D_PLZ_STOVETYPE_AIR;
        case 2:   return D_PLZ_STOVETYPE_WATER;
        case 3:   return D_PLZ_STOVETYPE_MULTIFIRE_AIR;
        case 4:   return D_PLZ_STOVETYPE_MULTIFIRE_IDRO;
        case 5:   return D_PLZ_STOVETYPE_KITCHEN_AIR;
        case 6:   return D_PLZ_STOVETYPE_KITCHEN_IDRO;
        default:  return D_PLZ_STOVETYPE_UNKNOWN;
    }
}

const char* getPlzFanStatus(uint16_t fanCode) {
    switch (fanCode) {
        case 0:   return D_OFF;
        case 1:   return "1";
        case 2:   return "2";
        case 3:   return "3";
        case 4:   return "4";
        case 5:   return "5";
        case 6:   return D_PLZ_HIGH;
        case 7:   return D_AUTO;
        case 8:   return D_PLZ_PROP;
        default:  return "Unknown";
    }
}

void initJSON() {
    plzIJson.cmdRes = Palazzetti::CommandResult::ERROR;
    memset(plzIJson.data, 0, sizeof(plzIJson.data));
    memset(plzIJson.info, 0, sizeof(plzIJson.info));
    memset(plzIJson.full, 0, sizeof(plzIJson.full));
    memset(plzIJson.msg, 0, sizeof(plzIJson.msg));
    memset(plzIJson.cmd, 0, sizeof(plzIJson.cmd));
    plzIJson.ts = 0;
}

void JSONAddStrElem(const char* key, const char* value) {
    char temp[64];
    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, key, value);
    strcat_P(plzIJson.data, temp);
}

void JSONAddIntElem(const char* key, int value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_INT_TEMPLATE, key, value);
    strcat_P(plzIJson.data, temp);
}

void JSONAddIntArrElem(int value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_ELE_TEMPLATE, value);
    strcat_P(plzIJson.data, temp);
}

void JSONAddBoolElem(const char* key, bool value) {
    char temp[32];
    snprintf_P(temp, sizeof(temp), JSON_BOOL_TEMPLATE, key, value ? PSTR("true") : PSTR("false"));
    strcat_P(plzIJson.data, temp);
}

void JSONAddFloatElem(const char *key, float value) {//hack float
    char temp[32];
    String formattedValue = String(value, 2);
    snprintf(temp, sizeof(temp), JSON_BOOL_TEMPLATE, key, formattedValue.c_str());
    strcat_P(plzIJson.data, temp);
}

void JSONAddObj(const char* data) {
    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_OBJ_TEMPLATE, data);
    strcat_P(plzIJson.data, temp);
}

void JSONAddArray(const char* data) {
    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_ARRAY_TEMPLATE, data);
    strcat_P(plzIJson.data, temp);
}

void JSONAddArrayElem(const char* key, uint16_t* values, int count) {
    char temp[128];
    int offset = 0;
    offset += snprintf_P(temp, sizeof(temp), PSTR("\"%s\":["), key);
    for (int i = 0; i < count; ++i) {
        if (i > 0) {
            offset += snprintf_P(temp + offset, sizeof(temp) - offset, JSON_COMMA);
        }
        offset += snprintf_P(temp + offset, sizeof(temp) - offset, PSTR("%d"), values[i]);
    }
    offset += snprintf_P(temp + offset, sizeof(temp) - offset, PSTR("]"));
    strcat_P(plzIJson.data, temp);
}

void JSONAddCSVElem(const char* key, int value) {
    char line[50];
    snprintf(line, sizeof(line), "%s;%d\r\n", key, value);
    strncat(plzIJson.data, line, sizeof(plzIJson.data) - strlen(plzIJson.data) - 1);
}

void JSONMerge() {
    snprintf_P(plzIJson.info, sizeof(plzIJson.info), JSON_INFO_TEMPLATE);

    char temp[256];
    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_CMD, plzIJson.cmd);
    strcat_P(plzIJson.info, temp);

    snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_RSP, commandResultToString());
    strcat_P(plzIJson.info, temp);

    snprintf_P(temp, sizeof(temp), JSON_INT_TEMPLATE, JSON_TS, getTimeStamp());
    strcat_P(plzIJson.info, temp);

    if (plzIJson.msg[0] != 0 && strcmp(commandResultToString(), "OK") != 0) {        
        snprintf_P(temp, sizeof(temp), JSON_STR_TEMPLATE, JSON_MSG, plzIJson.msg);
        strcat_P(plzIJson.info, temp);
    }

    JSONRemoveComma(plzIJson.info);

    strcat_P(plzIJson.info, JSON_OBJ_CLOSE_COMMA);

    snprintf_P(plzIJson.full, sizeof(plzIJson.full), PSTR("{%s%s}"), plzIJson.info, plzIJson.data);
}

void JSONCloseObj() {
    JSONRemoveComma(plzIJson.data);
    strcat_P(plzIJson.data, JSON_OBJ_CLOSE);
}

void JSONCloseArray() {
    JSONRemoveComma(plzIJson.data);
    strcat_P(plzIJson.data, JSON_ARRAY_CLOSE);
}

void JSONAddComma() {
    strcat_P(plzIJson.data, JSON_COMMA);
}

void JSONRemoveComma(char* json) {
    int len = strlen(json);
    if (len > 0 && json[len - 1] == ',') {
        json[len - 1] = '\0';
    }
}

/* ======================================================================
Function: plzDrvInit
Purpose : Tasmota core driver init
Input   : -
Output  : -
Comments: -
====================================================================== */
void plzDrvInit(void)
{
    if (PinUsed(GPIO_PALAZZETTI_RX) && PinUsed(GPIO_PALAZZETTI_TX)) {
        pala_rx_pin = Pin(GPIO_PALAZZETTI_RX);
        pala_tx_pin = Pin(GPIO_PALAZZETTI_TX);
    }
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Pin Rx used %d and Tx %d"), pala_rx_pin, pala_tx_pin);
}

/* ======================================================================
Function: plzInit
Purpose : Tasmota core device init
Input   : -
Output  : -
Comments: -
====================================================================== */
void plzInit(void)
{
    if (!PinUsed(GPIO_PALAZZETTI_RX) || !PinUsed(GPIO_PALAZZETTI_TX)) { return; }
    int baudrate = 38400;

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: RX on GPIO%d, TX on GPIO%d, baudrate %d"), pala_rx_pin, pala_tx_pin, baudrate);

#ifdef ESP8266
    if (!plzSerial) {
        plzSerial = new TasmotaSerial(pala_rx_pin, pala_tx_pin, 2, 0);
    }
#endif  // ESP8266
#ifdef ESP32
    if (!plzSerial) {
        plzSerial = new TasmotaSerial(pala_rx_pin, pala_tx_pin, 1, 0);
    }
#endif  // ESP32

    initJSON();

    plzIJson.cmdRes = pala.initialize(
        std::bind(&myOpenSerial, std::placeholders::_1),
        std::bind(&myCloseSerial),
        std::bind(&mySelectSerial, std::placeholders::_1),
        std::bind(&myReadSerial, std::placeholders::_1, std::placeholders::_2),
        std::bind(&myWriteSerial, std::placeholders::_1, std::placeholders::_2),
        std::bind(&myDrainSerial),
        std::bind(&myFlushSerial),
        std::bind(&myUSleep, std::placeholders::_1)
    );

    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Palazzetti connected"));
        plz_connected = true;
        plzIJson.cmdRes = pala.getSN(&Plz.SN);
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande response Plz.SN=%s"), Plz.SN);

    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Palazzetti stove connection failed"));
    }

    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        plzUpdate();
    }
    udpServer.begin(PLZ_UDP_PORT);
}

/* ======================================================================
Function: plzSaveBeforeRestart
Purpose : 
Input   : -
Output  : -
Comments: -
====================================================================== */
void plzSaveBeforeRestart()
{
    if (PinUsed(GPIO_PALAZZETTI_TX)) {
        pinMode(Pin(GPIO_PALAZZETTI_TX), OUTPUT);
        digitalWrite(Pin(GPIO_PALAZZETTI_TX), HIGH);
    }
}

/* ======================================================================
Function: myOpenSerial
Purpose : Ouverture du port série
====================================================================== */
int myOpenSerial(uint32_t baudrate)
{
    if (plzSerial && plzSerial->begin(baudrate)) {
#ifdef ESP8266
        if (plzSerial->hardwareSerial() ) {
            ClaimSerial();

            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: using hardware serial"));
        } else {
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: using software serial"));
        }
#endif  // ESP8266
#ifdef ESP32
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: using hardserial %d"), plzSerial->getUart());
#endif // ESP32
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Serial initialization failed"));
        return -1;
    }
    return 0;
}

/* ======================================================================
Function: myCloseSerial
Purpose : Fermeture du port série
====================================================================== */
void myCloseSerial()
{
    plzSerial->end();
    pinMode(pala_tx_pin, OUTPUT);
    digitalWrite(pala_tx_pin, HIGH);
}

/* ======================================================================
Function: mySelectSerial
Purpose : Sélection du port série
====================================================================== */
int mySelectSerial(unsigned long timeout)
{
    size_t avail;
    unsigned long startmillis = millis();
    while ((avail = plzSerial->available()) == 0 && (startmillis + timeout) > millis())
        ;

    return avail;
}

/* ======================================================================
Function: myReadSerial
Purpose : Lecture sur le port série
====================================================================== */
size_t myReadSerial(void *buf, size_t count)
{
    return plzSerial->readBytes((char *)buf, count);
}

/* ======================================================================
Function: myWriteSerial
Purpose : Écriture sur le port série
====================================================================== */
size_t myWriteSerial(const void *buf, size_t count)
{
    return plzSerial->write((const uint8_t *)buf, count);
}

/* ======================================================================
Function: myDrainSerial
Purpose : Vider le buffer
====================================================================== */
int myDrainSerial()
{
    plzSerial->flush();
    return 0;
}

/* ======================================================================
Function: myFlushSerial
Purpose : Vider le buffer de réception et envoi
====================================================================== */
int myFlushSerial()
{
    plzSerial->flush();
    while (plzSerial->read() != -1)
        ;
    return 0;
}

/* ======================================================================
Function: myUSleep
Purpose : Pause en microsecondes
====================================================================== */
void myUSleep(unsigned long usecond)
{
    delayMicroseconds(usecond);
}

void PlzHandleUdpRequest() {
    if (udpServer) {
        int packetSize = udpServer.parsePacket();
        if (packetSize <= 0) return;

        String strData;
        strData.reserve(packetSize + 1);

        int bufferByte;
        while ((bufferByte = udpServer.read()) >= 0) {
            strData += (char)bufferByte;
        }

        isUdpRequest = true;

        if (strData.endsWith("bridge?")) { //plz, jot
            plzExecuteCmd("GET STDT");
        } else if (strData.endsWith("bridge?GET ALLS")) {
            plzExecuteCmd("GET ALLS");
        } else {
            plzExecuteCmd("");
        }

        if (plzIJson.full[0] != '\0') {
            udpServer.beginPacket(udpServer.remoteIP(), udpServer.remotePort());
            size_t length = strlen(plzIJson.full);
            size_t bytesSent = udpServer.write((const uint8_t *)plzIJson.full, length);

            if (bytesSent == length) {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: UDP message sent successfully: %s"), plzIJson.full);
            } else {
                AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Failed to send UDP message"));
            }
            udpServer.endPacket();
        } else {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: JSON message is empty"));
        }
    } else {
        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: UDP server is not initialized"));
    }
}

/* ======================================================================
Function: plzShow
Purpose : Display Palazzetti infos on WEB Interface
====================================================================== */
void plzShow(bool json) 
{
    if (json) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: plzShow json"));
        //WSContentSend_P(PSTR("{\"data\": \""), currentAllStatus.c_str(), PSTR("}"));
        return;
    }
#ifdef USE_WEBSERVER
    else {
        // {t}            = <table style='width:100%%'>
        // {s}            = <tr><th>
        // {m}            = </th><td style='width:20px;white-space:nowrap'>
        // {e}            = </td></tr>
        // HTTP_TABLE100  = <table style='width:100%%'>
                if (Plz.F2L == 0) {
                    //silent
                }
        
        bool hasSetPoint    = (Plz.SETP != 0);
        bool hasPower       = (Plz.STOVETYPE != 8);
        bool hasSwitch      = (Plz.STOVETYPE != 7 && Plz.STOVETYPE != 8);
        bool hasFirstFan    = (Plz.FAN2TYPE > 1);
        bool hasSecondFan   = (Plz.FAN2TYPE > 2); //hasZeroSpeedFan
        bool hasThirdFan    = (Plz.FAN2TYPE > 3);
        bool hasRemoteProbe = ((Plz.BLEMBMODE == 7 || Plz.BLEMBMODE == 17) && (Plz.BLEDSPMODE == 7 || Plz.BLEDSPMODE == 17));
        bool isFan3ASwitch  = (Plz.FANLMINMAX[2] == 0 && Plz.FANLMINMAX[3] == 1);
        bool isFan4ASwitch  = (Plz.FANLMINMAX[4] == 0 && Plz.FANLMINMAX[5] == 1);
        bool hasFanAuto     = (Plz.FAN2MODE == 2 || Plz.FAN2MODE == 3); // F2L==> 0=OFF ; 7=AUTO
        bool hasFanHigh     = (Plz.FAN2MODE == 3); // 6=HIGH
        bool hasFanProp     = (Plz.FAN2MODE == 4); // 8=PROPORTIONAL
        bool isIdroType     = (Plz.STOVETYPE == 2 || Plz.STOVETYPE == 4 || Plz.STOVETYPE == 6);
        bool hasError       = (Plz.LSTATUS >= 1000);
        bool isStopped      = (Plz.STATUS == 0 || Plz.LSTATUS == 1);
        bool isStarted      = (Plz.LSTATUS == 0 || Plz.LSTATUS == 1 || Plz.LSTATUS == 6 || Plz.LSTATUS == 7 || Plz.LSTATUS == 9 
                            || Plz.LSTATUS == 11 || Plz.LSTATUS == 12 || Plz.LSTATUS == 51 || Plz.LSTATUS == 501 || Plz.LSTATUS == 504
                            || Plz.LSTATUS == 505 || Plz.LSTATUS == 506 || Plz.LSTATUS == 507);

        if (hasSwitch) {
            if (isStopped) { 
                WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", "CMD+ON", D_ON);
            } else {
                if (isStarted) { 
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", "CMD+OFF", D_OFF);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ffc233", "CMD+OFF", D_OFF);
                }
            }
        }

        WSContentSend_P("{t}");
        if (hasSwitch) {
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_S, "Statut", getPlzStatusMessage(Plz.STATUS));
        }

        if (hasSetPoint) {
            WSContentSend_P(HTTP_PALAZZETTI_THERMOSTAT, D_PLZ_SET_POINT, String(Plz.SETP, 2).c_str(), D_UNIT_CELSIUS[0]);
            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, "SET+STPD", Plz.SPLMIN, Plz.SPLMAX, String(Plz.SETP, 2).c_str(), "SET+SETP", "SET+STPU");
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_POWER, Plz.PWR);

            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, "SET+PWRD", 1, 5, String(Plz.PWR).c_str(), "SET+POWR", "SET+PWRU");
        }

        if (hasFirstFan) {
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_S, D_PLZ_MAIN_FAN, getPlzFanStatus(Plz.F2L));
            int maxValue = (hasFanProp ? 8 : (hasFanAuto ? 7 : Plz.FANLMINMAX[1]));
            WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, "SET+FN2D", Plz.FANLMINMAX[0], maxValue, String(Plz.F2L).c_str(), "SET+RFAN", "SET+FN2U");
        }

        if (!hasSecondFan) {
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_FAN4, Plz.F4L);
            char cmndOff4[10];
            char cmndOn4[10];
            snprintf_P(cmndOff4, sizeof(cmndOff4), PSTR("SET+FN4L+%d"), Plz.FANLMINMAX[4]);
            snprintf_P(cmndOn4, sizeof(cmndOn4), PSTR("SET+FN4L+%d"), Plz.FANLMINMAX[5]);
            if (!isFan4ASwitch) {
                if (Plz.F4L == 0) {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", cmndOff4, D_ON);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", cmndOn4, D_OFF);
                }
            } else {
                WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, cmndOff4, Plz.FANLMINMAX[4], Plz.FANLMINMAX[5], Plz.F4L, "SET+FN4L", cmndOn4);
            }
        }

        if (!hasThirdFan) {
            WSContentSend_PD(HTTP_PALAZZETTI_TITLE_D, D_PLZ_FAN3, Plz.F3L);
            char cmndOff3[10];
            char cmndOn3[10];
            snprintf_P(cmndOff3, sizeof(cmndOff3), PSTR("SET+FN3L+%d"), Plz.FANLMINMAX[2]);
            snprintf_P(cmndOn3, sizeof(cmndOn3), PSTR("SET+FN3L+%d"), Plz.FANLMINMAX[3]);
            if (!isFan3ASwitch) {
                if (Plz.F3L == 0) {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "1bcc4d", cmndOff3, D_ON);
                } else {
                    WSContentSend_P(HTTP_PALAZZETTI_POWER_BTN, "ff3333", cmndOn3, D_OFF);
                }
            } else {
                WSContentSend_P(HTTP_PALAZZETTI_SLIDER_BTN, cmndOff3, Plz.FANLMINMAX[2], Plz.FANLMINMAX[3], Plz.F3L, "SET+FN3L", cmndOn3);
            }
        }

        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T1, String(Plz.T1).c_str(), D_UNIT_CELSIUS[0]);
        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T2, String(Plz.T2).c_str(), D_UNIT_CELSIUS[0]);
        WSContentSend_PD(HTTP_PALAZZETTI_TEMPERATURE, D_PLZ_T3, String(Plz.T3).c_str(), D_UNIT_CELSIUS[0]);
/*
        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, "Extracteur fumées", Plz.F1RPM, "trs/min");
        WSContentSend_PD(HTTP_PALAZZETTI_POURCENT, "Vitesse Ventilation F1V", Plz.F1V);
        WSContentSend_PD(HTTP_PALAZZETTI_POURCENT, "Vitesse Ventilation F2V", Plz.F2V);

        WSContentSend_PD(HTTP_PALAZZETTI_TIME, "Durée alimentation électrique", Plz.POWERTIMEh, Plz.POWERTIMEm);

        WSContentSend_PD(HTTP_PALAZZETTI_TIME, "Durée de chauffage", Plz.HEATTIMEh, Plz.HEATTIMEm);
        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, "Durée depuis dernier entretien", Plz.SERVICETIMEh, D_UNIT_HOUR);

        WSContentSend_PD(HTTP_PALAZZETTI_SIMPLE, "Pellets brûlés", Plz.PQT, D_UNIT_KILOGRAM);

        if (Plz.OVERTMPERRORS > 0) {
            WSContentSend_P(PSTR("<div style='color:red;'>Erreur de surchauffe détectée</div>"));
        }
        if (Plz.IGNERRORS > 0) {
            WSContentSend_P(PSTR("<div style='color:red;'>Allumages ratés</div>"));
        }
        if (Plz.LSTATUS >= 1000) {
            WSContentSend_P(PSTR("<div style='color:red;'>Erreur poële</div>"));
        }
        
        if (Plz.IGN == 1) {
            //WSContentSend_P(PSTR("<div>Brûleur en marche</div>"));
        } else {
            //WSContentSend_P(PSTR("<div>Brûleur éteint</div>"));
        }
*/
        WSContentSend_P(PSTR("</table>"));

    }
#endif  // USE_WEBSERVER
}

#ifdef USE_WEBSERVER

void plzRequestHandler(void) {
    if (!HttpCheckPriviledgedAccess()) {
        return;
    }
    isGetRequest = true;
    char cmd[14];
    WebGetArg(PSTR("cmd"), cmd, sizeof(cmd));

    if (strlen(cmd) > 0) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: SENDMSG received with command: %s"), cmd);
        if (plzExecuteCmd(cmd)) {
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd successful"));
        } else {
            AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Commande échouée:"));
        }
    }
}
#endif  // USE_WEBSERVER

void plzUpdate(void)
{
    if (++tcnt < plzInterval) return;
    tcnt = 0;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: update"));

    char* cmdList[] = {
        (char*)"GET TIME", (char*)"GET SETP", (char*)"GET POWR", (char*)"GET DPRS",
        (char*)"GET STAT", (char*)"GET TMPS", (char*)"GET FAND", (char*)"GET CNTR"
    };

    for (char* cmd : cmdList) {
        if (!plzExecuteCmd(cmd)) {
            AddLog(LOG_LEVEL_ERROR, PSTR("Command failed: %s"), cmd);
            break;
        }
    }
}

void getStatus() {
    plzIJson.cmdRes = pala.getStatus(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("STATUS", Plz.STATUS);
        JSONAddIntElem("LSTATUS", Plz.LSTATUS);
        JSONAddIntElem("FSTATUS", Plz.FSTATUS);
    }
}

void setSwitchOn() {
    plzIJson.cmdRes = pala.switchOn(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("STATUS", Plz.STATUS);
        JSONAddIntElem("LSTATUS", Plz.LSTATUS);
        JSONAddIntElem("FSTATUS", Plz.FSTATUS);
    }
}

void setSwitchOff() {
    plzIJson.cmdRes = pala.switchOff(&Plz.STATUS, &Plz.LSTATUS, &Plz.FSTATUS);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("STATUS", Plz.STATUS);
        JSONAddIntElem("LSTATUS", Plz.LSTATUS);
        JSONAddIntElem("FSTATUS", Plz.FSTATUS);
    }
}

void getLabel() {
    plzIJson.cmdRes = Palazzetti::CommandResult::OK;
    JSONAddStrElem("LABEL", NetworkHostname());
}

void getAllTemps() {
    plzIJson.cmdRes = pala.getAllTemps(&Plz.T1, &Plz.T2, &Plz.T3, &Plz.T4, &Plz.T5);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("T1", Plz.T1);
        JSONAddFloatElem("T2", Plz.T2);
        JSONAddFloatElem("T3", Plz.T3);
        JSONAddFloatElem("T4", Plz.T4);
        JSONAddFloatElem("T5", Plz.T5);
    }
}

void getFanData() {
    plzIJson.cmdRes = pala.getFanData(&Plz.F1V, &Plz.F2V, &Plz.F1RPM, &Plz.F2L, &Plz.F2LF, &Plz.isF3SF4SValid, &Plz.F3S, &Plz.F4S, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("F1V", Plz.F1V);
        JSONAddIntElem("F2V", Plz.F2V);
        JSONAddIntElem("F1RPM", Plz.F1RPM);
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
        if (Plz.isF3SF4SValid) {
            JSONAddFloatElem("F3S", Plz.F3S);
            JSONAddFloatElem("F4S", Plz.F4S);
        }
        if (Plz.isF3LF4LValid) {
            JSONAddIntElem("F3L", Plz.F3L);
            JSONAddIntElem("F4L", Plz.F4L);
        }
    }
}

void setRoomFan(const String &cmd) {
    int fanLevel = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setRoomFan(fanLevel, &Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            JSONAddIntElem("PWR", Plz.PWR);
        }
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
    }
}

void setRoomFanUp() {
    plzIJson.cmdRes = pala.setRoomFanUp(&Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            JSONAddIntElem("PWR", Plz.PWR);
        }
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
    }
}

void setRoomFanDown() {
    plzIJson.cmdRes = pala.setRoomFanDown(&Plz.isPWRValid, &Plz.PWR, &Plz.F2L, &Plz.F2LF);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (Plz.isPWRValid) {
            JSONAddIntElem("PWR", Plz.PWR);
        }
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
    }
}

void setRoomFan3(const String &cmd) {
    int fanLevel = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setRoomFan3(fanLevel, &Plz.F3L);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("F3L", Plz.F3L);
    }
}

void setRoomFan4(const String &cmd) {
    int fanLevel = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setRoomFan4(fanLevel, &Plz.F4L);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("F4L", Plz.F4L);
    }
}

void setSilentMode(const String &cmd) {
    int silentMode = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setSilentMode(silentMode, &Plz.SLNT, &Plz.PWR, &Plz.F2L, &Plz.F2LF, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("SLNT", Plz.SLNT);
        JSONAddIntElem("PWR", Plz.PWR);
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
        if (Plz.isF3LF4LValid) {
            JSONAddIntElem("F3L", Plz.F3L);
            JSONAddIntElem("F4L", Plz.F4L);
        }
    }
}

void getCounters() {
    plzIJson.cmdRes = pala.getCounters(&Plz.IGN, &Plz.POWERTIMEh, &Plz.POWERTIMEm, &Plz.HEATTIMEh, &Plz.HEATTIMEm, &Plz.SERVICETIMEh, &Plz.SERVICETIMEm, &Plz.ONTIMEh, &Plz.ONTIMEm, &Plz.OVERTMPERRORS, &Plz.IGNERRORS, &Plz.PQT);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("IGN", Plz.IGN);
        JSONAddIntElem("POWERTIMEh", Plz.POWERTIMEh);
        JSONAddIntElem("POWERTIMEm", Plz.POWERTIMEm);
        JSONAddIntElem("HEATTIMEh", Plz.HEATTIMEh);
        JSONAddIntElem("HEATTIMEm", Plz.HEATTIMEm);
        JSONAddIntElem("SERVICETIMEh", Plz.SERVICETIMEh);
        JSONAddIntElem("SERVICETIMEm", Plz.SERVICETIMEm);
        JSONAddIntElem("ONTIMEh", Plz.ONTIMEh);
        JSONAddIntElem("ONTIMEm", Plz.ONTIMEm);
        JSONAddIntElem("OVERTMPERRORS", Plz.OVERTMPERRORS);
        JSONAddIntElem("IGNERRORS", Plz.IGNERRORS);
        JSONAddIntElem("PQT", Plz.PQT);
    }
}

void getDateTime() {
    plzIJson.cmdRes = pala.getDateTime(&Plz.STOVE_DATETIME, &Plz.STOVE_WDAY);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("STOVE_DATETIME", Plz.STOVE_DATETIME);
        JSONAddIntElem("STOVE_WDAY", Plz.STOVE_WDAY);
    }
}

void setDateTime(const String &cmd) {
    int year = cmd.substring(9).toInt();
    int month = cmd.substring(14, 16).toInt();
    int day = cmd.substring(17, 19).toInt();
    int hour = cmd.substring(20, 22).toInt();
    int minute = cmd.substring(23, 25).toInt();
    int second = cmd.substring(26).toInt();

    if (year < 2000 || year > 2099) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Year"));
        return;
    } else if (month < 1 || month > 12) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Month"));
        return;
    } else if ((day < 1 || day > 31) ||
               ((month == 4 || month == 6 || month == 9 || month == 11) && day > 30) ||
               (month == 2 && day > 29) ||
               (month == 2 && day == 29 && !(((year % 4 == 0) && (year % 100 != 0)) || (year % 400 == 0)))) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Day"));
        return;
    } else if (hour > 23) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Hour"));
        return;
    } else if (minute > 59) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Minute"));
        return;
    } else if (second > 59) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Second"));
        return;
    }

    plzIJson.cmdRes = pala.setDateTime(year, month, day, hour, minute, second, &Plz.STOVE_DATETIME, &Plz.STOVE_WDAY);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("STOVE_DATETIME", Plz.STOVE_DATETIME);
        JSONAddIntElem("STOVE_WDAY", Plz.STOVE_WDAY);
    }
}

void getSetpoint() {
    plzIJson.cmdRes = pala.getSetPoint(&Plz.SETP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("SETP", Plz.SETP);
    }
}

void setSetpoint(const String &cmd) {
    int setpoint = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setSetpoint(static_cast<byte>(setpoint), &Plz.SETP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("SETP", Plz.SETP);
    }
}

void setSetpointUp() {
    plzIJson.cmdRes = pala.setSetPointUp(&Plz.SETP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("SETP", Plz.SETP);
    }
}

void setSetpointDown() {
    plzIJson.cmdRes = pala.setSetPointDown(&Plz.SETP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("SETP", Plz.SETP);
    }
}

void setSetpointFloat(const String &cmd) {
    int intPart = cmd.substring(9, 13).toInt();
    int decPart = cmd.substring(14).toInt();
    if (decPart > 80 || decPart % 20 != 0) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Incorrect Year %d.%d"), intPart, decPart);
        return;
    }
    float setpointFloat = static_cast<float>(decPart) / 100.0f + static_cast<float>(intPart);
    plzIJson.cmdRes = pala.setSetpoint(setpointFloat, &Plz.SETP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddFloatElem("SETP", Plz.SETP);
    }
}

void getAllStatus() {
    unsigned long currentMillis = millis();
    bool refreshStatus = ((currentMillis - _lastAllStatusRefreshMillis) > 15000UL);
    plzIJson.cmdRes = pala.getAllStatus(refreshStatus, &Plz.MBTYPE, &Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE, &Plz.APLTS, &Plz.APLWDAY, &Plz.CHRSTATUS, &Plz.STATUS, &Plz.LSTATUS, &Plz.isMFSTATUSValid, &Plz.MFSTATUS, &Plz.SETP, &Plz.PUMP, &Plz.PQT, &Plz.F1V, &Plz.F1RPM, &Plz.F2L, &Plz.F2LF, &Plz.FANLMINMAX, &Plz.F2V, &Plz.isF3LF4LValid, &Plz.F3L, &Plz.F4L, &Plz.PWR, &Plz.FDR, &Plz.DPT, &Plz.DP, &Plz.IN, &Plz.OUT, &Plz.T1, &Plz.T2, &Plz.T3, &Plz.T4, &Plz.T5, &Plz.isSNValid, &Plz.SN);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        if (refreshStatus) {
            _lastAllStatusRefreshMillis = currentMillis;
        }
        JSONAddIntElem("MBTYPE", Plz.MBTYPE);
        JSONAddIntElem("MOD", Plz.MOD);
        JSONAddIntElem("VER", Plz.VER);
        JSONAddIntElem("CORE", Plz.CORE);
        JSONAddStrElem("FWDATE", Plz.FWDATE);
        JSONAddStrElem("APLTS", Plz.APLTS);
        JSONAddIntElem("APLWDAY", Plz.APLWDAY);
        JSONAddIntElem("CHRSTATUS", Plz.CHRSTATUS);
        JSONAddIntElem("STATUS", Plz.STATUS);
        JSONAddIntElem("LSTATUS", Plz.LSTATUS);
        JSONAddFloatElem("SETP", Plz.SETP);
        JSONAddIntElem("PUMP", Plz.PUMP);
        JSONAddIntElem("PQT", Plz.PQT);
        JSONAddIntElem("F1V", Plz.F1V);
        JSONAddIntElem("F1RPM", Plz.F1RPM);
        JSONAddIntElem("F2L", Plz.F2L);
        JSONAddIntElem("F2LF", Plz.F2LF);
        JSONAddIntElem("F2V", Plz.F2V);
        JSONAddIntElem("PWR", Plz.PWR);
        JSONAddFloatElem("FDR", Plz.FDR);
        JSONAddIntElem("DPT", Plz.DPT);
        JSONAddIntElem("DP", Plz.DP);
        JSONAddIntElem("IN", Plz.IN);
        JSONAddIntElem("OUT", Plz.OUT);
        JSONAddFloatElem("T1", Plz.T1);
        JSONAddFloatElem("T2", Plz.T2);
        JSONAddFloatElem("T3", Plz.T3);
        JSONAddFloatElem("T4", Plz.T4);
        JSONAddFloatElem("T5", Plz.T5);
        if (Plz.isSNValid) {
            JSONAddStrElem("SN", Plz.SN);
        }
        JSONAddIntElem("EFLAGS", 0);
    }
}

void getStaticData() {
    plzIJson.cmdRes = pala.getStaticData(&Plz.SN, &Plz.SNCHK, &Plz.MBTYPE, &Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE, &Plz.FLUID, &Plz.SPLMIN, &Plz.SPLMAX, &Plz.UICONFIG, &Plz.HWTYPE, &Plz.DSPTYPE, &Plz.DSPFWVER, &Plz.CONFIG, &Plz.PELLETTYPE, &Plz.PSENSTYPE, &Plz.PSENSLMAX, &Plz.PSENSLTSH, &Plz.PSENSLMIN, &Plz.MAINTPROBE, &Plz.STOVETYPE, &Plz.FAN2TYPE, &Plz.FAN2MODE, &Plz.BLEMBMODE, &Plz.BLEDSPMODE, &Plz.CHRONOTYPE, &Plz.AUTONOMYTYPE, &Plz.NOMINALPWR);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("LABEL", NetworkHostname());
        JSONAddStrElem("GWDEVICE", "wlan0");
        JSONAddStrElem("MAC", WiFi.macAddress().c_str());
        JSONAddStrElem("GATEWAY", WiFi.gatewayIP().toString().c_str());
        JSONAddStrElem("DNS", WiFi.dnsIP().toString().c_str());
        JSONAddStrElem("WMAC", WiFi.macAddress().c_str());
        JSONAddStrElem("WMODE", (WiFi.getMode() & WIFI_STA) ? "sta" : "ap");
        JSONAddStrElem("WADR", WiFi.localIP().toString().c_str());
        JSONAddStrElem("WGW", WiFi.gatewayIP().toString().c_str());
        JSONAddIntElem("WPWR", WiFi.RSSI());
        JSONAddStrElem("WSSID", WiFi.SSID().c_str());
        JSONAddStrElem("WPR", WiFi.isConnected() ? "dhcp" : "static");
        JSONAddStrElem("WMSK", WiFi.subnetMask().toString().c_str());
        JSONAddIntElem("WCH", WiFi.channel());

#if defined(ESP32) && defined(USE_ETHERNET)
        JSONAddStrElem("EPR", "dhcp");
        JSONAddStrElem("EGW", ETH.gatewayIP().toString().c_str());
        JSONAddStrElem("EMSK", ETH.subnetMask().toString().c_str());
        JSONAddStrElem("EADR", ETH.localIP().toString().c_str());
        JSONAddStrElem("EMAC", ETH.macAddress().c_str());
        JSONAddStrElem("ECBL", ETH.linkUp() ? "up" : "down");
        JSONAddStrElem("EBCST", ETH.broadcastIP().toString().c_str());
#else
        JSONAddStrElem("EPR", "dhcp");
        JSONAddStrElem("EGW", "0.0.0.0");
        JSONAddStrElem("EMSK", "0.0.0.0");
        JSONAddStrElem("EADR", "0.0.0.0");
        JSONAddStrElem("EMAC", WiFi.macAddress().c_str());
        JSONAddStrElem("ECBL", "down");
        JSONAddStrElem("EBCST", "");
#endif

        bool internetConnected = (WiFi.status() == WL_CONNECTED);
        JSONAddIntElem("APLCONN", 1);
        JSONAddIntElem("ICONN", internetConnected ? 1 : 0);
        JSONAddStrElem("CBTYPE", "miniembplug");
        JSONAddStrElem("sendmsg", "2.1.2 2018-03-28 10:19:09");
        JSONAddStrElem("plzbridge", "2.2.1 2022-10-24 11:13:21");
        JSONAddStrElem("SYSTEM", "2.5.3 2021-10-08 10:30:20 (657c8cf)");
        JSONAddBoolElem("CLOUD_ENABLED", true);

        JSONAddStrElem("SN", Plz.SN);
        JSONAddIntElem("SNCHK", Plz.SNCHK);
        JSONAddIntElem("MBTYPE", Plz.MBTYPE);
        JSONAddIntElem("MOD", Plz.MOD);
        JSONAddIntElem("VER", Plz.VER);
        JSONAddIntElem("CORE", Plz.CORE);
        JSONAddStrElem("FWDATE", Plz.FWDATE);
        JSONAddIntElem("FLUID", Plz.FLUID);
        JSONAddIntElem("SPLMIN", Plz.SPLMIN);
        JSONAddIntElem("SPLMAX", Plz.SPLMAX);
        JSONAddIntElem("UICONFIG", Plz.UICONFIG);
        JSONAddIntElem("HWTYPE", Plz.HWTYPE);
        JSONAddIntElem("DSPTYPE", Plz.DSPTYPE);
        JSONAddIntElem("DSPFWVER", Plz.DSPFWVER);
        JSONAddIntElem("CONFIG", Plz.CONFIG);
        JSONAddIntElem("PELLETTYPE", Plz.PELLETTYPE);
        JSONAddIntElem("PSENSTYPE", Plz.PSENSTYPE);
        JSONAddIntElem("PSENSLMAX", Plz.PSENSLMAX);
        JSONAddIntElem("PSENSLTSH", Plz.PSENSLTSH);
        JSONAddIntElem("PSENSLMIN", Plz.PSENSLMIN);
        JSONAddIntElem("MAINTPROBE", Plz.MAINTPROBE);
        JSONAddIntElem("STOVETYPE", Plz.STOVETYPE);
        JSONAddIntElem("FAN2TYPE", Plz.FAN2TYPE);
        JSONAddIntElem("FAN2MODE", Plz.FAN2MODE);
        JSONAddIntElem("BLEMBMODE", Plz.BLEMBMODE);
        JSONAddIntElem("BLEDSPMODE", Plz.BLEDSPMODE);
        JSONAddIntElem("CHRONOTYPE", Plz.CHRONOTYPE);
        JSONAddIntElem("AUTONOMYTYPE", Plz.AUTONOMYTYPE);
        JSONAddIntElem("NOMINALPWR", Plz.NOMINALPWR);
    }
}

void getPower() {
    plzIJson.cmdRes = pala.getPower(&Plz.PWR, &Plz.FDR);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("PWR", Plz.PWR);
        JSONAddFloatElem("FDR", Plz.FDR);
    }
}

void setPower(byte powerLevel) {
    plzIJson.cmdRes = pala.setPower(powerLevel, &Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            JSONAddIntElem("F2L", Plz.F2L);
        }
        JSONAddArrayElem("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

void setPowerUp() {
    plzIJson.cmdRes = pala.setPowerUp(&Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            JSONAddIntElem("F2L", Plz.F2L);
        }
        JSONAddArrayElem("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

void setPowerDown() {
    plzIJson.cmdRes = pala.setPowerDown(&Plz.PWR, &Plz.isF2LValid, &Plz.F2L, &Plz.FANLMINMAX);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("PWR", Plz.PWR);
        if (Plz.isF2LValid) {
            JSONAddIntElem("F2L", Plz.F2L);
        }
        JSONAddArrayElem("FANLMINMAX", Plz.FANLMINMAX, 6);
    }
}

void getDPressData() {
    plzIJson.cmdRes = pala.getDPressData(&Plz.DPT, &Plz.DP);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("DPT", Plz.DPT);
        JSONAddIntElem("DP", Plz.DP);
    }
}

void getIO() {
    plzIJson.cmdRes = pala.getIO(&Plz.IN_I01, &Plz.IN_I02, &Plz.IN_I03, &Plz.IN_I04, &Plz.OUT_O01, &Plz.OUT_O02, &Plz.OUT_O03, &Plz.OUT_O04, &Plz.OUT_O05, &Plz.OUT_O06, &Plz.OUT_O07);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("IN_I01", Plz.IN_I01);
        JSONAddIntElem("IN_I02", Plz.IN_I02);
        JSONAddIntElem("IN_I03", Plz.IN_I03);
        JSONAddIntElem("IN_I04", Plz.IN_I04);
        JSONAddIntElem("OUT_O01", Plz.OUT_O01);
        JSONAddIntElem("OUT_O02", Plz.OUT_O02);
        JSONAddIntElem("OUT_O03", Plz.OUT_O03);
        JSONAddIntElem("OUT_O04", Plz.OUT_O04);
        JSONAddIntElem("OUT_O05", Plz.OUT_O05);
        JSONAddIntElem("OUT_O06", Plz.OUT_O06);
        JSONAddIntElem("OUT_O07", Plz.OUT_O07);
    }
}

void getSN() {
    plzIJson.cmdRes = pala.getSN(&Plz.SN);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("SERN", Plz.SN);
    }
}

void getModelVersion() {
    plzIJson.cmdRes = pala.getModelVersion(&Plz.MOD, &Plz.VER, &Plz.CORE, &Plz.FWDATE);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("MOD", Plz.MOD);
        JSONAddIntElem("VER", Plz.VER);
        JSONAddIntElem("CORE", Plz.CORE);
        JSONAddStrElem("FWDATE", Plz.FWDATE);
    }
}

void getChronoData() {
    plzIJson.cmdRes = pala.getChronoData(&Plz.CHRSTATUS, &Plz.PCHRSETP, &Plz.PSTART, &Plz.PSTOP, &Plz.DM);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("CHRSTATUS", Plz.CHRSTATUS);
        JSONAddObj("Programs");
        for (byte i = 0; i < 6; i++) {
            JSONAddObj(String("P" + String(i + 1)).c_str());
            JSONAddFloatElem("CHRSETP", Plz.PCHRSETP[i]);
            char time[6] = {'0', '0', ':', '0', '0', 0};
            time[0] = Plz.PSTART[i][0] / 10 + '0';
            time[1] = Plz.PSTART[i][0] % 10 + '0';
            time[3] = Plz.PSTART[i][1] / 10 + '0';
            time[4] = Plz.PSTART[i][1] % 10 + '0';
            JSONAddStrElem("START", time);
            time[0] = Plz.PSTOP[i][0] / 10 + '0';
            time[1] = Plz.PSTOP[i][0] % 10 + '0';
            time[3] = Plz.PSTOP[i][1] / 10 + '0';
            time[4] = Plz.PSTOP[i][1] % 10 + '0';
            JSONAddStrElem("STOP", time);
            JSONCloseObj();
        }
        JSONCloseObj();
        JSONAddObj("Days");
        for (byte dayNumber = 0; dayNumber < 7; dayNumber++) {
            JSONAddObj(String("D" + String(dayNumber + 1)).c_str());
            for (byte memoryNumber = 0; memoryNumber < 3; memoryNumber++) {
                char memoryName[3] = {'M', (char)(memoryNumber + '1'), 0};
                JSONAddStrElem(memoryName, Plz.DM[dayNumber][memoryNumber] ? String("P" + String(Plz.DM[dayNumber][memoryNumber])).c_str() : "OFF");
            }
            JSONCloseObj();
        }
        JSONCloseObj();
    }
}

void setChronoStatus(const String &cmd) {
    int status = cmd.substring(9).toInt();
    plzIJson.cmdRes = pala.setChronoStatus(status, &Plz.CHRSTATUS);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddIntElem("CHRSTATUS", Plz.CHRSTATUS);
    }
}

void setChronoStartHH(const String &cmd) {
    int programNumber = cmd.substring(9, 11).toInt();
    int startHour = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setChronoStartHH(programNumber, startHour);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("Result", "OK");
    }
}

void setChronoStartMM(const String &cmd) {
    int programNumber = cmd.substring(9, 11).toInt();
    int startMinute = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setChronoStartMM(programNumber, startMinute);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("Result", "OK");
    }
}

void setChronoStopHH(const String &cmd) {
    int programNumber = cmd.substring(9, 11).toInt();
    int stopHour = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setChronoStopHH(programNumber, stopHour);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("Result", "OK");
    }
}

void setChronoStopMM(const String &cmd) {
    int programNumber = cmd.substring(9, 11).toInt();
    int stopMinute = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setChronoStopMM(programNumber, stopMinute);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("Result", "OK");
    }
}

void setChronoSetpoint(const String &cmd) {
    int programNumber = cmd.substring(9, 11).toInt();
    int setpoint = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setChronoSetpoint(programNumber, setpoint);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        JSONAddStrElem("Result", "OK");
    }
}

void setChronoDay(const String &cmd) {
    int day = cmd.substring(9, 11).toInt();
    int memory = cmd.substring(12, 14).toInt();
    int program = cmd.substring(15).toInt();
    plzIJson.cmdRes = pala.setChronoDay(day, memory, program);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char dayName[3] = {'D', static_cast<char>(day + '0'), '\0'};
        char memoryName[3] = {'M', static_cast<char>(memory + '0'), '\0'};
        char programName[3] = {'P', static_cast<char>(program + '0'), '\0'};

        JSONAddObj(dayName);
        JSONAddObj(memoryName);
        if (program) {
            JSONAddStrElem("Program", programName);
        } else {
            JSONAddStrElem("Program", "OFF");
        }
        JSONCloseObj(); // Close memory object
        JSONCloseObj(); // Close day object
    }
}

void setChronoPrg(const String &cmd) {
    int program = cmd.substring(9, 11).toInt();
    int setpoint = cmd.substring(12, 16).toInt();
    int startHour = cmd.substring(17, 19).toInt();
    int startMinute = cmd.substring(20, 22).toInt();
    int stopHour = cmd.substring(23, 25).toInt();
    int stopMinute = cmd.substring(26).toInt();
    
    plzIJson.cmdRes = pala.setChronoPrg(program, setpoint, startHour, startMinute, stopHour, stopMinute);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char programName[3] = {'P', static_cast<char>(program + '0'), '\0'};
        float chrsetp = static_cast<float>(setpoint) / 10.0f; // Assuming setpoint is in tenths

        char startTime[6];
        snprintf(startTime, sizeof(startTime), "%02d:%02d", startHour, startMinute);

        char stopTime[6];
        snprintf(stopTime, sizeof(stopTime), "%02d:%02d", stopHour, stopMinute);

        JSONAddObj(programName);
        JSONAddFloatElem("CHRSETP", chrsetp);
        JSONAddStrElem("START", startTime);
        JSONAddStrElem("STOP", stopTime);
        JSONCloseObj(); // Close program object
    }
}

void getParameter(byte param, const char *prefix) {
    byte paramValue;
    plzIJson.cmdRes = pala.getParameter(param, &paramValue);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, paramValue);
        JSONAddIntElem(key, paramValue);
    }
}

void setParameter(const String &cmd, const char *prefix) {
    int paramIndex = cmd.substring(9, 11).toInt();
    int paramValue = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setParameter(paramIndex, paramValue);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, paramIndex);
        JSONAddIntElem(key, paramValue);
    }
}

void getHiddenParameter(byte param, const char *prefix) {
    uint16_t hiddenParamValue;
    plzIJson.cmdRes = pala.getHiddenParameter(param, &hiddenParamValue);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, param);
        JSONAddIntElem(key, hiddenParamValue);
    }
}

void setHiddenParameter(const String &cmd, const char *prefix) {
    int paramIndex = cmd.substring(9, 11).toInt();
    int paramValue = cmd.substring(12).toInt();
    plzIJson.cmdRes = pala.setHiddenParameter(paramIndex, paramValue);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        char key[8];
        snprintf(key, sizeof(key), PSTR("%s%d"), prefix, paramIndex);
        JSONAddIntElem(key, paramValue);
    }
}

#ifdef USE_WEBSERVER
void getAllParameters(char* fileType) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getAllParameters=%s"), fileType);

    if (!isValidFileType(fileType)) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Unknown filetype: %s"), fileType);
        plzIJson.cmdRes = Palazzetti::CommandResult::UNSUPPORTED;
        WSSend(200, CT_APP_JSON, plzIJson.full);
        return;
    }
    plzIJson.cmdRes = pala.getAllParameters(&Plz.params);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        sendParametersResponse(fileType, Plz.params, 0x6A, "PARM");
    }
}

bool isValidFileType(const char* fileType) {
    return (strcmp(fileType, "CSV") == 0 || strcmp(fileType, "JSON") == 0);
}

void getAllHiddenParameters(char* fileType) {
    AddLog(LOG_LEVEL_INFO, PSTR("Free heap: %d"), ESP.getFreeHeap());
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande getAllHiddenParameters=%s"), fileType);

    if (!isValidFileType(fileType)) {
        snprintf(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Unknown filetype: %s"), fileType);
        plzIJson.cmdRes = Palazzetti::CommandResult::UNSUPPORTED;
        WSSend(200, CT_APP_JSON, plzIJson.full);
        return;
    }
    plzIJson.cmdRes = pala.getAllHiddenParameters(&Plz.hiddenParams);
    if (plzIJson.cmdRes == Palazzetti::CommandResult::OK) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande CommandResult=%s"), fileType);
        sendParametersResponse(fileType, Plz.hiddenParams, 0x6F, "HPAR");
    }
}

void sendParametersResponse(char* fileType, const void* params, size_t paramCount, const char* paramType) {
    //char attachment[100];

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande sendParametersResponse=%s"), fileType);
    if (strcmp(fileType, "CSV") == 0 ) {
        //JSONAddObj(paramType);
        char header[50];
        snprintf(header, sizeof(header), "%s;VALUE\r\n", paramType);
        strncat(plzIJson.data, header, sizeof(plzIJson.data) - strlen(plzIJson.data) - 1);

        for (size_t i = 0; i < paramCount; i++) {
            JSONAddCSVElem(String(i).c_str(), ((const byte*)params)[i]);
        }

/*
        char temp[1024];

        toReturn += String(paramType) + F(";VALUE\r\n");
        for (size_t i = 0; i < paramCount; i++) {
            toReturn += String(i) + ';' + String(((const byte*)params)[i]) + '\r' + '\n';
        }
        snprintf_P(temp, sizeof(temp), PSTR("%s"), toReturn.c_str());
        strcat_P(plzIJson.data, temp);
        JSONCloseObj();*/
    } else if (strcmp(fileType, "JSON") == 0) {
        JSONAddArray(paramType);
        for (size_t i = 0; i < paramCount; i++) {
            JSONAddIntArrElem(((const byte*)params)[i]);
        }
        JSONCloseArray();
    }
}

#endif //USE_WEBSERVER

bool plzExecuteCmd(const char* cmd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd cmnd=%s"), cmd);

    char cmndType[4] = {0};
    char cmnd[12] = {0};

    const char* separator = strchr(cmd, ' ');
    if (!separator) {
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd no separator found"));
        return false;
    }

    strncpy(cmndType, cmd, separator - cmd);
    cmndType[3] = '\0';
    strncpy(cmnd, separator + 1, sizeof(cmnd) - 1);

    // Convertir en majuscules
    for (char &c : cmndType) c = toupper(c);
    for (char &c : cmnd) c = toupper(c);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd cmndType=%s"), cmndType);
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd cmnd=%s"), cmnd);

    initJSON();
    snprintf(plzIJson.cmd, sizeof(plzIJson.cmd), PSTR("%s"), cmd);
    return plzParser(cmndType, cmnd);
}

bool plzParser(const char* type, const char* cmnd) {
    char response[JSON_RESPONSE_SIZE];
    plzIJson.cmdRes = Palazzetti::CommandResult::COMMUNICATION_ERROR;
    bool success = false;
    char localCommand[16];
    strncpy(localCommand, cmnd, sizeof(localCommand) - 1);
    //localCommand[sizeof(localCommand) - 1] = '\0';

    if (isGetRequest || isUdpRequest) JSONAddObj("DATA");

    if (strcasecmp(type, "GET") == 0) {
        success = plzParserGET(localCommand);
    } else if (strcasecmp(type, "SET") == 0) {
        success = plzParserSET(localCommand);
    } else if (strcasecmp(type, "CMD") == 0) {
        success = plzParserCMD(localCommand);
    } else if (strcasecmp(type, "BKP") == 0) {
        success = plzParserBKP(localCommand);
    } else if (strcasecmp(type, "EXT") == 0) {
        success = plzParserEXT(localCommand);
    } else {
        plzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
        snprintf_P(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Unknown command received: %s"), type);
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande plzExecuteCmd commande non reconnue=%s"), type);
        //return false;
    }

    if (success) plz_connected = true;

    if (isGetRequest || isUdpRequest) {
        if (!success) {
            JSONAddBoolElem("NODATA", !success);
            if (plzIJson.msg == '\0') snprintf_P(plzIJson.msg, sizeof(plzIJson.msg), PSTR("Stove communication failed"));
        }
        JSONCloseObj();
    }
    snprintf(response, sizeof(response), "{%s}", plzIJson.data);
    if (isGetRequest || isUdpRequest) {
        JSONAddComma();
        JSONAddBoolElem("SUCCESS", success);
        JSONRemoveComma(plzIJson.data);
        plzIJson.ts = getTimeStamp();
        JSONMerge();

        if (strcasecmp(type, "BKP") == 0) {
            char attachment[100];
            snprintf_P(attachment, sizeof(attachment), PSTR("attachment; filename=%s.%s"), localCommand, localCommand + 5);
            Webserver->sendHeader(F("Content-Disposition"), attachment);
            
            const char* parmStart = nullptr;
            parmStart = strstr(plzIJson.data, "\"DATA\":{"); // Pour CSV
            if (parmStart != nullptr) {
                parmStart += 8;  // Ajuste le point de départ pour inclure seulement "PARM" et son contenu
                const char* parmEnd = strstr(parmStart, ",\"SUCCESS\"");

                if (parmEnd != nullptr) {
                    int startIndex = parmStart - plzIJson.data;
                    int endIndex = parmEnd - plzIJson.data;

                    if (strcmp(localCommand + 5, "CSV") == 0) {
                        WSSend(200, CT_PLAIN, String(plzIJson.data).substring(startIndex, endIndex - 1));
                    } else if (strcmp(localCommand + 5, "JSON") == 0) {
                        WSSend(200, CT_APP_JSON, String(plzIJson.data).substring(startIndex - 1, endIndex));
                    }
                } else {
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: BKP end data not found"));
                }
            } else {
                AddLog(LOG_LEVEL_INFO, PSTR("PLZ: BKP start data not found"));
            }
        } else {
            WSSend(200, CT_APP_JSON, plzIJson.full);
        }
        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: plzParser1 Commande response=%s"), plzIJson.full);
        isGetRequest = false;
        isUdpRequest = false;
    }

    const uint32_t mqttPrefix = TELE;
    char mqttTopic[128];
    if (PalazzettiSettings.mqtttopic == 1) {
        plzIJson.ts = getTimeStamp();
        JSONMerge();

        snprintf(mqttTopic, sizeof(mqttTopic), "%s", localCommand);
        Response_P(PSTR("%s"), plzIJson.data);
        MqttPublishPrefixTopicRulesProcess_P(mqttPrefix, mqttTopic);

    } else if (PalazzettiSettings.mqtttopic == 0 || PalazzettiSettings.mqtttopic == 2) {
        char *keyStart = strstr(plzIJson.data, "{");
        while (keyStart) {
            keyStart = strchr(keyStart, '"');
            if (!keyStart) break;

            char* keyEnd = strchr(keyStart + 1, '"');
            if (!keyEnd) break;
            *keyEnd = '\0';

            char *valueStart = strchr(keyEnd + 1, ':') + 1;
            while (*valueStart == ' ') valueStart++;
            char* valueEnd = *valueStart == '"' ? strchr(valueStart + 1, '"') : valueStart;
            if (!valueEnd) break;
            *valueEnd = '\0';

            snprintf(mqttTopic, sizeof(mqttTopic), PalazzettiSettings.mqtttopic == 2 ? "%s/%s" : "%s", localCommand, keyStart + 1);
            // Prépare le topic et publie
/*            if (PalazzettiSettings.mqtttopic == 0) {
                snprintf(mqttTopic, sizeof(mqttTopic), "%s", keyStart + 1);
            } else if (PalazzettiSettings.mqtttopic == 2) {
                snprintf(mqttTopic, sizeof(mqttTopic), "%s/%s", localCommand, keyStart + 1);
            }
               */ 
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Publishing MQTT: topic = %s, value = %s"), mqttTopic, valueStart);
            Response_P(PSTR("%s"), valueStart);
            MqttPublishPrefixTopicRulesProcess_P(mqttPrefix, mqttTopic);

            keyStart = valueEnd + 1;
        }
    } 
    return success;
}

bool plzParserGET(char* cmnd) {
    plzIJson.cmdRes = Palazzetti::CommandResult::PARSER_ERROR;
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd=%s"), cmnd);
    if (strcmp(cmnd, "ALLS") == 0 || strcmp(cmnd, "LALL") == 0 || strcmp(cmnd, "JALL") == 0) {
        getAllStatus();
    } else if (strcmp(cmnd, "TMPS") == 0) {
        getAllTemps();
    } else if (strcmp(cmnd, "IOPT") == 0) {
        getIO();
    } else if (strcmp(cmnd, "DPRS") == 0) {
        getDPressData();
    } else if (strcmp(cmnd, "FAND") == 0) {
        getFanData();
    } else if (strcmp(cmnd, "SETP") == 0) {
        getSetpoint();
    } else if (strcmp(cmnd, "STAT") == 0) {
        getStatus();
    } else if (strcmp(cmnd, "TIME") == 0) {
        getDateTime();
    } else if (strcmp(cmnd, "MDVE") == 0) {
        getModelVersion();
    } else if (strcmp(cmnd, "CHRD") == 0 || strcmp(cmnd, "JTMR") == 0) {
        getChronoData();
    //} else if (strcmp(cmnd, "CSET") == 0) {
    } else if (strcmp(cmnd, "STDT") == 0 || strcmp(cmnd, "LSTD") == 0) {
        getStaticData();
        strcpy(cmnd, "STDT");
    } else if (strcmp(cmnd, "SERN") == 0) {
        getSN();
    } else if (strcmp(cmnd, "LABL") == 0) {
        getLabel();
    } else if (strcmp(cmnd, "CUNT") == 0 || strcmp(cmnd, "CNTR") == 0) {
        getCounters();
        strcpy(cmnd, "CNTR");
    } else if (strcmp(cmnd, "POWR") == 0) {
        getPower();
    } else if (strncmp(cmnd, "PARM", 4) == 0) {
        const char* commandArg = cmnd + 5;
        if (*commandArg != '\0') {
            byte paramIndex = atoi(commandArg);
            getParameter(paramIndex, "PAR");
        }
    } else if (strncmp(cmnd, "HPAR", 4) == 0) {
        const char* commandArg = cmnd + 5;
        if (*commandArg != '\0') {
            byte paramIndex = atoi(commandArg);
            getHiddenParameter(paramIndex, "HPAR");
        }
    }
    return (plzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool plzParserSET(char* cmnd) {
    if (strncmp(cmnd, "RFAN", 4) == 0) {
        const char* fanLevel = cmnd + 5;
        setRoomFan(String(fanLevel));
        strcpy(cmnd, "FAND");
    } else if (strcmp(cmnd, "FN2D") == 0) {
        setRoomFanDown();
        strcpy(cmnd, "FAND");
    } else if (strcmp(cmnd, "FN2U") == 0) {
        setRoomFanUp();
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN3L", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setRoomFan3(String(commandArg));
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "FN4L", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setRoomFan4(String(commandArg));
        strcpy(cmnd, "FAND");
    } else if (strncmp(cmnd, "SLNT", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setSilentMode(String(commandArg));
    } else if (strncmp(cmnd, "POWR", 4) == 0) {
        const char* commandArg = cmnd + 5;
        byte powerLevel = atoi(commandArg);
        setPower(powerLevel);
    } else if (strcmp(cmnd, "PWRD") == 0) {
        setPowerDown();
        strcpy(cmnd, "POWR");
    } else if (strncmp(cmnd, "PWRU", 4) == 0) {
        setPowerUp();
        strcpy(cmnd, "POWR");
    } else if (strcmp(cmnd, "SETP") == 0) {
        const char* commandArg = cmnd + 5;
        setSetpoint(String(commandArg));
    } else if (strcmp(cmnd, "STPD") == 0) {
        setSetpointDown();
        strcpy(cmnd, "SETP");
    } else if (strcmp(cmnd, "STPU") == 0) {
        setSetpointUp();
        strcpy(cmnd, "SETP");
    } else if (strcmp(cmnd, "STPF") == 0) {
        const char* commandArg = cmnd + 5;
        setSetpointFloat(String(commandArg));
        strcpy(cmnd, "SETP");
    } else if (strncmp(cmnd, "TIME", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setDateTime(String(commandArg));
    } else if (strcmp(cmnd, "CPRD") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoPrg(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSTH", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setChronoStartHH(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strcmp(cmnd, "CSTM") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoStartMM(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strcmp(cmnd, "CSPH") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoStopHH(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strcmp(cmnd, "CSPM") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoStopMM(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strcmp(cmnd, "CDAY") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoDay(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strcmp(cmnd, "CSET") == 0) {
        const char* commandArg = cmnd + 5;
        setChronoSetpoint(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    } else if (strncmp(cmnd, "CSST", 4) == 0) {
        const char* commandArg = cmnd + 5;
        setChronoStatus(String(commandArg));
        strcpy(cmnd, "CHRSTATUS");
    //} else if (strncmp(cmnd, "CDYD", 4) == 0) {
    } else if (strcmp(cmnd, "PARM") == 0) {
        const char* commandArg = cmnd + 5;
        setParameter(String(commandArg), "PAR");
    } else if (strcmp(cmnd, "HPAR") == 0) {
        const char* commandArg = cmnd + 5;
        setHiddenParameter(String(commandArg), "HPAR");
    }
    //} else if (strcmp(cmnd, "LMAX") == 0) {
    //} else if (strcmp(cmnd, "LMIN") == 0) {
    return (plzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool plzParserCMD(char* cmnd) {
    if (strcmp(cmnd, "ON") == 0) {
        setSwitchOn();
        strcpy(cmnd, "STAT");
    } else if (strcmp(cmnd, "OFF") == 0) {
        setSwitchOff();
        strcpy(cmnd, "STAT");
    }
    return (plzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool plzParserBKP(char* cmnd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd BKP=%s"), cmnd);
    char* commandArg = cmnd + 5;
    if (strncmp(cmnd, "PARM", 4) == 0 && *commandArg != '\0') {
        getAllParameters(commandArg);
    } else if (strncmp(cmnd, "HPAR", 4) == 0 && *commandArg != '\0') {
        getAllHiddenParameters(commandArg);
    }
    return (plzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool plzParserEXT(char* cmnd) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Commande reçue cmnd EXT=%s"), cmnd);
    char* commandArg = cmnd + 5;
    byte paramIndex = atoi(commandArg);
    if (strncmp(cmnd, "PARL", 4) == 0 && *commandArg != '\0') {
        getParameter(paramIndex, "P");
    } else if (strncmp(cmnd, "HPRL", 4) == 0 && *commandArg != '\0') {
        getHiddenParameter(paramIndex, "HP");
    }
    return (plzIJson.cmdRes == Palazzetti::CommandResult::OK);
}

bool plzCmd(void) {
    char command[CMDSZ];
    char subCommand[CMDSZ];
    char *arg_part = NULL;
    char *cmd_part = NULL;

    bool serviced = false;
    uint8_t disp_len = strlen(D_NAME_PALAZZETTI);

    if (!strncasecmp_P(XdrvMailbox.topic, PSTR(D_NAME_PALAZZETTI), disp_len)) {

        if (XdrvMailbox.data_len > 0) {
            cmd_part = strtok_r(XdrvMailbox.data, " ", &arg_part);
            AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Cmd mqtt_part=%s"), cmd_part);
            //AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Cmd arg_part=%s"), arg_part);

            uint8_t command_code = GetCommandCode(command, sizeof(command), cmd_part, kPalazzetti_Commands);
            uint8_t newSetting = 255;

            //AddLog(LOG_LEVEL_INFO, PSTR("PLZ: What cmd code=%d cmd=%s arg=%s"), command_code, cmd_part, arg_part);
            switch (command_code) {
                case CMND_PALAZZETTI_SENDMSG:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Sendmsg cmd %s"), arg_part);
                    if (plz_connected && *arg_part != '\0') {
                        serviced = plzExecuteCmd(arg_part);
                    }
                    serviced = true;
                    break;
                case CMND_PALAZZETTI_INIT:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Init cmd"));
                    plzInit();
                    serviced = true;
                    break;
                case CMND_PALAZZETTI_INTERVAL:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Interval cmd=%s"), arg_part);
                    if (plzInterval > atoi(arg_part)) {
                        itoa(plzInterval, arg_part, 10);
                    }
                    PalazzettiSettings.interval = atoi(arg_part);
                    PlzSettingsSave();
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Timesync setting updated to %d"), newSetting);
                    Response_P(PSTR("{\"TIMESYNC_SETTING_UPDATED\":%d}"), newSetting);  // Réponse en console

                    serviced = true;
                    break;
                case CMND_PALAZZETTI_TIMESYNC:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Timesync cmd"));
                    if (strcmp(arg_part, "0") == 0 || strcmp(arg_part, "off") == 0) {
                        newSetting = 0;  // Désactiver
                    } else if (strcmp(arg_part, "1") == 0 || strcmp(arg_part, "on") == 0) {
                        newSetting = 1;  // Activer
                    }
                    
                    if (newSetting != 255) {
                        PalazzettiSettings.timesync = newSetting;
                        PlzSettingsSave();  // Sauvegarder les settings mis à jour
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Timesync setting updated to %d"), newSetting);
                        Response_P(PSTR("{\"TIMESYNC_SETTING_UPDATED\":%d}"), newSetting);  // Réponse en console
                    } else {
                        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Invalid timesync argument: %s"), arg_part);
                        Response_P(PSTR("{\"Error\":\"Invalid timesync argument\"}"));
                    }

                    serviced = true;
                    break;
                case CMND_PALAZZETTI_MQTT:
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: mqtt"));
                    
                    if (strcmp(arg_part, "0") == 0 || strcmp(arg_part, "raw") == 0) {
                        newSetting = 0;
                    } else if (strcmp(arg_part, "1") == 0 || strcmp(arg_part, "json") == 0) {
                        newSetting = 1;
                    } else if (strcmp(arg_part, "2") == 0 || strcmp(arg_part, "rawtopic") == 0) {
                        newSetting = 2;
                    }
                    
                    if (newSetting != 255) {
                        PalazzettiSettings.mqtttopic = newSetting;
                        PlzSettingsSave();
                        AddLog(LOG_LEVEL_INFO, PSTR("PLZ: MQTT setting updated to %d"), newSetting);
                        Response_P(PSTR("{\"MQTT_SETTING_UPDATED\":%d}"), newSetting);  // Réponse en console
                    } else {
                        AddLog(LOG_LEVEL_ERROR, PSTR("PLZ: Invalid MQTT argument: %s"), arg_part);
                        Response_P(PSTR("{\"Error\":\"Invalid MQTT argument\"}"));
                    }
                    serviced = true;
                    break;
                default:
                    plzDisplayCmdHelp();
                    break;
            }
        } else {
            plzDisplayCmdHelp();
        }
    }
    return serviced;
}

void plzDisplayCmdHelp(void) {
    AddLog(LOG_LEVEL_INFO, PSTR("Palazzetti commands :"));
    AddLog(LOG_LEVEL_INFO, PSTR("    sendmsg [GET/SET/CMD/BKP/EXT] [] []   Send commands to Palazzetti"));
    AddLog(LOG_LEVEL_INFO, PSTR("    init                                  Initialize serial connection"));
    AddLog(LOG_LEVEL_INFO, PSTR("    refresh interval=%d                    Refresh informations interval"), PalazzettiSettings.interval);
    AddLog(LOG_LEVEL_INFO, PSTR("    timesync=%d                       Sync Palazzetti time from Tasmota"), PalazzettiSettings.timesync);
    AddLog(LOG_LEVEL_INFO, PSTR("    mqtt=%d                             MQTT topic usage"), PalazzettiSettings.mqtttopic);
}

void PlzSettingsDefault(void) {
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: " D_USE_DEFAULTS));
    PalazzettiSettings.timesync = PLZ_SETTINGS_TIMESYNC;
    PalazzettiSettings.mqtttopic = PLZ_SETTINGS_MQTT_TOPIC;
    PalazzettiSettings.version = PLZ_SETTINGS_VERSION;
}

bool PlzSettingsRestore(void) {
    XdrvMailbox.data  = (char*)&PalazzettiSettings;
    XdrvMailbox.index = sizeof(PalazzettiSettings);
    return true;
}

void PlzSettingsLoad(bool erase) {
    // Called from FUNC_PRE_INIT once at restart
    // Called from FUNC_RESET_SETTINGS

    memset(&PalazzettiSettings, 0x00, sizeof(PalazzettiSettings));
    // Init default values in case file is not found
    PlzSettingsDefault();

    char filename[20];
    // Use for drivers:
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_DRIVER), XDRV_91);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: About to load settings from file %s"), filename);

#ifdef USE_UFILESYS
    if (erase) {
        // Use defaults
        TfsDeleteFile(filename);  
    }
    else if (TfsLoadFile(filename, (uint8_t*)&PalazzettiSettings, sizeof(PalazzettiSettings))) {
        // Fix possible setting deltas
        
    } else {
        // File system not ready: No flash space reserved for file system
        AddLog(LOG_LEVEL_INFO, PSTR(D_ERROR_FILE_NOT_FOUND));
    }
#else
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Error, file system not enabled"));
#endif  // USE_UFILESYS
}

void PlzSettingsSave(void) {
  // Called from FUNC_SAVE_SETTINGS every SaveData second and at restart
  uint32_t crc32 = GetCfgCrc32((uint8_t*)&PalazzettiSettings +4, sizeof(PalazzettiSettings) -4);  // Skip crc32
  if (crc32 != PalazzettiSettings.crc32 && PalazzettiSettings.version > 0) {
    PalazzettiSettings.crc32 = crc32;
    
    char filename[20];
    snprintf_P(filename, sizeof(filename), PSTR(TASM_FILE_DRIVER), XDRV_91);

    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: About to save settings to file %s"), filename);

#ifdef USE_UFILESYS
    if (!TfsSaveFile(filename, (const uint8_t*)&PalazzettiSettings, sizeof(PalazzettiSettings))) {
        // File system not ready: No flash space reserved for file system
        AddLog(LOG_LEVEL_INFO, D_ERROR_FILE_NOT_FOUND);
    }
#else
    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Error, file system not enabled"));
#endif  // USE_UFILESYS
    }
}

// Initialisation
bool Xdrv91(uint32_t function) {
    bool result = false;
    switch (function)
    {
        case FUNC_INIT:
            plzInit();  // Initialiser le contrôle
            break;
        case FUNC_SAVE_BEFORE_RESTART:
            if (plz_connected) { plzSaveBeforeRestart(); }
            break;
        case FUNC_RESTORE_SETTINGS:
            result = PlzSettingsRestore();
            break;
        case FUNC_RESET_SETTINGS:
            PlzSettingsLoad(1);
            break;
        case FUNC_SAVE_SETTINGS:
            PlzSettingsSave();
            break;
        case FUNC_PRE_INIT:
            PlzSettingsLoad(0);
            plzDrvInit();
            break; 
        case FUNC_EVERY_SECOND:
            if (plz_connected) {
                plzUpdate();
                PlzHandleUdpRequest();
            } else if (pala_rx_pin != NOT_A_PIN) {
                unsigned long currentTime = millis();
                if (currentTime - lastAttemptTime >= retryInterval) {
                    AddLog(LOG_LEVEL_INFO, PSTR("PLZ: Tentative de reconnexion..."));
                    plzInit();
                    lastAttemptTime = currentTime;
                }
            }
            break;
        case FUNC_COMMAND:
            result = plzCmd();
            break;
        case FUNC_JSON_APPEND:
            if (plz_connected) { plzShow(1); }
            break;
    #ifdef USE_WEBSERVER
        case FUNC_WEB_ADD_HANDLER:
            WebServer_on(PLZ_ENDPOINT, plzRequestHandler);
            break;
        case FUNC_WEB_GET_ARG:
            plzRequestHandler();
            break;
        case FUNC_WEB_SENSOR:
            if (plz_connected) { plzShow(0); }
            break;
    #endif  // USE_WEBSERVER
    }
    return result;
}
#endif //USE_PALAZZETTI