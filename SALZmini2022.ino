// SALZmini2022 bbd
/* for M5Stamp C3 using Wartering Unit
 * Arduino 1.8.19
 * [環境設定]-[追加のボードマネージャのURL]
 * https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_dev_index.json
 * [ESP32]-[ESP32C3 Dev Module]
 * Ambientのサンプルコードを参考にした。
 * https://github.com/AmbientDataInc/EnvSensorBleGw/blob/master/src/envSensor_esp32/BLE_BME280_bcast/BLE_BME280_bcast.ino
 * BLE_write.ino, BLE_notify.inoを参考にした。
 * https://github.com/electricbaka/bluejelly
 */

#include <Adafruit_NeoPixel.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

//#define TEST_MODE //水分計の戻り値をテストデータにする。

/*
 * コマンド:
 * WN   :今すぐ水やり
 * WL30 :水やりレベル設定（初期値:30）
 * YL40 :黄色レベル設定（初期値:40）
 * BL60 :青色レベル設定（初期値:60）
 * PT10 :ポンプ作動秒（初期値:10）
 * WT10 :待ち秒（初期値:10）
 */

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "fd531f24-3c63-482a-bfb1-2c493e6f2c07"
#define CHARACTERISTIC_UUID "e953e123-8e3c-4de9-a70a-264898b4ea90"

#define INPUT_PIN 0
#define PUMP_PIN  1
#define LED_PIN   2

#define SECONDS(s)    ((s) * 1000)
#define MINUTES(m)    SECONDS((m) * 60)


#define ADVARTISE_TIME  SECONDS(5)
#define NOTIFY_TIME     100
#define HISTORY_TIME    MINUTES(20)


enum {
    IN_WLGT,  // 水分量％を調べる時間になった
    IN_WTNW,  // 今すぐ水やりコマンド, 水やり
    IN_POFF,  // ポンプOFFタイマーの時間になった
    IN_CNCT,  // 接続中になった
    IN_DCNT,  // 接続が解除された
    IN_ADDS,  // Advartiseデータ送信の時間になった
    IN_NTDS,  // notifyデータ送信の時間になった
    IN_HDAD,  // 履歴データ追加の時間になった
    IN_NOOP,  // 何もない
    NINPUTS
};

enum {
    AC_WLGT,  // 水分量％を調べる。タイマーリセット
    AC_PPON,  // ポンプON。 ポンプOFFタイマーセット
    AC_NPTR,  // 何もしない。 ポンプOFFタイマーリセット
    AC_POFF,  // ポンプOFF
    AC_AVDS,  // Advartiseデータ送信。ADタイマーリセット
    AC_NTDS,  // notifyデータ送信。notifyタイマーリセット
    AC_HDAD,  // 履歴データ追加の時間になった
    AC_NOOP,  // 何もしない。
    NACTIONS
};

enum {
    ST_WAIT,  // 待機
    ST_WTNG,  // 水やり中
    ST_CTWT,  // 接続中で待機
    ST_CTWG,  // 接続中で水やり中
    NSTATUS
};

static int action[NSTATUS][NINPUTS] = {
    { AC_WLGT, AC_PPON, AC_NOOP, AC_NOOP, AC_NOOP, AC_AVDS, AC_NTDS, AC_HDAD, AC_NOOP },
    { AC_WLGT, AC_NPTR, AC_POFF, AC_NOOP, AC_NOOP, AC_AVDS, AC_NTDS, AC_HDAD, AC_NOOP },
    { AC_WLGT, AC_PPON, AC_NOOP, AC_NOOP, AC_NOOP, AC_AVDS, AC_NTDS, AC_HDAD, AC_NOOP },
    { AC_WLGT, AC_NPTR, AC_POFF, AC_NOOP, AC_NOOP, AC_AVDS, AC_NTDS, AC_HDAD, AC_NOOP }
};

static int next_status[NSTATUS][NINPUTS] = {
    { ST_WAIT, ST_WTNG, ST_WAIT, ST_CTWT, ST_WAIT, ST_WAIT, ST_WAIT, ST_WAIT, ST_WAIT },
    { ST_WTNG, ST_WTNG, ST_WAIT, ST_CTWG, ST_WTNG, ST_WTNG, ST_WTNG, ST_WTNG, ST_WTNG },
    { ST_CTWT, ST_CTWG, ST_CTWT, ST_CTWT, ST_WAIT, ST_CTWT, ST_CTWT, ST_CTWT, ST_CTWT },
    { ST_CTWG, ST_CTWG, ST_CTWT, ST_CTWG, ST_WTNG, ST_CTWG, ST_CTWG, ST_CTWG, ST_CTWG }
};

bool deviceConnected = false;
bool prevDeviceConnected = false;

#define NCOLORS 5
static int colorRs[NCOLORS] = { 0x60, 0x50, 0x30, 0x00, 0x00 };
static int colorGs[NCOLORS] = { 0x00, 0x10, 0x30, 0x50, 0x00 };
static int colorBs[NCOLORS] = { 0x00, 0x00, 0x00, 0x10, 0x60 };
static int delays[NCOLORS] = { 4, 6, 8, 10, 12 };

//装置の状態
enum {
  stError,
  stWatering,
  stWaitRed,
  stWaitYellow,
  stWaitBlue
};

typedef struct {
  uint16_t count;             // 2:(0):カウント
  uint16_t wCount;            // 2:(2):水やりカウント
  uint8_t currentWaterLevel;  // 1:(4):現在の水分量%
  uint8_t status;             // 1:(5):状態
  uint8_t wateringLevel;      // 1:(6):水やりレベル
  uint8_t yellowLevel;        // 1:(7):黄色レベル
  uint8_t blueLevel;          // 1:(8):青色レベル
  uint8_t pumpTime;           // 1:(9):ポンプ作動秒
  uint8_t waitTime;           // 1:(10):待ち秒
} SALZData;

typedef struct {
  uint32_t millis;     //4
  uint8_t  waterLevel; //1
} HistoryData;

#define MAX_HD  200
HistoryData historyData[MAX_HD];

int lHd = 0;
int tHd = 0;
int bHd = 0;

typedef struct {
  SALZData sd;
  uint8_t iHd;
  uint8_t nHd;
  HistoryData hd;
  uint32_t nowMillis;
} SALZNotifyData;

/*
typedef struct {
  uint8_t a[28];
} SNDDecoy;
*/

static SALZNotifyData pSnd;


Adafruit_NeoPixel pixels(1, LED_PIN, NEO_GRB + NEO_KHZ800);

BLEServer         *pServer = NULL;
BLEService        *pService = NULL;
BLECharacteristic *pCharacteristic = NULL;
BLEAdvertising    *pAdvertising = NULL;

SALZData salzData;

unsigned long pumpTimeMillis = 0L;
unsigned long waitTimeMillis = 0L;
unsigned long advartiseTimeMillis = 0L;
unsigned long notifyTimeMillis = 0L;
unsigned long historyTimeMillis = 0L;

bool bForceWater = false;
bool bPump = false;


class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


class MyCallbacks1: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    Serial.println("pCharacteristic->getValue();");

    if (value.length() > 0) {
      Serial.printf("Command:%s\n", value.c_str());
      std::string sCommand = value.substr(0, 2);
      int number = (value.length() > 2)
          ? std::stoi(value.substr(2)) : 0;

      if (sCommand == "WN") {
        Serial.printf("WN\n");
        bForceWater = true;
      }
      else if (sCommand == "WL") {
        Serial.printf("WL%d\n", number);
        salzData.wateringLevel = number;
      }
      else if (sCommand == "YL") {
        Serial.printf("YL%d\n", number);
        salzData.yellowLevel = number;
      }
      else if (sCommand == "BL") {
        Serial.printf("BL%d\n", number);
        salzData.blueLevel = number;
      }
      else if (sCommand == "PT") {
        Serial.printf("PT%d\n", number);
        salzData.pumpTime = number;
      }
      else if (sCommand == "WT") {
        Serial.printf("WT%d\n", number);
        salzData.waitTime = number;
      }
    }
  }
};


#ifdef TEST_MODE
int
GetCurrentWaterLevel(void) {
// テスト用 10～90の値を返す。
  static float th = 0.0;
  th += 2.0 * PI / 36.0;
  if (th > 2.0 * PI)
    th = 0.0;

  return cos(th) * 40.0 + 50.0;
}


#else
int
GetWl(void) {
  float val = 0.0;
  pinMode(INPUT_PIN, INPUT);
  for (int i = 0; i < 10; i++) {
    float v = analogRead(INPUT_PIN);
    //Serial.printf("ML(%d):%8.3f\r\n", i, v);
    val += v;
    delay(10);
  }
  return (int)(val / 10.0);
}


int
GetCurrentWaterLevel(void) {
  //校正済の水分％を返す。
  //センサーを水に浸けた状態：1900
  //センサーを外に出した状態：2500
  const int wetWl = 1900;
  const int dryWl = 2500;
  int ret = (int)((1.0 - ((float)(GetWl() - wetWl) / (float)(dryWl - wetWl))) * 100.0);

  return (ret < 0) ? 0 : ((ret > 99) ? 99 : ret);
}
#endif


void
SetAdvartiseData(void) {
  // AdvartiseDataを設定する。送信は一定時間ごとに行われる。
  Serial.printf("\nSetData():wl:%d st:%d c:%d wC:%d\n",
      salzData.currentWaterLevel,
      salzData.status,
      salzData.count,
      salzData.wCount);

  BLEAdvertisementData oAdvertisementData = BLEAdvertisementData();

  oAdvertisementData.setName("SM2022");
  oAdvertisementData.setFlags(0x06); // BR_EDR_NOT_SUPPORTED | LE General Discoverable Mode

  std::string strServiceData = "";
  strServiceData += (char)0xff;   // AD Type 0xFF: Manufacturer specific data
  strServiceData += (char)0xff;   // Test manufacture ID low byte
  strServiceData += (char)0xff;   // Test manufacture ID high byte
  strServiceData += (char)salzData.currentWaterLevel;      // 水分量%
  strServiceData += (char)salzData.status;                 // 状態
  strServiceData += (char)(salzData.count & 0xff);         // カウントの下位バイト
  strServiceData += (char)((salzData.count >> 8) & 0xff);  // カウントの上位バイト
  strServiceData += (char)(salzData.wCount & 0xff);        // 水やりカウントの下位バイト
  strServiceData += (char)((salzData.wCount >> 8) & 0xff); // 水やりカウントの上位バイト
  strServiceData = (char)strServiceData.length() + strServiceData;
  oAdvertisementData.addData(strServiceData);

  pAdvertising->setAdvertisementData(oAdvertisementData);

  salzData.count = (++salzData.count < ULONG_MAX) ? salzData.count : 0;
}


int
GetDispLevel(void) {
  int ret;
  ret = (int)(((float)salzData.currentWaterLevel / 100.0) * (float)NCOLORS);
  ret = (ret > NCOLORS - 1) ? (NCOLORS - 1) : ret;
  return ret;
}

//---------------------------------------

void
task1(void * pvParameters) {
  int a = 0;
  int rPrev = 0;
  int gPrev = 0;
  int bPrev = 0;
  while (1) {
    int lv = GetDispLevel();
    a = (a < 360) ? ++a : 0;
    float v = (sin(PI * (float)a / 180.0) + 1.0) / 2.0;
    int r = (float)colorRs[lv] * v;
    int g = (float)colorGs[lv] * v;
    int b = (float)colorBs[lv] * v;

    if (r != rPrev || g != gPrev || b != bPrev) {
      pixels.setPixelColor(0, pixels.Color(r, g, b));    //Colorメソッド内の引数の順番は赤、緑、青
      pixels.show();
    }
    delay(delays[lv]);

    rPrev = r;
    gPrev = g;
    bPrev = b;
  }
}


unsigned long
TimeDiff(unsigned long t1, unsigned long t2) {
  if (t1 - t2 < 0)
    return t1 + (ULONG_MAX - t2);
  else
    return t1 - t2;
}


void
SetHistoryData(void)
{
  if (++bHd > MAX_HD - 1)
    bHd = 0;
  if (++lHd > MAX_HD - 1) {
    lHd = MAX_HD;
    if (++tHd > MAX_HD - 1) {
      tHd = 0;
    }
  }
  historyData[bHd].waterLevel = salzData.currentWaterLevel;
  historyData[bHd].millis = millis();
  Serial.printf("SetHistoryData[l:%d t:%d b:%d](%lu, %d)",
      lHd, tHd, bHd,
      historyData[bHd].millis,
      historyData[bHd].waterLevel
  );
}


void
NotifyData(void)
{
  SALZNotifyData snd;

  snd.sd = salzData;
  snd.nHd = lHd;
  if (snd.nHd > 0) {
    snd.iHd = (pSnd.iHd + 1) % lHd;
    snd.hd = historyData[(snd.iHd + tHd) % MAX_HD];
  }
  else {
    snd.iHd = -1;
    snd.hd.millis = 0L;
    snd.hd.waterLevel = 0;
  }
  snd.nowMillis = millis();

  pCharacteristic->setValue((uint8_t*)&snd, sizeof(SALZNotifyData));
/*
  SNDDecoy sd;
  for (int i = 0; i < 28; i++)
    sd.a[i] = i;
  pCharacteristic->setValue((uint8_t*)&sd, sizeof(SNDDecoy));
  Serial.printf("sizeof(SALZNotifyData):%d\n", sizeof(SALZNotifyData));
*/
  pCharacteristic->notify();
  pSnd = snd;
}


int
GetInput(void) {
  int input = IN_NOOP;

  if (prevDeviceConnected == false && deviceConnected == true) {
    Serial.print("C");
    prevDeviceConnected = deviceConnected;
    input = IN_CNCT;
  }
  else if (prevDeviceConnected == true && deviceConnected == false) {
    Serial.print("D");
    delay(500); // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    prevDeviceConnected = deviceConnected;
    input = IN_DCNT;
  }
  else if (TimeDiff(millis(), waitTimeMillis) > SECONDS(salzData.waitTime)) {
    Serial.print("w");
    input = IN_WLGT;
  }
  else if (TimeDiff(millis(), advartiseTimeMillis) > ADVARTISE_TIME) {
    Serial.print("A");
    input = IN_ADDS;
  }
  else if (TimeDiff(millis(), historyTimeMillis) > HISTORY_TIME) {
    Serial.print("h");
    input = IN_HDAD;
  }
  else if (deviceConnected == true && TimeDiff(millis(), notifyTimeMillis) > NOTIFY_TIME) {
    Serial.print("N");
    input = IN_NTDS;
  }
  else if (bForceWater || salzData.currentWaterLevel < salzData.wateringLevel) {
    Serial.print("W");
    bForceWater = false;
    input = IN_WTNW;
  }
  else if (bPump && TimeDiff(millis(), pumpTimeMillis) > SECONDS(salzData.pumpTime)) {
    Serial.print("P");
    input = IN_POFF;
  }
  else {
    //Serial.print("n");
    input = IN_NOOP;
  }

  // その他の処理
  if (salzData.currentWaterLevel < salzData.yellowLevel) {
    salzData.status = stWaitRed;
  }
  else if (salzData.currentWaterLevel < salzData.blueLevel) {
    salzData.status = stWaitYellow;
  }
  else {
    salzData.status = stWaitBlue;
  }

  return input;
}


void
setup(void) {
  memset(&salzData, 0, sizeof(SALZData));
  salzData.count = 0;
  salzData.wCount = 0;
  salzData.currentWaterLevel = 0;
  salzData.status = stError;
  salzData.wateringLevel = 30;
  salzData.yellowLevel = 40;
  salzData.blueLevel = 60;
  salzData.pumpTime = 10;
  salzData.waitTime = 10;

  pSnd.sd = salzData;
  pSnd.iHd = -1;
  pSnd.nHd = 0;
  pSnd.hd.waterLevel = 0;
  pSnd.hd.millis = 0L;
  pSnd.nowMillis = 0L;

  Serial.begin(115200);
  Serial.print("SALZmini2022\n");

  pixels.begin();

  pinMode(INPUT_PIN, INPUT);
  pinMode(PUMP_PIN, OUTPUT);

  // Initialize the BLE environment
  BLEDevice::init("SALZmini2022");

  // Create the server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the service
  pService = pServer->createService(SERVICE_UUID);

  // Create the characteristic 1
  pCharacteristic = pService->createCharacteristic(
                     CHARACTERISTIC_UUID,
                     BLECharacteristic::PROPERTY_READ |
                     BLECharacteristic::PROPERTY_WRITE |
                     BLECharacteristic::PROPERTY_NOTIFY |
                     BLECharacteristic::PROPERTY_INDICATE
                  );
  pCharacteristic->setCallbacks(new MyCallbacks1());

  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  // Set the characteristic value
  //pCharacteristic->setValue("Hello World");

  // Start the service
  pService->start();

  pAdvertising = pServer->getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");

  // Task 1
  xTaskCreatePinnedToCore(
                  task1,     /* Function to implement the task */
                  "task1",   /* Name of the task */
                  4096,      /* Stack size in words */
                  NULL,      /* Task input parameter */
                  1,         /* Priority of the task */
                  NULL,      /* Task handle. */
                  0);        /* Core where the task should run */

  salzData.currentWaterLevel = GetCurrentWaterLevel();
  SetAdvartiseData();
  SetHistoryData();
}


void
loop(void) {
    static int status = ST_WAIT;
    int input = GetInput(); // 入力を決定します。
    switch (action[status][input]) { // status と input から次の action を決定します。
    case AC_WLGT:  // 水分量％を調べる。タイマーリセット
        //Serial.println("\nAC_WLGT");
        // 水分量%を取得する。
        salzData.currentWaterLevel = GetCurrentWaterLevel();
        waitTimeMillis = millis();
        break;
    case AC_PPON:  // ポンプON。 ポンプOFFタイマーセット
        //Serial.println("\nAC_PPON");
        digitalWrite(PUMP_PIN, true);
        salzData.wCount = (++salzData.wCount < ULONG_MAX) ? salzData.wCount : 0;
        bPump = true;
        pumpTimeMillis = millis();
        break;
    case AC_NPTR:  // 何もしない。 ポンプOFFタイマーリセット
        //Serial.println("\nAC_NPTR");
        bPump = true;
        pumpTimeMillis = millis();
        break;
    case AC_POFF:  // ポンプOFF
        //Serial.println("\nAC_POFF");
        digitalWrite(PUMP_PIN, false);
        bPump = false;
        break;
    case AC_AVDS:  // Advartiseデータ設定。ADタイマーリセット
        //Serial.println("\nAC_AVDS");
        SetAdvartiseData();
        advartiseTimeMillis = millis();
        break;
    case AC_NTDS:  // notifyデータ送信。notifyタイマーリセット
        //Serial.println("\nAC_NTDS");
        NotifyData();
        notifyTimeMillis = millis();
        break;
    case AC_HDAD:  // 履歴データ追加。履歴タイマーリセット
        //Serial.println("\nAC_HDAD");
        SetHistoryData();
        historyTimeMillis = millis();
        break;
    case AC_NOOP:  // 何もしない。
        break;
    }
    status = next_status[status][input]; // status と input から次の stat を決定します。
}
