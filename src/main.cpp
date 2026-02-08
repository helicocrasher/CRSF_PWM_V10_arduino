#include <arduino.h>
#include <Wire.h>

#include <SPI.h>

#include <NullSerial.h>     // NullSerial for disabling debug output
#include <AlfredoCRSF.h>

#include "hardware.h"       //required for Hardware stuff

AlfredoCRSF crsf;
TwoWire BaroWire(SDA1, SCL1);	// I2C bus for Barometer sensor SDA1 & SCL1 defined in hardware.h

void setupBaroSensor();
void setup_HW_OUT_PWM(void);
void setPWMChannels();
static void telemetrySendCellVoltage(uint8_t cellId, float voltage);
void telemetrySendBaroAltitude(float altitude, float verticalspd);
void baroProcessingTask(uint32_t millis_now);
void baroSerialDisplayTask(uint32_t millis_now);
void getI2cData(uint8_t I2C_Address,uint8_t device_register  , uint8_t num_bytes,short * indata);
void printChannels();

#define STRING_BUFFER_SIZE 120
char stringBuffer[STRING_BUFFER_SIZE]={0};

#if !defined(STM32_CORE_VERSION) || (STM32_CORE_VERSION  < 0x01090000)
#error "Due to API change, this sketch is compatible with STM32_CORE_VERSION  >= 0x01090000"
#endif

#ifdef TARGET_BLUEPILL        // and BMP280  sensor
  #include <Adafruit_BMP280.h>
  Adafruit_BMP280 BaroSensor(&BaroWire); // 
  #define SerialDebug Serial2
  #define SerialI2CDebug SerialDebug

  #define GNSSSerialDebug SerialDebug
  #define gnssSerial Serial3
  #define GNSS_SERIAL Serial3
  #define GNSS_SERIAL_TX_PIN SERIAL3_TX  // USART3 TX
  #define GNSS_SERIAL_RX_PIN SERIAL3_RX  // USART3 RX
  uint8_t BaudRateIndex=0;
  #define NR_OF_GNSS_baudrates 5
  const uint32_t possibleBauds[NR_OF_GNSS_baudrates] = { 115200, 57600, 38400, 19200, 9600 };
  #include "SparkFun_u-blox_GNSS_Arduino_Library.h" 
  SFE_UBLOX_GNSS myGNSS;
  bool autoBaudGNSS(void);
  void sendGps_int(int32_t latitude, int32_t longitude, int32_t groundspeed, int32_t heading, int32_t altitude, uint8_t satellites);
  void printGNSS(void);
  bool GNSS_available=false;
  #define mmsTokmh 278          // conversion factor from mm/s to km/h
  #define NR_OF_PWM_OUT 8
#endif  // TARGET_BLUEPILL and BMP280 sensor

#ifdef TARGET_MATEK_CRSF_PWM_V10        // TARGET_MATEK_CRSF_PWM_V10 with SPL06-001
  #include <SPL06-001.h>
  #define SerialDebug Serial2
  #define SerialI2CDebug SerialDebug
  SPL06 BaroSensor(&BaroWire);   // 
  #define NR_OF_PWM_OUT 10
#endif // end TARGET_MATEK_CRSF_PWM_V10 with SPL06-001 

#define PWM_PERIOD_US 20000   // 20ms period for 50Hz
#define crsfSerial Serial1
#define CRSF_SERIAL_TX_PIN SERIAL1_TX
#define CRSF_SERIAL_RX_PIN SERIAL1_RX

uint32_t Tim_out_channel[NR_OF_PWM_OUT];
HardwareTimer *My_PWM_Out_Tim[NR_OF_PWM_OUT];

void setup() {
  SerialDebug.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH^LED_ACTIVE_LOW); // turn on built in LED to indicate setup is running
  #ifdef TARGET_BLUEPILL
//  while (!Serial) ; // wait for serial port to connect. Needed for native USB
  #endif
  SerialI2CDebug.println("Setup Baro Sensor starting");
  delay(100);    
  setupBaroSensor();
  delay(100);
  crsfSerial.setTx(CRSF_SERIAL_TX_PIN);
  crsfSerial.setRx(CRSF_SERIAL_RX_PIN); 
  delay(100);
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  crsf.begin(crsfSerial);
  delay(100);
  setup_HW_OUT_PWM();
  #ifdef TARGET_BLUEPILL
  GNSS_available = autoBaudGNSS();
  if (GNSS_available) {
    myGNSS.setMeasurementRate(250);     // Set the GNSS module to 1 second (1000 ms) measurement rate
    myGNSS.saveConfiguration();
    myGNSS.setAutoPVT(true);            // Enable automatic PVT data messages
//    myGNSS.enableDebugging();           // Uncomment this line to enable helpful debug messages on Serial
    myGNSS.disableDebugging();           // comment this line to enable helpful debug messages on Serial
    myGNSS.setUART1Output(COM_TYPE_UBX); //Set the UART1 port to output UBX only (turn off NMEA noise)
  }
  #endif
  SerialDebug.print("F_CPU = ");
  SerialDebug.print(F_CPU);
  SerialDebug.print("   HSE_VALUE = ");
  SerialDebug.println(HSE_VALUE);
  SerialDebug.println("Setup done");
//  loop_counter=0;
//  micros_last=millis();
}

#define IIR_ALPHA   0.135755f  // 0.5Hz cut off (fc/40 / fc=Hz /Tc=320ms)
#define IIR_BETA   (1.0f - IIR_ALPHA)

static float GND_altitude=0;
static float previous_alt_ASL=0;
static float vario=0,filt_alt_AGL=0;
static double filt_vario=0, filt_alt_ASL=0;
static uint32_t GND_alt_count=0;

void loop() {
static uint32_t millis_now=0, millis_last=0,main_loop_counter=0,crsf_last_update=0;

  millis_now = millis();
  if(millis_now - millis_last >= 250){  // every 250ms
    millis_last = millis_now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    printChannels();
    telemetrySendCellVoltage(1, 4.11);
    telemetrySendBaroAltitude(filt_alt_AGL, filt_vario);
  #ifdef TARGET_BLUEPILL
    if(GNSS_available) {
      sendGps_int(myGNSS.getLatitude(), myGNSS.getLongitude(), myGNSS.getGroundSpeed()/mmsTokmh, myGNSS.getHeading(), myGNSS.getAltitudeMSL()/1000, myGNSS.getSIV());
      printGNSS();
    }
  }
  if(GNSS_available) myGNSS.getPVT();
#else
  }  
#endif
  baroProcessingTask(millis_now);
  baroSerialDisplayTask(millis_now);
  crsf.update();
  setPWMChannels();
  delay(1);  // just to avoid a tight loop, not really needed
  main_loop_counter++;
}

void setPWMChannels() {
  for (int PWM_channelIndex = 0; PWM_channelIndex < NR_OF_PWM_OUT; PWM_channelIndex++)
  {
    My_PWM_Out_Tim[PWM_channelIndex]->setCaptureCompare(Tim_out_channel[PWM_channelIndex], crsf.getChannel(PWM_channelIndex+1), TICK_COMPARE_FORMAT);
  }
}

void printChannels() {
  for (int ChannelNum = 1; ChannelNum <= NR_OF_PWM_OUT; ChannelNum++)
  {
    SerialDebug.print(crsf.getChannel(ChannelNum));
    SerialDebug.print(", ");
  }
  SerialDebug.println(" ");
}

void baroSerialDisplayTask(uint32_t millis_now){
  static uint32_t last_millis=0;
  if (millis_now - last_millis < 250) return;
  last_millis = millis_now;
  SerialI2CDebug.print(F("Temperature = "));
  SerialI2CDebug.print(BaroSensor.readTemperature());
  SerialI2CDebug.println(" *C");
  SerialI2CDebug.print(F("Pressure = "));
  SerialI2CDebug.print(BaroSensor.readPressure());
  SerialI2CDebug.println(" Pa");
  SerialI2CDebug.print(F("Approx altitude ASL = "));    
  SerialI2CDebug.print(filt_alt_ASL);
  SerialI2CDebug.print(" m ; Altitude AGL = ");
  SerialI2CDebug.print(filt_alt_AGL);
  SerialI2CDebug.println(" m");
  SerialI2CDebug.print("Vario = ");
  SerialI2CDebug.print(filt_vario);
  SerialI2CDebug.println(" m/s");
  SerialI2CDebug.println("\n ");  
}


void baroProcessingTask(uint32_t millis_now){
  float altitude;
  static uint32_t last_millis=0,loop_counter=0;
  if (millis_now - last_millis < 100) return;
  last_millis = millis_now;

  #ifdef TARGET_BLUEPILL  // BMP280 sensor
    altitude = BaroSensor.readAltitude(1013.25); /* Adjusted to local forecast! */
  #else                   // SPL06-001 sensor
    altitude = BaroSensor.readPressureAltitudeMeter(1013.25);
  #endif
  if (loop_counter <1000) {    // initialisation --> average ground altitude
    GND_altitude += altitude;
    GND_alt_count++;
    filt_alt_ASL = altitude;
  } else {
    filt_alt_ASL = (IIR_ALPHA * altitude) + (IIR_BETA * filt_alt_ASL);
  }
  filt_alt_AGL =filt_alt_ASL - (GND_altitude / GND_alt_count);
  vario=(filt_alt_AGL-previous_alt_ASL)*20;
  filt_vario = (IIR_ALPHA * vario) + (IIR_BETA * filt_vario);
  previous_alt_ASL = filt_alt_AGL;
  loop_counter++;
}

/*
void calc_baro(uint32_t millis_now){
  static uint32_t last_millis=0;
  if (millis_now - last_millis < 50) return;
  last_millis = millis_now;
}

void bla(uint32_t millis_now){
  static uint32_t last_millis=0;
  if (millis_now - last_millis < 50) return;
  last_millis = millis_now;
}
*/

#define CRSF_BATTERY_SENSOR_CELLS_MAX 12

static void telemetrySendCellVoltage(uint8_t cellId, float voltage) {
  if (cellId < 1 || cellId > CRSF_BATTERY_SENSOR_CELLS_MAX)     return;

  uint8_t payload[3];
  payload[0] = cellId;
  uint16_t voltage_be = htobe16((uint16_t)(voltage * 1000.0)); //mV
  memcpy(&payload[1], &voltage_be, sizeof(voltage_be));
  crsf.queuePacket(CRSF_SYNC_BYTE, 0x0e, payload, sizeof(payload));
}

void telemetrySendBaroAltitude(float altitude, float verticalspd)
{
  crsf_sensor_baro_altitude_t crsfBaroAltitude = { 0 };

  // Values are MSB first (BigEndian)
  crsfBaroAltitude.altitude = htobe16((uint16_t)(altitude*10.0 + 10000.0));
  //crsfBaroAltitude.verticalspd = htobe16((int16_t)(verticalspd*100.0)); //TODO: fix verticalspd in BaroAlt packets
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_BARO_ALTITUDE, &crsfBaroAltitude, sizeof(crsfBaroAltitude) - 2);
  
  //Supposedly vertical speed can be sent in a BaroAltitude packet, but I cant get this to work.
  //For now I have to send a second vario packet to get vertical speed telemetry to my TX.
  crsf_sensor_vario_t crsfVario = { 0 };

  // Values are MSB first (BigEndian)
  crsfVario.verticalspd = htobe16((int16_t)(verticalspd*100.0));
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_VARIO, &crsfVario, sizeof(crsfVario));
}


void setup_HW_OUT_PWM(void)
{
  // no need to configure pin, it will be done by HardwareTimer configuration

  uint32_t PrescalerFactor = TIMER_PRESCALER;

  // Automatically retrieve TIM instance and channel associated to pin
  // This is used to be compatible with all STM32 series automatically.
  for (int PWM_ChannelIndex=0; PWM_ChannelIndex<NR_OF_PWM_OUT;PWM_ChannelIndex++){
    TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(PWM_Tim_Pin_Map[PWM_ChannelIndex]), PinMap_PWM);
    SerialDebug.println();
    SerialDebug.print("Configuring PWM output on pin: ");
    SerialDebug.println(digitalPinToPinName(PWM_Tim_Pin_Map[PWM_ChannelIndex]),HEX);
    //Instantiate HardwareTimer object. Thanks to 'new' instantiation, HardwareTimer is not destructed when setup() function is finished.
    My_PWM_Out_Tim[PWM_ChannelIndex] = new HardwareTimer(Instance);
    sprintf(stringBuffer," %08X", (void *)My_PWM_Out_Tim[PWM_ChannelIndex]);
    SerialDebug.println(stringBuffer);

    My_PWM_Out_Tim[PWM_ChannelIndex]->setPrescaleFactor(PrescalerFactor);
    My_PWM_Out_Tim[PWM_ChannelIndex]->setOverflow(PWM_PERIOD_US,TICK_FORMAT);   // Set period in microseconds
    SerialDebug.print("timer for "); SerialDebug.print(PWM_Tim_Pin_Map[PWM_ChannelIndex],HEX); SerialDebug.println(" configured");

  
    Tim_out_channel[PWM_ChannelIndex] = STM_PIN_CHANNEL(pinmap_function(digitalPinToPinName(PWM_Tim_Pin_Map[PWM_ChannelIndex]), PinMap_PWM));
    sprintf(stringBuffer,"PWM_ChannelIndex = %02d, Tim_Out_Channel = %02d, Pin_Map= %04X", PWM_ChannelIndex, Tim_out_channel[PWM_ChannelIndex],PWM_Tim_Pin_Map[PWM_ChannelIndex]);
    SerialDebug.println(stringBuffer);

    My_PWM_Out_Tim[PWM_ChannelIndex]->setMode(Tim_out_channel[PWM_ChannelIndex], TIMER_OUTPUT_COMPARE_PWM1, PWM_Tim_Pin_Map[PWM_ChannelIndex]);
    My_PWM_Out_Tim[PWM_ChannelIndex]->setCaptureCompare(Tim_out_channel[PWM_ChannelIndex], (uint32_t) (1500+PWM_ChannelIndex*100), TICK_COMPARE_FORMAT);
  
    My_PWM_Out_Tim[PWM_ChannelIndex]->resume();
    sprintf(stringBuffer,"timer channel %d started",PWM_ChannelIndex);
    SerialDebug.println(stringBuffer);
  }
}

void getI2cData(uint8_t I2C_Address,uint8_t device_register  , uint8_t num_bytes,short * indata) {
  uint8_t bytes_read;
  Wire.begin();  
  Wire.setClock(100000);
  delay(100);
  Wire.beginTransmission(I2C_Address);
  Wire.write(device_register);
  Wire.endTransmission();
  bytes_read = Wire.requestFrom(I2C_Address,num_bytes);
  *indata = Wire.read();
  Wire.endTransmission();
  SerialI2CDebug.print("Bytes read = ");
  SerialI2CDebug.print(bytes_read);
  SerialI2CDebug.print(" SensorID = ");
  SerialI2CDebug.println(*indata,HEX);
}

#ifdef TARGET_BLUEPILL  // BMP280 sensor
//#define BMP280_ADDRESS_ALT 0x76
//#define BMP280_CHIPID 0x58

void setupBaroSensor(){   // BMP280 sensor version
  SerialI2CDebug.println(F("BMP280 test"));
  unsigned status;
  BaroWire.begin();
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = BaroSensor.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    SerialI2CDebug.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    SerialI2CDebug.print("SensorID was: 0x"); SerialI2CDebug.println(BaroSensor.sensorID(),16);
    SerialI2CDebug.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    SerialI2CDebug.print("        ID of 0x56-0x58 represents a BMP 280,\n");
    SerialI2CDebug.print("        ID of 0x60 represents a BME 280.\n");
    SerialI2CDebug.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  BaroSensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
}


bool autoBaudGNSS(void)
{ 
  bool GNSS_Connected = false;

  GNSSSerialDebug.print(("0 : Attempting to connect to GNSS module with "));
  GNSSSerialDebug.println(possibleBauds[BaudRateIndex]);
  gnssSerial.setTx(GNSS_SERIAL_TX_PIN);
  gnssSerial.setRx(GNSS_SERIAL_RX_PIN);
  gnssSerial.begin(possibleBauds[BaudRateIndex]);
  GNSS_Connected = myGNSS.begin(gnssSerial);
  while ((BaudRateIndex < 4) && ! GNSS_Connected) //Connect to the u-blox module using gnssSerial (defined above)
  {
    delay (5);
    BaudRateIndex++;
    BaudRateIndex = BaudRateIndex % 5;    
    GNSSSerialDebug.print(BaudRateIndex);
    GNSSSerialDebug.print((" : Attempting to connect to GNSS module with "));
    GNSSSerialDebug.println(possibleBauds[BaudRateIndex %5]);
    gnssSerial.begin(possibleBauds[BaudRateIndex%5]);
    GNSS_Connected=myGNSS.begin(gnssSerial);
  }
  if (GNSS_Connected){
    GNSSSerialDebug.print(("Connected to GNSS module at "));
    GNSSSerialDebug.print(possibleBauds[BaudRateIndex%5]);
    GNSSSerialDebug.println((" baud."));
    if (BaudRateIndex != 0){
      BaudRateIndex = 0; // We want to set back to 115200
      myGNSS.setSerialRate(possibleBauds[BaudRateIndex]); // Set u-blox module to 115200 baud for reconnect
      gnssSerial.begin(possibleBauds[BaudRateIndex]);
      GNSSSerialDebug.print(("Reconnected to GNSS module at "));
      GNSSSerialDebug.print(possibleBauds[BaudRateIndex]);
      GNSSSerialDebug.println((" baud."));
    }
    return true;
  }
  else {
    GNSSSerialDebug.println("GNSS Timeout -- Not Connected");
    return false;  
  }
}

void sendGps_int(int32_t latitude, int32_t longitude, int32_t groundspeed, int32_t heading, int32_t altitude, uint8_t satellites) {
  crsf_sensor_gps_t crsfGps = { 0 };

  // Values are MSB first (BigEndian)
  crsfGps.latitude = htobe32(latitude);
  crsfGps.longitude = htobe32(longitude);
  crsfGps.groundspeed = htobe16(groundspeed);
  crsfGps.heading = htobe16(heading);   //TODO: heading seems to not display in EdgeTX correctly, some kind of overflow error
  crsfGps.altitude = htobe16((uint16_t)(altitude + 1000));
  crsfGps.satellites = (uint8_t)(satellites);
  crsf.queuePacket(CRSF_SYNC_BYTE, CRSF_FRAMETYPE_GPS, &crsfGps, sizeof(crsfGps));
}

void printGNSS(void)
{
  static uint16_t i=0;

    snprintf(stringBuffer, STRING_BUFFER_SIZE, "%02d  : ", i);
    GNSSSerialDebug.print(stringBuffer);
    i++;i%=100;

    uint8_t FixType=myGNSS.getFixType();
    GNSSSerialDebug.print(F(" Fix Type: "));
    GNSSSerialDebug.print(FixType); // 0 = No Fix, 2 = 2D Fix, 3 = 3D Fix 

    uint8_t NumSat=myGNSS.getSIV();
    GNSSSerialDebug.print(F(" Sats: "));
    snprintf(stringBuffer, STRING_BUFFER_SIZE, "%2d", NumSat);
    GNSSSerialDebug.print(stringBuffer); 
    
    int32_t latitude = myGNSS.getLatitude();
    GNSSSerialDebug.print(F(" Lat: "));
    GNSSSerialDebug.print(latitude);

    int32_t longitude = myGNSS.getLongitude();
    GNSSSerialDebug.print(F(" Lon: "));
    GNSSSerialDebug.print(longitude);
    GNSSSerialDebug.print(F(" (degrees * 10^-7)"));

    int32_t altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    GNSSSerialDebug.print(F(" Alt: "));
    GNSSSerialDebug.print(altitude);
    GNSSSerialDebug.print(F(" (mm)"));

    uint32_t GroundSpeed=myGNSS.getGroundSpeed();
    GNSSSerialDebug.print(F(" Speed: "));
    snprintf(stringBuffer, STRING_BUFFER_SIZE, "%5d", GroundSpeed);
    GNSSSerialDebug.print(stringBuffer);
    GNSSSerialDebug.print(F(" (mm/s)"));

    int32_t Heading=myGNSS.getHeading();
    GNSSSerialDebug.print(F(" Heading: "));
    snprintf(stringBuffer, STRING_BUFFER_SIZE, "%8d", Heading);
    GNSSSerialDebug.print(stringBuffer);
    GNSSSerialDebug.print(F(" (degrees * 10^-5)"));  
/*
    uint16_t G_year = myGNSS.getYear();
    uint8_t G_month = myGNSS.getMonth();
    uint8_t G_day = myGNSS.getDay();
    uint8_t G_hour = myGNSS.getHour();
    uint8_t G_minute =myGNSS.getMinute();
    uint8_t G_second = myGNSS.getSecond();
    uint16_t G_millis =myGNSS.getMillisecond();
*/
    GNSSSerialDebug.println();
}

#endif  // TARGET_BLUEPILL  and BMP280 sensor

#ifdef TARGET_MATEK_CRSF_PWM_V10  // with SPL06-001 sensor
//#define SPL06_ADDRESS_ALT 0x76
//#define SPL06_PRODID 0x10


void setupBaroSensor(){   // SPL06-001 sensor version 
  SerialI2CDebug.println(F("SPL06-001 test"));
  unsigned status;
  BaroWire.begin();

// uint32_t read_data;
// getI2cData(SPL06_ADDRESS_ALT, 0x0d, 1, (short*) &read_data);

  delay(10);
  
  status = BaroSensor.begin(SPL06_ADDRESS_ALT,SPL06_PRODID);
  //unsigned status = BaroSensor.begin(SPL06_ADDRESS_ALT);
  if (!status) {
    SerialI2CDebug.println(F("Could not find a valid SPL06-001 sensor, check wiring or "
                      "try a different address!"));
    SerialI2CDebug.print("SensorID was:   0x"); SerialI2CDebug.println(BaroSensor.sensorID(),HEX); //HEX
    SerialI2CDebug.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    while (1) delay(10);
  }
  BaroSensor.setSampling(SPL06::MODE_BACKGND_BOTH,
                          SPL06::SAMPLING_X16,
                          SPL06::SAMPLING_X16,
                          SPL06::RATE_X16,
                          SPL06::RATE_X16);
  SerialI2CDebug.println("SPL06-001 ready");
}


bool autoBaudGNSS(void){
   return(false);
}
#endif  // TARGET_MATEK_CRSF_PWM_V10 with SPL06-001

