#include <arduino.h>
#include <Wire.h>
#include <SPI.h>

#include <AlfredoCRSF.h>

AlfredoCRSF crsf;


static void sendCellVoltage(uint8_t cellId, float voltage);
void sendBaroAltitude(float altitude, float verticalspd);
void update_baro_vals(uint32_t millis_now);
void display_baro_vals(uint32_t millis_now);
void get_I2C_Data(uint8_t I2C_Address,uint8_t device_register  , uint8_t num_bytes,short * indata);

#ifdef TARGET_BLUEPILL        // and BMP280  sensor
  #include <Adafruit_BMP280.h>
  
  #define LED_BUILTIN PB2
  //HardwareSerial Serial1(USART1); // somewhere in arduino already defined for Bluepill
  HardwareSerial Serial2(USART2);
  HardwareSerial Serial3(USART3);
  #define crsfSerial Serial1
  #define SerialI2CDebug Serial2
  
  #define SerialI2CDebug Serial2
  TwoWire BaroWire(PB9, PB8);
  Adafruit_BMP280 Baro_Sensor(&BaroWire); // I2C
  //Adafruit_BMP280 Baro_Sensor(&Wire); // I2C default pins for Wire PB7,PB6
  


void setup_BMP280(){
  SerialI2CDebug.println(F("BMP280 test"));
  unsigned status;
  BaroWire.begin();
  //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  status = Baro_Sensor.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
  if (!status) {
    SerialI2CDebug.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    SerialI2CDebug.print("SensorID was: 0x"); SerialI2CDebug.println(Baro_Sensor.sensorID(),16);
    SerialI2CDebug.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    SerialI2CDebug.print("        ID of 0x56-0x58 represents a BMP 280,\n");
    SerialI2CDebug.print("        ID of 0x60 represents a BME 280.\n");
    SerialI2CDebug.print("        ID of 0x61 represents a BME 680.\n");
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  Baro_Sensor.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1);   /* Standby time. */
}

#else      // TARGET_G031G8 and SPL06-001
  #include <SPL06-001.h>
  #define LED_BUILTIN PC14
  HardwareSerial Serial1(USART1); 
  HardwareSerial Serial2(USART2);

  #define crsfSerial Serial1
  #define SerialI2CDebug Serial2


//SPL06 Baro_Sensor(&Wire);
SPL06 Baro_Sensor;


void setup_SPL06_001(){

// uint32_t read_data;
  
// get_I2C_Data(SPL06_ADDRESS_ALT, 0x0d, 1, (short*) &read_data);

  delay(10);
  
  unsigned status = Baro_Sensor.begin(SPL06_ADDRESS_ALT,SPL06_PRODID);
  //unsigned status = Baro_Sensor.begin(SPL06_ADDRESS_ALT);
  if (!status) {
    SerialI2CDebug.println(F("Could not find a valid SPL06-001 sensor, check wiring or "
                      "try a different address!"));
    SerialI2CDebug.print("SensorID was:   0x"); SerialI2CDebug.println(Baro_Sensor.sensorID(),HEX); //HEX
    SerialI2CDebug.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
    while (1) delay(10);
  }
  Baro_Sensor.setSampling(SPL06::MODE_BACKGND_BOTH,
                          SPL06::SAMPLING_X16,
                          SPL06::SAMPLING_X16,
                          SPL06::RATE_X16,
                          SPL06::RATE_X16);
  SerialI2CDebug.println("SPL06-001 ready");
}
#endif  // TARGET_G031G8 and SPL06-001

void setup() {
  SerialI2CDebug.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  #ifdef TARGET_BLUEPILL
  
 //   while ( !Serial ) delay(100);   // wait for native usb
    SerialI2CDebug.println("Setup BMP280 starting");
    delay(100);    
    setup_BMP280();
    delay(100);
    crsfSerial.setTx(PB6);
    crsfSerial.setRx(PB7); 
  #else  
    SerialI2CDebug.println("Setup SPL06-001 starting");
    delay(100);
    setup_SPL06_001();
    delay(100);
    crsfSerial.setTx(PB6);
    crsfSerial.setRx(PB7);
  #endif
  delay(100);
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1);
  crsf.begin(crsfSerial);
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
  if(millis_now - millis_last > 250) {
    millis_last = millis_now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    sendCellVoltage(1, 4.11);
    sendBaroAltitude(filt_alt_AGL, filt_vario);
  }
  update_baro_vals(millis_now);
  display_baro_vals(millis_now);
  if (millis_now-crsf_last_update > 1) {
    crsf.update();
    crsf_last_update = millis_now;
  }
  main_loop_counter++;
}

void display_baro_vals(uint32_t millis_now){
  static uint32_t last_millis=0;
  if (millis_now - last_millis < 500) return;
  last_millis = millis_now;
  SerialI2CDebug.print(F("Temperature = "));
  SerialI2CDebug.print(Baro_Sensor.readTemperature());
  SerialI2CDebug.println(" *C");
  SerialI2CDebug.print(F("Pressure = "));
  SerialI2CDebug.print(Baro_Sensor.readPressure());
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


void update_baro_vals(uint32_t millis_now){
  float altitude;
  static uint32_t last_millis=0,loop_counter=0;
  if (millis_now - last_millis < 50) return;
  last_millis = millis_now;

  #ifdef TARGET_BLUEPILL  // BMP280 sensor
    altitude = Baro_Sensor.readAltitude(1013.25); /* Adjusted to local forecast! */
  #else                   // SPL06-001 sensor
    altitude = Baro_Sensor.readPressureAltitudeMeter(1013.25);
  #endif
  if (loop_counter <1000) {    // initialisation --> average ground altitude
    GND_altitude += altitude;
    GND_alt_count++;
    filt_alt_ASL = altitude;
  } else {
    filt_alt_ASL = (IIR_ALPHA * altitude) + (IIR_BETA * filt_alt_ASL);
  }
  filt_alt_AGL =filt_alt_ASL - (GND_altitude / GND_alt_count);
  vario=(previous_alt_ASL - filt_alt_AGL)*20;
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

void sendCellVoltage(uint8_t cellId, float voltage) {
  if (cellId < 1 || cellId > CRSF_BATTERY_SENSOR_CELLS_MAX)     return;

  uint8_t payload[3];
  payload[0] = cellId;
  uint16_t voltage_be = htobe16((uint16_t)(voltage * 1000.0)); //mV
  memcpy(&payload[1], &voltage_be, sizeof(voltage_be));
  crsf.queuePacket(CRSF_SYNC_BYTE, 0x0e, payload, sizeof(payload));
}

void sendBaroAltitude(float altitude, float verticalspd)
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


void get_I2C_Data(uint8_t I2C_Address,uint8_t device_register  , uint8_t num_bytes,short * indata) {
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
