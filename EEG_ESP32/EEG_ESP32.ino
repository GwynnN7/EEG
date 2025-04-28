#include <ADS1220_WE.h>
#include <SPI.h>
#include <WiFi.h>

const char ssid[] = "Home&Life SuperWiFi-3451";
const char password[] = "3YRC8T4GB3X4A4XA";
const char host[] = "192.168.1.118";
const int port = 1234;

WiFiClient client;

#define ADS1220_CS_PIN    5
#define ADS1220_DRDY_PIN  4

ADS1220_WE ads = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);
float deltaError = 0;

void setup() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
  while (!client.connect(host, port)) delay(2000);

  while (!ads.init()) delay(2000);

  ads.setGain(ADS1220_GAIN_128);

  ads.setDataRate(ADS1220_DR_LVL_6);

  ads.setOperatingMode(ADS1220_TURBO_MODE);

  ads.setConversionMode(ADS1220_CONTINUOUS);

  ads.enableTemperatureSensor(false);

  ads.setAvddAvssAsVrefAndCalibrate();

  ads.setFIRFilter(ADS1220_50HZ_60HZ);

  ads.setCompareChannels(ADS1220_MUX_AVDD_P_AVSS_2);
  delay(1000);
  deltaError = getAverageData(50);

  ads.setCompareChannels(ADS1220_MUX_0_1);
  delay(500); 
}

void loop() {
  float result =  ads.getVoltage_muV() - deltaError;;
  client.write((uint8_t*)&result, sizeof(float));
}

float getAverageData(int num)
{
  float result = 0;
  for(int i=0; i<num; i++)
  {
    result += ads.getVoltage_muV();
  }
  return result / num;
}
