#include <ADS1220_WE.h>
#include <SPI.h>

#define ADS1220_CS_PIN 5
#define ADS1220_DRDY_PIN 4

ADS1220_WE ads = ADS1220_WE(ADS1220_CS_PIN, ADS1220_DRDY_PIN);

void setup()
{
  Serial.begin(250000);
  if (!ads.init())
  {
    while (1)
    {
      Serial.println("ADS1220 Error");
      delay(1000);
    }
  }

  ads.setGain(ADS1220_GAIN_1);
  ads.setDataRate(ADS1220_DR_LVL_0);
  ads.setConversionMode(ADS1220_SINGLE_SHOT);

  ads.setAvddAvssAsVrefAndCalibrate();
  float supplyVoltage = ads.getVRef_V();

  Serial.print("Battery Voltage: ");
  Serial.print(supplyVoltage, 3);
  Serial.println(" V");

  if (supplyVoltage < 2.2)
  {
    Serial.println("WARNING: Battery Low!");
  }

  delay(1000);

  ads.setGain(ADS1220_GAIN_128);
  ads.setDataRate(ADS1220_DR_LVL_4);
  ads.setOperatingMode(ADS1220_TURBO_MODE);
  ads.setConversionMode(ADS1220_SINGLE_SHOT);
  ads.setFIRFilter(ADS1220_NONE);
}

void loop()
{
  ads.setCompareChannels(ADS1220_MUX_0_2);
  float val_C3 = ads.getVoltage_muV();

  ads.setCompareChannels(ADS1220_MUX_1_2);
  float val_C4 = ads.getVoltage_muV();

  Serial.write(0xAA);
  Serial.write((uint8_t *)&val_C3, sizeof(float));
  Serial.write((uint8_t *)&val_C4, sizeof(float));
}
