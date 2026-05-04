#include <Arduino.h>
#include "FS.h"
#include "SD_MMC.h"

#define SD_CLK 38
#define SD_CMD 39
#define SD_D0  40

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Starting SD test");

  SD_MMC.setPins(SD_CLK, SD_CMD, SD_D0);

  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("mount failed");
    while (true) delay(1000);
  }

  if (SD_MMC.cardType() == CARD_NONE) {
    Serial.println("No SD card");
    while (true) delay(1000);
  }

  File f = SD_MMC.open("/test.txt", FILE_WRITE);
  if (!f) {
    Serial.println("open failed");
    while (true) delay(1000);
  }

  f.println("hello");
  f.close();

  Serial.println("write ok");
}

void loop() {}