Important

Dans .pio\libdeps\heltec_wifi_lora_32_V4\Heltec ESP32 Dev-Boards\src\lora\LoRa.cpp remplacer le fichier par celui dans le commit ou changer les lignes 307 - 313 par : 

 if (callback) {
    writeRegister(REG_DIO_MAPPING_1, 0x00);
    attachInterrupt(_dio0, LoRaClass::onDio0Rise, RISING);
//    attachInterrupt(digitalPinToInterrupt(_dio0), LoRaClass::onDio0Rise, RISING);
  } else {
    detachInterrupt(_dio0);
  }

Changer le fichier : .pio\libdeps\heltec_wifi_lora_32_V4\TFT_eSPI\User_Setup.h par celui dans le commit