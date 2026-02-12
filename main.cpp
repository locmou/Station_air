#include <Arduino.h>

#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#define MQTT_MAX_PACKET_SIZE 1700
#include <PubSubClient.h>


#include <ArduinoJson.h>

// Objets capteurs
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;   // I2C

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

//Définition des contrastes
#define BRIGHTNESS_PIN 5   // Must be a PWM pin

// Définir la broche 33 comme entrée analogique
#define LDR 33
uint8_t bright;

// ---------- CONFIG MQ-7 / ESP32 ----------
#define MQ7_PIN 34              // Entrée analogique
#define MESURE_INTERVAL 10000    // ms entre deux mesures + publication
#define RL_VALUE 10.0           // Résistance de charge en kOhms (10kΩ sur Flying Fish)
#define RO_CLEAN_AIR_FACTOR 27.5 // Ratio RS/RO dans l'air pur pour MQ7
#define ADC_RESOLUTION 4095.0   // Résolution ADC 12 bits ESP32

float Ro = 1.95;  // Résistance du capteur dans l'air pur (valeur par défaut, à calibrer)

unsigned long lastMeasure = 0;

// Caractères personnalisés optimisés pour chiffres LCD
byte LT[8] = {B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111};  // 0: Left Top
byte UB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000};  // 1: Upper Bar
byte RT[8] = {B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111};  // 2: Right Top
byte LL[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111};  // 3: Left Bottom
byte LB[8] = {B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111};  // 4: Lower Bar
byte LR[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100};  // 5: Right Bottom
byte MB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111};  // 6: Middle Bar
byte block[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}; // 7: Full Block

// Variables pour l'alternance d'affichage
unsigned long lastDisplayChange = 0;
const unsigned long DISPLAY_DURATION = 9000;  // 5 secondes
bool showBigPPM = true;  // true = afficher PPM, false = afficher détails

// Variables pour le défilement des infos
unsigned long lastInfoChange = 0;
const unsigned long INFO_DURATION = 3000;  // 3sec par info
int currentInfo = 0;  // 0=RS, 1=Ratio, 2=Brut
int lastDisplayedInfo = -1;  // Pour savoir si l'affichage a changé

// ---------- OBJET WiFiMulti ----------
WiFiMulti wifiMulti;

// ---------- CONFIG MQTT ----------
const char* mqtt_server = "192.168.1.11";
const int   mqtt_port   = 1883;
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";
bool discoveryPublished = false;
char DEVICE[300];
char ESP_ID[15];
char StateTopic[50];
bool discoveryDone = false;
unsigned long discoveryTimer = 0;

// ========== MESSAGES DISCOVERY EN PROGMEM ==========
// Stockage en Flash au lieu de RAM pour économiser la mémoire

const char discovery_temp_json[] PROGMEM = R"({
"name":"Temperature",
"uniq_id":"stationair_temp",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"temperature",
"unit_of_meas":"°C",
"val_tpl":"{{value_json.temperature}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_hum_json[] PROGMEM = R"({
"name":"Humidite",
"uniq_id":"stationair_hum",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"humidity",
"unit_of_meas":"%",
"val_tpl":"{{value_json.humidity}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_co_json[] PROGMEM = R"({
"name":"CO",
"uniq_id":"stationair_co",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"unit_of_meas":"ppm",
"val_tpl":"{{value_json.co}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

const char discovery_press_json[] PROGMEM = R"({
"name":"Pression",
"uniq_id":"stationair_press",
"stat_t":"stationair/data",
"avty_t":"stationair/status",
"dev_cla":"pressure",
"unit_of_meas":"hPa",
"val_tpl":"{{value_json.pressure}}",
"device":{"ids":["stationair"],"name":"Station Air","mf":"DIY","mdl":"ESP32"}
})";

// Après les 4 messages discovery_xxx_json
char mqttBuffer[600];  // Buffer global


WiFiClient espClient;
PubSubClient client(espClient);




//////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// FONCTIONS ////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////

/**********************************VOID RETROECLAIRAGE*************************************** */
void Retroeclairage(){
  //réglage de l'intensité lumineus du LCD selon la lumière ambiante
  bright=(analogRead(LDR)/4);
  analogWrite(BRIGHTNESS_PIN, bright);
  }

/**********************************Lecture mq7************************************************ */
float readRS(int adcValue) {
  
  // Conversion ADC vers tension (0-3.3V sur ESP32)
  float voltage = (adcValue / ADC_RESOLUTION) * 3.3;
  
  // Calcul de RS : RS = [(Vc × RL) / Vout] - RL
  float rs = ((3.3 * RL_VALUE) / voltage) - RL_VALUE;
  
  return rs;
}

float calculatePPM(float ratio) {
  // Constantes de la courbe MQ7 pour le CO
  float m = -0.46;  // Pente de la courbe log-log
  float b = 0.42;   // Ordonnée à l'origine
  
  // Formule : PPM = 10^[(log(ratio) - b) / m]
  float ppm = pow(10, ((log10(ratio) - b) / m));
  
  return ppm;
}


// Affiche un chiffre en gros (3 colonnes × 2 lignes)
void printBigDigit(int digit, int col, int row) {
  switch(digit) {
    case 0:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(1); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(5);
      break;
    case 1:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(2); lcd.print(" ");
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(7); lcd.write(4);
      break;
    case 2:
      lcd.setCursor(col, row);     lcd.write(6); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(4);
      break;
    case 3:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(6); lcd.write(5);
      break;
    case 4:
      lcd.setCursor(col, row);     lcd.write(3); lcd.write(4); lcd.write(7);
      lcd.setCursor(col, row + 1); lcd.print(" "); lcd.print(" "); lcd.write(7);
      break;
    case 5:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(6);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(4); lcd.write(5);
      break;
    case 6:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(6);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(4); lcd.write(5);
      break;
    case 7:
      lcd.setCursor(col, row);     lcd.write(1); lcd.write(1); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.print(" "); lcd.print(" "); lcd.write(7);
      break;
    case 8:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(3); lcd.write(6); lcd.write(5);
      break;
    case 9:
      lcd.setCursor(col, row);     lcd.write(0); lcd.write(6); lcd.write(2);
      lcd.setCursor(col, row + 1); lcd.write(4); lcd.write(4); lcd.write(5);
      break;
  }
}

// Affiche un nombre entier en gros au centre des lignes 2-3
void printBigNumber(int number) {
  lcd.setCursor(0, 2); lcd.print("                    "); // Effacer ligne 2
  lcd.setCursor(0, 3); lcd.print("                    "); // Effacer ligne 3
  
  // Convertir en string pour compter les chiffres
  String numStr = String(number);
  int numDigits = numStr.length();
  
  // Calcul position de départ pour centrer (chaque chiffre = 3 colonnes + 1 espace)
  int startCol = (20 - (numDigits * 4 - 1)) / 2;
  
  // Afficher chaque chiffre
  for(int i = 0; i < numDigits; i++) {
    int digit = numStr.charAt(i) - '0';  // Convertir char en int
    printBigDigit(digit, startCol + (i * 4), 2);
  }
}

void setup_wifi() {
  
  Serial.println();
  Serial.println("=== Connexion WiFi ===");

  WiFi.mode(WIFI_STA); // Mode Station (client WiFi)

  // Ajoute ici tous les réseaux possibles
  wifiMulti.addAP("Mounwiff",    "rue_de_la_Grande680Plage_10!");
  //wifiMulti.addAP("Mounwiff", "en_face_du_20_rue_des_joncs");

  Serial.print("Connexion en cours");

  // Tentative de connexion (timeout 10 secondes)
  int attempts = 0;
  while (wifiMulti.run() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  // Boucle jusqu'à connexion sur l'un des réseaux
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println("WiFi connecté !");
    Serial.print("Adresse IP : ");
    Serial.println(WiFi.localIP());
    Serial.print("SSID : ");
    Serial.println(WiFi.SSID());
  } else {
    Serial.println();
    Serial.println("Échec de connexion WiFi !");
  }
}


void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Connexion au broker MQTT...");
    
    if (client.connect("stationair",
                       mqtt_user,
                       mqtt_pass,
                       "stationair/status",
                       0,
                       true,
                       "offline")) {
      
      Serial.println(" Connecté !");
      Serial.print("Buffer size actuel: ");
      Serial.println(client.getBufferSize());
      
      // Status online
      client.publish("stationair/status", "online", true);
      client.loop();
      delay(100);
      
      Serial.println("D-Start");
      
      // Temperature
      Serial.print("Temp...");
      memset(mqttBuffer, 0, sizeof(mqttBuffer));
      strcpy_P(mqttBuffer, discovery_temp_json);
      Serial.print(strlen(mqttBuffer)); Serial.print("b...");
      
      // Publier sans attendre la confirmation (QoS 0)
      if (client.beginPublish("homeassistant/sensor/stationair_temp/config", strlen(mqttBuffer), true)) {
        client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
        client.endPublish();
        Serial.println("OK");
      } else {
        Serial.println("FAIL");
      }
      
      client.loop();
      delay(500);  // Délai plus long
      
      // Humidity
      Serial.print("Hum...");
      memset(mqttBuffer, 0, sizeof(mqttBuffer));
      strcpy_P(mqttBuffer, discovery_hum_json);
      Serial.print(strlen(mqttBuffer)); Serial.print("b...");
      
      if (client.beginPublish("homeassistant/sensor/stationair_hum/config", strlen(mqttBuffer), true)) {
        client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
        client.endPublish();
        Serial.println("OK");
      } else {
        Serial.println("FAIL");
      }
      
      client.loop();
      delay(500);
      
      // CO
      Serial.print("CO...");
      memset(mqttBuffer, 0, sizeof(mqttBuffer));
      strcpy_P(mqttBuffer, discovery_co_json);
      Serial.print(strlen(mqttBuffer)); Serial.print("b...");
      
      if (client.beginPublish("homeassistant/sensor/stationair_co/config", strlen(mqttBuffer), true)) {
        client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
        client.endPublish();
        Serial.println("OK");
      } else {
        Serial.println("FAIL");
      }
      
      client.loop();
      delay(500);
      
      // Pressure
      Serial.print("Press...");
      memset(mqttBuffer, 0, sizeof(mqttBuffer));
      strcpy_P(mqttBuffer, discovery_press_json);
      Serial.print(strlen(mqttBuffer)); Serial.print("b...");
      
      if (client.beginPublish("homeassistant/sensor/stationair_press/config", strlen(mqttBuffer), true)) {
        client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
        client.endPublish();
        Serial.println("OK");
      } else {
        Serial.println("FAIL");
      }
      
      client.loop();
      Serial.println("D-End");
      
      // Première donnée
      client.publish("stationair/data", 
                     "{\"temperature\":0,\"humidity\":0,\"co\":0,\"pressure\":0}", 
                     true);
      
      break;  // Sortir du while après succès
      
    } else {
      Serial.print(" Échec : ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}



/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
void setup() {

// Désactiver le watchdog pendant l'init
disableCore0WDT();

  Serial.begin(115200);  // Démarrer le port série pour voir les messages
  delay(1000);

  // Connexion WiFi
  setup_wifi();

    // Créer l'ID unique de l'appareil
  byte mac[6];
  WiFi.macAddress(mac);
  sprintf(ESP_ID, "%02x%02x%02x%02x%02x", mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("RAM libre : ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");

  // Créer le JSON DEVICE
  String nomRouteur = "Station Air";
  String mdl = "ESP32-" + String(ESP_ID);
  String mf = "DIY";
  String hw = String(ESP.getChipModel());
  String sw = "v1.0";
  String cu = "http://" + WiFi.localIP().toString();
  
  sprintf(DEVICE, 
    "{\"ids\":\"%s\","
    "\"name\":\"%s\","
    "\"mdl\":\"%s\","
    "\"mf\":\"%s\","
    "\"hw\":\"%s\","
    "\"sw\":\"%s\","
    "\"cu\":\"%s\"}", 
    ESP_ID, nomRouteur.c_str(), mdl.c_str(), mf.c_str(), hw.c_str(), sw.c_str(), cu.c_str()
  );
  
  sprintf(StateTopic, "homeassistant/sensor/stationair/state");

   // Configuration MQTT
// Dans setup(), AVANT client.setServer()
client.setBufferSize(1700);
client.setServer(mqtt_server, mqtt_port);
client.setKeepAlive(60);
client.setSocketTimeout(5);
   Serial.print("Buffer MQTT configuré : ");
  Serial.println(client.getBufferSize());  // ← AJOUTER pour vérifier
  
  
  Serial.println("Configuration MQTT terminée");

  // Déclaration des broches
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  pinMode(LDR, INPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Créer les caractères personnalisés
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, MB);
  lcd.createChar(7, block);

  // Init I2C sur les pins ESP32 (21 = SDA, 22 = SCL)
  Wire.begin(21, 22);
  Serial.println("Init AHT20 + BMP280");


  // --- AHT20 ---
  if (!aht.begin()) {          // auto‑détection AHT10/AHT20 à l'adresse I2C 0x38[web:2]
  Serial.println("Erreur: AHT20 non detecte, verifier le cablage !");
  while (1) delay(10);
  }
  Serial.println("AHT20 OK");

  // --- BMP280 ---
  // Adresse I2C la plus fréquente : 0x76 ; si échec, essayer 0x77[web:1][web:3]
  if (!bmp.begin(0x76)) {
    Serial.println("BMP280 0x76 non detecte, essai 0x77...");
    if (!bmp.begin(0x77)) {
      Serial.println("Erreur: BMP280 non detecte, verifier le cablage/adresse !");
      while (1) delay(10);
    }
  }
  Serial.println("BMP280 OK");

  // Configuration BMP280 (exemple de réglages "classiques")[web:1]
  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,   // temp
    Adafruit_BMP280::SAMPLING_X16,  // pression
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );
  
} 



/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */

void loop() {
  // Vérifier WiFi
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi déconnecté, reconnexion...");
    setup_wifi();
  }

  // Maintien connexion MQTT
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();
  
  unsigned long now = millis();

  if (now - lastMeasure > MESURE_INTERVAL) {
    lastMeasure = now;
    
    // Republier status périodiquement
    client.publish("stationair/status", "online", true);

    // Réglage LED rétroéclairage
    Retroeclairage();

    // Lecture AHT20
    sensors_event_t humid, tempAHT;
    aht.getEvent(&humid, &tempAHT);

    // Lecture BMP280
    float tempBMP = bmp.readTemperature();
    float press_hPa = bmp.readPressure() / 100.0F;

    // Lecture MQ-7
    int rawValue = analogRead(MQ7_PIN);
    float rs = readRS(rawValue);
    float ratio = rs / Ro;
    float ppm = calculatePPM(ratio);

    // Publication MQTT
    JsonDocument doc;
    doc["temperature"] = tempAHT.temperature;
    doc["humidity"]    = humid.relative_humidity;
    doc["pressure"]    = press_hPa;
    doc["co"]          = ppm;

    char buffer[256];
    serializeJson(doc, buffer);
    client.publish("stationair/data", buffer, true);

    // Affichage série
    Serial.println("===== Mesures =====");
    Serial.print("AHT20  - T: "); Serial.print(tempAHT.temperature, 1);
    Serial.print(" °C  |  RH: "); Serial.println(humid.relative_humidity);
    Serial.print("BMP280 - T: "); Serial.print(tempBMP);
    Serial.print(" °C  |  P: "); Serial.println(press_hPa);
    Serial.print("MQ-7   - CO: "); Serial.print(ppm); Serial.println(" ppm");
    Serial.println();

    // Affichage LCD lignes 0 et 1
    lcd.setCursor(0, 0); lcd.print("Tmp:"); lcd.print(tempAHT.temperature, 1); lcd.print("C ");
    lcd.setCursor(10, 0); lcd.print("Hum:"); lcd.print(humid.relative_humidity, 1); lcd.print("% ");
    lcd.setCursor(0, 1); lcd.print("Brgt:"); lcd.print(bright); lcd.print("   ");
    lcd.setCursor(10, 1); lcd.print("P:"); lcd.print(press_hPa, 0); lcd.print("hPa");
  }

  // ========== GESTION AFFICHAGE LCD LIGNES 2-3 (en continu) ==========
  // Relecture MQ7 pour affichage en temps réel
  int rawValue = analogRead(MQ7_PIN);
  float rs = readRS(rawValue);
  float ratio = rs / Ro;
  float ppm = calculatePPM(ratio);

  // Alternance d'affichage
  if (now - lastDisplayChange >= DISPLAY_DURATION) {
    lastDisplayChange = now;
    showBigPPM = !showBigPPM;
    currentInfo = 0;
    lastInfoChange = now;
  }

  // Affichage selon le mode
  if (showBigPPM) {
    if (lastDisplayedInfo != -2) {
      lcd.setCursor(0, 2); lcd.print("                    ");
      lcd.setCursor(0, 3); lcd.print("                    ");
      printBigNumber((int)ppm);
      lcd.setCursor(0, 3); lcd.print("CO :");
      lcd.setCursor(17, 3); lcd.print("ppm");
      lastDisplayedInfo = -2;
    }
    
  } else {
    // Défilement des infos
    if (now - lastInfoChange >= INFO_DURATION) {
      lastInfoChange = now;
      currentInfo++;
      if (currentInfo > 2) currentInfo = 0;
    }
    
    if (lastDisplayedInfo != currentInfo) {
      lcd.setCursor(0, 2); lcd.print("                    ");
      lcd.setCursor(0, 3); lcd.print("                    ");
      
      switch(currentInfo) {
        case 0:
          lcd.setCursor(0, 2); lcd.print("RS (Resistance):");
          lcd.setCursor(0, 3); lcd.print(rs, 2); lcd.print(" kOhms");
          break;
        case 1:
          lcd.setCursor(0, 2); lcd.print("Ratio RS/Ro:");
          lcd.setCursor(0, 3); lcd.print(ratio, 2);
          break;
        case 2:
          lcd.setCursor(0, 2); lcd.print("Valeur brute ADC:");
          lcd.setCursor(0, 3); lcd.print(rawValue);
          break;
      }
      lastDisplayedInfo = currentInfo;
    }
  }

  delay(10);
} // FIN du loop
