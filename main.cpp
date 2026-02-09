#include <Arduino.h>

#include <LiquidCrystal_I2C.h>

#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#define MQTT_MAX_PACKET_SIZE 512
#include "DHTesp.h" 

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
bool discoveryDone = false;


// Device ID unique
const char* device_id = "stationair";

// Topics de configuration (Discovery)
const char* discovery_temp = "homeassistant/sensor/stationair/temperature/config";
const char* discovery_hum = "homeassistant/sensor/stationair/humidity/config";
const char* discovery_co = "homeassistant/sensor/stationair/co/config";
const char* discovery_pressure = "homeassistant/sensor/stationair/pressure/config";

// Topics d'état (données)
const char* state_temp = "homeassistant/sensor/stationair/temperature/state";
const char* state_hum = "homeassistant/sensor/stationair/humidity/state";
const char* state_co = "homeassistant/sensor/stationair/co/state";
const char* state_pressure = "homeassistant/sensor/stationair/pressure/state";

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


void publishMQTTDiscovery() {
  if (!client.connected()) {
    Serial.println("Client non connecte");
    return;
  }
  
  Serial.println("=== MQTT Discovery ===");
  
  char payload[400];
  
  // TEMPÉRATURE
  snprintf(payload, sizeof(payload),
    "{\"name\":\"Temperature\","
    "\"dev_cla\":\"temperature\","
    "\"stat_t\":\"homeassistant/sensor/stationair/temperature/state\","
    "\"unit_of_meas\":\"°C\","
    "\"uniq_id\":\"stationair_temp\","
    "\"dev\":{"
      "\"ids\":[\"stationair\"],"
      "\"name\":\"Station Air\","
      "\"mdl\":\"ESP32 Capteurs\","
      "\"mf\":\"DIY\""
    "}}");
  
  if (client.publish("homeassistant/sensor/stationair/temperature/config", payload, true)) {
    Serial.println("OK Temp");
  }
  yield();
  delay(200);
  
  // HUMIDITÉ
  snprintf(payload, sizeof(payload),
    "{\"name\":\"Humidite\","
    "\"dev_cla\":\"humidity\","
    "\"stat_t\":\"homeassistant/sensor/stationair/humidity/state\","
    "\"unit_of_meas\":\"%%\","
    "\"uniq_id\":\"stationair_hum\","
    "\"dev\":{"
      "\"ids\":[\"stationair\"],"
      "\"name\":\"Station Air\","
      "\"mdl\":\"ESP32 Capteurs\","
      "\"mf\":\"DIY\""
    "}}");
  
  if (client.publish("homeassistant/sensor/stationair/humidity/config", payload, true)) {
    Serial.println("OK Hum");
  }
  yield();
  delay(200);
  
  // CO
  snprintf(payload, sizeof(payload),
    "{\"name\":\"CO\","
    "\"stat_t\":\"homeassistant/sensor/stationair/co/state\","
    "\"unit_of_meas\":\"ppm\","
    "\"icon\":\"mdi:molecule-co\","
    "\"uniq_id\":\"stationair_co\","
    "\"dev\":{"
      "\"ids\":[\"stationair\"],"
      "\"name\":\"Station Air\","
      "\"mdl\":\"ESP32 Capteurs\","
      "\"mf\":\"DIY\""
    "}}");
  
  if (client.publish("homeassistant/sensor/stationair/co/config", payload, true)) {
    Serial.println("OK CO");
  }
  yield();
  delay(200);
  
  // PRESSION
  snprintf(payload, sizeof(payload),
    "{\"name\":\"Pression\","
    "\"dev_cla\":\"pressure\","
    "\"stat_t\":\"homeassistant/sensor/stationair/pressure/state\","
    "\"unit_of_meas\":\"hPa\","
    "\"uniq_id\":\"stationair_pressure\","
    "\"dev\":{"
      "\"ids\":[\"stationair\"],"
      "\"name\":\"Station Air\","
      "\"mdl\":\"ESP32 Capteurs\","
      "\"mf\":\"DIY\""
    "}}");
  
  if (client.publish("homeassistant/sensor/stationair/pressure/config", payload, true)) {
    Serial.println("OK Pression");
  }
  yield();
  delay(200);
  
  Serial.println("=== Discovery termine ===");
}


void reconnect_mqtt() {
  while (!client.connected()) {
    Serial.print("Connexion au broker MQTT...");
    
    String clientId = "ESP32_StationAir_";
    clientId += String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
      Serial.println(" Connecté !");
      
      delay(500);
      
      if (!discoveryDone) {
    publishMQTTDiscovery();
    discoveryDone = true;
  }
      
    } else {
      Serial.print(" Échec : ");
      Serial.println(client.state());
      delay(5000);
    }
  }
}


void publishSensorData(float temperature, float humidity, float co_ppm, float pressure) {
  char payload[10];
  
  // Température
  dtostrf(temperature, 4, 1, payload);
  client.publish(state_temp, payload);
  
  // Humidité
  dtostrf(humidity, 4, 1, payload);
  client.publish(state_hum, payload);
  
  // CO
  dtostrf(co_ppm, 5, 1, payload);
  client.publish(state_co, payload);
  
  // Pression
  dtostrf(pressure, 6, 1, payload);
  client.publish(state_pressure, payload);
  
  Serial.println("✓ Données MQTT publiées");
}

/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
void setup() {

  Serial.begin(115200);  // Démarrer le port série pour voir les messages
  delay(1000);

  // Connexion WiFi
  setup_wifi();


   // Configuration MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
  client.setSocketTimeout(5);

client.setBufferSize(512);
  
  // VÉRIFIER LA TAILLE DU BUFFER
  Serial.print("Taille buffer MQTT: ");
  Serial.println(MQTT_MAX_PACKET_SIZE);
  
  // Si la taille n'est pas 512, forcer manuellement
  client.setBufferSize(512);
  Serial.println("Buffer MQTT forcé à 512");
  
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


  // ========== MAINTIEN CONNEXION MQTT (TOUJOURS EN PREMIER) ==========
  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();  // ✅ Appel régulier pour garder la connexion

  unsigned long now = millis();

  if ( now - lastMeasure > MESURE_INTERVAL) {
    
    lastMeasure = now;
  

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

    // Affichage série
    Serial.println("===== Mesures =====");
    Serial.print("AHT20  - T: "); Serial.print(tempAHT.temperature, 1);
    Serial.print(" °C  |  RH: "); Serial.println(humid.relative_humidity);
    Serial.print("BMP280 - T: "); Serial.print(tempBMP);
    Serial.print(" °C  |  P: "); Serial.println(press_hPa);
    Serial.println();

    // Affichage LCD lignes 0 et 1
    lcd.setCursor(0, 0); lcd.print("Tmp:"); lcd.print(tempAHT.temperature, 1); lcd.print("C ");
    lcd.setCursor(10, 0); lcd.print("Hum:"); lcd.print(humid.relative_humidity, 1); lcd.print("% ");
    lcd.setCursor(0, 1); lcd.print("Brgt:"); lcd.print(bright); lcd.print("   ");
    lcd.setCursor(10, 1); lcd.print("P:"); lcd.print(press_hPa, 0); lcd.print("hPa");

    // ========== PUBLICATION MQTT ==========
    publishSensorData(tempAHT.temperature, humid.relative_humidity, ppm, press_hPa);
    
  } // ✅ FIN du IF des mesures périodiques

  // ========== GESTION AFFICHAGE (en continu) ==========
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

  // Petit délai
  delay(100);
  
} // FIN du loop
