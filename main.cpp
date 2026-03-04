/*
Reste à faire : modif affichage Température en gros, 



*/

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

// ========== OBJETS CAPTEURS ==========
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;  //I2C

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

//Définition des contrastes
#define BRIGHTNESS_PIN 5   // Must be a PWM pin
// Définir la broche 33 comme entrée analogique
#define LDR 33
uint8_t bright;

// ---------- CONFIG MQ-7 / ESP32 ----------
#define MQ7_PIN 34              // Entrée analogique
#define CYCLE_3mn 180000    // 3mn entre deux mesures + publication 
#define RL_VALUE 10.0           // Résistance de charge en kOhms (10kΩ sur Flying Fish)
#define RO_CLEAN_AIR_FACTOR 27.5 // Ratio RS/RO dans l'air pur pour MQ7
#define ADC_RESOLUTION 4095.0   // Résolution ADC 12 bits ESP32
float Ro = 1.95;  // Résistance du capteur dans l'air pur (valeur par défaut, à calibrer)
unsigned long last_3m_time = -CYCLE_3mn;
int rawValue;
float rs;
float ratio;
float ppm;
float press_hPa;
float tempture;
float Humite;

// Caractères personnalisés optimisés pour chiffres LCD
byte LT[8] = {B00111, B01111, B11111, B11111, B11111, B11111, B11111, B11111};  // 0: Left Top
byte UB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B00000, B00000};  // 1: Upper Bar
byte RT[8] = {B11100, B11110, B11111, B11111, B11111, B11111, B11111, B11111};  // 2: Right Top
byte LL[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B01111, B00111};  // 3: Left Bottom
byte LB[8] = {B00000, B00000, B00000, B00000, B00000, B11111, B11111, B11111};  // 4: Lower Bar
byte LR[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11110, B11100};  // 5: Right Bottom
byte MB[8] = {B11111, B11111, B11111, B00000, B00000, B00000, B11111, B11111};  // 6: Middle Bar
byte block[8] = {B11111, B11111, B11111, B11111, B11111, B11111, B11111, B11111}; // 7: Full Block

// ========== VARIABLES AFFICHAGE LCD ==========
// Variables pour l'alternance d'affichage
const unsigned long DISPLAY_DURATION = 9000;  // 5 secondes
unsigned long lastDisplayChange = -DISPLAY_DURATION;
bool showBigPPM = true;  // true = afficher PPM, false = afficher détails
// Variables pour le défilement des infos
const unsigned long INFO_DURATION = 3000;  // 3sec par info
unsigned long lastInfoChange = -INFO_DURATION;
int currentInfo = 0;  // 0=RS, 1=Ratio, 2=Brut
int lastDisplayedInfo = -1;  // Pour savoir si l'affichage a changé

// ========== CONFIG WIFI ==========
WiFiMulti wifiMulti;

// ========== CONFIG MQTT ==========
const char* mqtt_server = "192.168.1.11";
const int   mqtt_port   = 1883;
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";

// ========== AJOUT : Variables pour gestion des reconnexions ==========
unsigned long last_1s_time = 0; 
const unsigned long CYCLE_30s = 30000;   // Vérifier WiFi toutes les 30s
unsigned long last_30s_time = -CYCLE_30s;
int mqttReconnectAttempts = 0;
const int MAX_MQTT_ATTEMPTS = 3;                   // Max 3 tentatives avant d'abandonner temporairement
static bool mqttWasDisconnected = false;  

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
  //réglage de l'intensité lumineuse du LCD selon la lumière ambiante
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

// Affiche un nombre entier en gros avec une décimale sur la ligne lign et colonne col
void printBigNumber(float number, int col, int lign) {

  int entier = (int)number;
  int decimale = (int)((number - entier) * 10);

  lcd.setCursor(0, lign); lcd.print("                    "); // Effacer ligne haute
  lcd.setCursor(0, lign+1); lcd.print("                    "); // Effacer ligne basse
  
  // Convertir en string pour compter les chiffres
  String entierStr = String(entier);
  int numDigitsEntier = entierStr.length();

  /*/
  // Calcul position de départ pour centrer (chaque chiffre = 3 colonnes + 1 espace)
  int startCol = (20 - (numDigits * 4 - 1)) / 2;
  */

  // Afficher chaque chiffre
  for(int i = 0; i < entierStr.length(); i++) {
    int digit = entierStr.charAt(i) - '0';  // Convertir char en int
    printBigDigit(digit, col + (i * 4), lign);
  }

  // Point décimal (caractère standard '.')
  int pointCol = col + (numDigitsEntier * 4);
  lcd.setCursor(pointCol, lign+1);
  lcd.print(".");  // ← Point standard du LCD

  // Partie décimale
  printBigDigit(decimale, pointCol+1, lign);
}

void setup_wifi() {
  
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
    Serial.println("\nWiFi connecté !");
    Serial.print("IP : ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nÉchec WiFi ! Réessai dans 30s...");
  }
}

void reconnect_mqtt() {

  mqttReconnectAttempts++;

  Serial.print("Tentative MQTT ");
  Serial.print(mqttReconnectAttempts);
  Serial.print("/");
  Serial.print(MAX_MQTT_ATTEMPTS);
  Serial.print("...");


  if (client.connect("stationair", mqtt_user, mqtt_pass,
                     "stationair/status", 0, true, "offline")) {
    
    Serial.println(" OK !");
    mqttReconnectAttempts = 0;  // Reset compteur
    
    client.publish("stationair/status", "online", true);
    client.loop();
    delay(100);
    
    Serial.println("Discovery...");
    
    // Temperature
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_temp_json);
    if (client.beginPublish("homeassistant/sensor/stationair_temp/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // Humidity
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_hum_json);
    if (client.beginPublish("homeassistant/sensor/stationair_hum/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // CO
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_co_json);
    if (client.beginPublish("homeassistant/sensor/stationair_co/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    client.loop();
    delay(500);
    
    // Pressure
    memset(mqttBuffer, 0, sizeof(mqttBuffer));
    strcpy_P(mqttBuffer, discovery_press_json);
    if (client.beginPublish("homeassistant/sensor/stationair_press/config", strlen(mqttBuffer), true)) {
      client.write((uint8_t*)mqttBuffer, strlen(mqttBuffer));
      client.endPublish();
    }
    
    Serial.println("Discovery OK");
    
    client.publish("stationair/data", 
                   "{\"temperature\":0,\"humidity\":0,\"co\":0,\"pressure\":0}", 
                   true);
    
  } else {
    Serial.print(" Échec (");
    Serial.print(client.state());
    Serial.println(")");
    
    // Après MAX_MQTT_ATTEMPTS échecs, attendre plus longtemps
    if (mqttReconnectAttempts >= MAX_MQTT_ATTEMPTS) {
      Serial.println("Trop d'échecs MQTT, pause jusqu'au prochain cycle");
      mqttReconnectAttempts = 0;

    }
  }
}



/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
/**********************************************************VOID SETUP*********************************************** */
void setup() {
  disableCore0WDT();
  Serial.begin(115200);
  delay(1000);

  setup_wifi();

  // Configuration MQTT
  client.setBufferSize(1700);
  client.setServer(mqtt_server, mqtt_port);
  client.setKeepAlive(60);
  client.setSocketTimeout(5);

  // Pins
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  pinMode(LDR, INPUT);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.createChar(0, LT);
  lcd.createChar(1, UB);
  lcd.createChar(2, RT);
  lcd.createChar(3, LL);
  lcd.createChar(4, LB);
  lcd.createChar(5, LR);
  lcd.createChar(6, MB);
  lcd.createChar(7, block);

  // I2C
  Wire.begin(21, 22);
  Serial.println("Init capteurs I2C...");

  // AHT20
  if (!aht.begin()) {
    Serial.println("ERREUR: AHT20 non détecté !");
    while (1) delay(10);
  }
  Serial.println("AHT20 OK");

  // BMP280
  if (!bmp.begin(0x76) && !bmp.begin(0x77)) {
    Serial.println("ERREUR: BMP280 non détecté !");
    while (1) delay(10);
  }
  Serial.println("BMP280 OK");

  bmp.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X2,
    Adafruit_BMP280::SAMPLING_X16,
    Adafruit_BMP280::FILTER_X16,
    Adafruit_BMP280::STANDBY_MS_500
  );

  // Première lecture sensor (puis toutes les 30s dans le loop)
  sensors_event_t humid, tempAHT;
  aht.getEvent(&humid, &tempAHT);
  press_hPa = bmp.readPressure() / 100.0F;
  tempture=tempAHT.temperature;
  Humite=humid.relative_humidity;
  
  rawValue = analogRead(MQ7_PIN);
  rs = readRS(rawValue);
  ratio = rs / Ro;
  ppm = calculatePPM(ratio);
   
  
  Serial.println("Setup terminé !");
} 



/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */
/*********************************************LOOP ************************************************ */

void loop() {

  unsigned long now = millis();

  /* toutes les secondes : client loop et actualisation du bright*/

  // ===== CLIENT.LOOP() TOUTES LES SECONDES et ajustement éclairage =====
  if (now - last_1s_time >= 1000) {
    last_1s_time = now;
    if (client.connected()) {
      client.loop();
    }
    //ajuste en permanence l'intensité du rétroéclairage et affiche les mesures sur le port série
    Retroeclairage();
    Serial.println("===== Mesures =====");
    Serial.printf("Bright: %i |T: %.1f°C | H: %.1f%% | P: %.0fhPa | CO: %.6fppm\n", bright, tempture, Humite, press_hPa, ppm);
  }


  /*Toutes les 30' vérification mqtt */

  // ===== VÉRIFICATION WIFI + MQTT + lecture  capteurs TOUTES LES 30 SECONDES =====
  if (now - last_30s_time >= CYCLE_30s) {
    last_30s_time = now;   

    // Lecture capteurs
    sensors_event_t humid, tempAHT;
    aht.getEvent(&humid, &tempAHT);
    press_hPa = bmp.readPressure() / 100.0F;
    tempture=tempAHT.temperature;
    Humite=humid.relative_humidity;
    
    rawValue = analogRead(MQ7_PIN);
    rs = readRS(rawValue);
    ratio = rs / Ro;
    ppm = calculatePPM(ratio);
   
    // Affichage série
    Serial.println("===== Nouvelles mesures =====");

  
     // LCD ligne 0 - 1
    printBigNumber(tempture,4,0);
    lcd.setCursor(16, 1);
    lcd.write(0xDF);
    lcd.print("C");

    // LCD ligne 2-3
    lcd.setCursor(0, 2); 
    lcd.printf("Hum:%4.1f%%  P:%4.0fhPa",Humite, press_hPa);
    lcd.setCursor(0, 3);
    // Afficher statut connexion
    if (WiFi.status() != WL_CONNECTED) {
      lcd.print("WiFi:OFF ");
    } else if (!client.connected()) {
      lcd.print("MQTT:OFF ");
    } else {
      lcd.printf("CO:%7.4f  Lum:%-3d  ", ppm, bright);
    }

    // 1. Vérifier WiFi
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi perdu, reconnexion...");
      lcd.setCursor(0, 3);
      lcd.print("WiFi OFF - Reconnex");
      setup_wifi();
      
      if (WiFi.status() == WL_CONNECTED) {
        mqttReconnectAttempts = 0;  // Reset si WiFi revient
      }
    }
    
    // 2. Vérifier MQTT (seulement si WiFi OK)
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
      Serial.println("MQTT déconnecté, reconnexion...");
      lcd.setCursor(0, 3);
      lcd.print("MQTT OFF - Reconnex");
      mqttWasDisconnected = true;
      reconnect_mqtt();
    }

    // 3. Effacer le message si MQTT vient de se connecter
    if (mqttWasDisconnected && client.connected()) {
      lcd.setCursor(0, 3);
      lcd.print("MQTT OK            ");  // Effacer le message
      mqttWasDisconnected = false;
      delay(1000);  // Afficher "MQTT OK" pendant 1 seconde
      lcd.setCursor(0, 3);
      lcd.print("                   ");  // Effacer complètement
    }
  }

  // ===== Envoi périodique MQTT (toutes les 3 minutes) =====
  if (now - last_3m_time > CYCLE_3mn) {
    last_3m_time = now;
    
    // Publication MQTT seulement si connecté
    if (client.connected()) {
      client.publish("stationair/status", "online", true);
      
      JsonDocument doc;
      doc["temperature"] = tempture;
      doc["humidity"] = Humite;
      doc["pressure"] = press_hPa;
      doc["co"] = ppm;

      char buffer[256];
      serializeJson(doc, buffer);
      
      if (client.publish("stationair/data", buffer, true)) {
        Serial.println("MQTT: Données publiées");
      } else {
        Serial.println("MQTT: Échec publication");
      }
    } else {
      Serial.println("MQTT: Non connecté, données non publiées");
    }
  }

  delay(10);
} // FIN du loop
