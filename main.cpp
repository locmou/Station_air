#include <Arduino.h>


#include <LiquidCrystal_I2C.h>
/*************** a ajouter pour passer au aht/BMP **************
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
*/
/***************** a ajouter pour le wifi
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
*/
#include "DHTesp.h" 

/*************** a ajouter pour passer au aht/BMP **************
// Objets capteurs
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;   // I2C
*/

/** Initialize DHT sensor 1 */
DHTesp dhtSensor;

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

/** Pin number for DHT11 1 data pin */
#define dhtPin 32

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

/*****************************Congfig wifi********************
// ---------- OBJET WiFiMulti ----------
WiFiMulti wifiMulti;

// ---------- CONFIG MQTT ----------
const char* mqtt_server = "192.168.1.11"; // IP ou hostname du broker
const int   mqtt_port   = 1883;
const char* mqtt_topic  = "maison/co/mq7";
const char* mqtt_user = "loic.mounier@laposte.net";
const char* mqtt_pass = "vgo:?2258H";

WiFiClient espClient;
PubSubClient client(espClient);
*/

/**********************************VOID RETROECLAIRAGE*************************************** */
void Retroeclairage(){
  //réglage de l'intensité lumineus du LCD selon la lumière ambiante
  bright=(analogRead(LDR)/4);
  analogWrite(BRIGHTNESS_PIN, bright);
  }

/**********************************Lecture mq7*************************************** */
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
/*******************************WIFI/MQTT**********************************
// ---------- FONCTIONS ----------
void setup_wifi() {
  delay(10);
  WiFi.mode(WIFI_STA);

  // Ajoute ici tous les réseaux possibles
  wifiMulti.addAP("Mounwiff",    "rue_de_la_Grande680Plage_10!");
  wifiMulti.addAP("SSID_BUREAU", "mdp_bureau");
  wifiMulti.addAP("SSID_TEL",    "mdp_tel");

  // Boucle jusqu'à connexion sur l'un des réseaux
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(500);
  }
}

void reconnect_mqtt() {
  while (!client.connected()) {
    // clientId doit être unique
    if (client.connect("ESP32_MQ7", mqtt_user, mqtt_pass)) {
      // connecté
    } else {
      delay(2000);
    }
  }
}
*/

/***********************************VOID SETUP*********************************************** */
void setup() {

  /*********************WIFI/MQTT**************
  // WiFi + MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
 */

  /*démarrage port série
  Serial.begin(115200);
  while(!Serial);  // Attendre connexion USB (PC/Mac/Linux)
  Serial.println("Prêt !");
  */

  /* wire pour le lecteur bmp/aht
  // Init I2C sur les pins ESP32 (21 = SDA, 22 = SCL)
  Wire.begin(21, 22);
  Serial.println("Init AHT20 + BMP280");
  */

  /***** a ajouter pour passer au aht/BMP **************

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
  */

// Déclaration des broches
  pinMode(BRIGHTNESS_PIN, OUTPUT);
  pinMode(LDR, INPUT);

// LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

// Initialize temperature sensor 1
  dhtSensor.setup(dhtPin, DHTesp::DHT11);
  
} 

/*********************************************VOID LOOP ************************************************ */
void loop() {

  unsigned long now = millis();
  if (now - lastMeasure > MESURE_INTERVAL) {
    lastMeasure = now;

    // réglage led rétroéclairage
    Retroeclairage();

    // Lecture des données du capteur
    TempAndHumidity data = dhtSensor.getTempAndHumidity();

    /*
    // Affichage sur le moniteur série
    Serial.print("Température: ");
    Serial.print(data.temperature, 1);
    Serial.print("°C, Humidité: ");
    Serial.print(data.humidity, 1);
    Serial.println("%");
    */

    // Affichage sur le LCD
    lcd.setCursor(0, 0);  lcd.print("Tmp:");  lcd.print(data.temperature, 1);  lcd.print(" C");
    lcd.setCursor(10, 0);  lcd.print("Hum:");  lcd.print(data.humidity, 1);  lcd.print(" %");

    // Affichage bright 
    lcd.setCursor(0, 1);  lcd.print("Bright: ");  lcd.print(bright);

    // Lecture du MQ-7 avec calcul calibré
    int rawValue = analogRead(MQ7_PIN);
    float rs = readRS(rawValue);  // Calcul de la résistance RS, On passe rawValue
    float ratio = rs / Ro;            // Calcul du ratio RS/Ro
    float ppm = calculatePPM(ratio);  // Conversion en PPM réels

    /************* a ajouter pour passer au aht/BMP **************
    /*  // ----- Lecture AHT20 -----
    sensors_event_t humid, tempAHT;
    aht.getEvent(&humid, &tempAHT);   // remplit tempAHT.temperature et humid.relative_humidity[web:2]

    // ----- Lecture BMP280 -----
    float tempBMP  = bmp.readTemperature();   // °C[web:1]
    float press_hPa = bmp.readPressure() / 100.0F;  // hPa[web:1]

    // ----- Affichage -----
    lcd.setCursor(0, 0);  lcd.print("Tmp:");  lcd.print(tempAHT.temperature);  lcd.print(" C");
    lcd.setCursor(10, 0);  lcd.print("Hum:");  lcd.print(humid.relative_humidity);  lcd.print(" %");
    lcd.setCursor(0, 1);  lcd.print("Press:");  lcd.print(press_hPa);  lcd.print(" hPa");
    lcd.setCursor(10, 1);  lcd.print("Bright:");  lcd.print(bright);  


    //---------Affichagesérie--------
    Serial.println("===== Mesures =====");
    Serial.print("AHT20  - T: ");
    Serial.print(tempAHT.temperature);
    Serial.print(" °C  |  RH: ");
    Serial.print(humid.relative_humidity);
    Serial.println(" %");

    Serial.print("BMP280 - T: ");
    Serial.print(tempBMP);
    Serial.print(" °C  |  P: ");
    Serial.print(press_hPa);
    Serial.println(" hPa");

    Serial.println();
    delay(2000);   // 2 s entre deux mesures*/


    // Affichage série
    /*Serial.print("MQ7 brut = ");
    Serial.print(rawValue);
    Serial.print("  ~");
    Serial.print(pseudoPPM);
    Serial.println(" ppm (approx)");*/

    // Affichage LCD 20x4
    lcd.setCursor(0, 2);
    lcd.print("CO MQ7 =>  "); lcd.print("Brut : "); lcd.print(rawValue);

    lcd.setCursor(0, 3);
    lcd.print("Approx :");
    lcd.print((int)pseudoPPM);
    lcd.print(" ppm");

/*
    lcd.setCursor(0, 3);
    lcd.print("WiFi: ");
    lcd.print(WiFi.SSID());
*/
/*********************pUBLI mqtt**************************
    // Publication MQTT (envoi de la valeur brute)
    char payload[32];
    snprintf(payload, sizeof(payload), "%d", rawValue);
    client.publish(mqtt_topic, payload);
*/

  }

}