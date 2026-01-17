#include <Arduino.h>
/*
#include <WiFi.h>
#include <WiFiMulti.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>

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

// ---------- CONFIG LCD ----------
LiquidCrystal_I2C lcd(0x27, 20, 4); // Adapter l'adresse I2C (0x27 ou 0x3F)

// ---------- CONFIG MQ-7 / ESP32 ----------
#define MQ7_PIN 34              // Entrée analogique
#define MESURE_INTERVAL 10000    // ms entre deux mesures + publication
unsigned long lastMeasure = 0;

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

// ---------- SETUP UNIQUE ----------
void setup() {
  Serial.begin(115200);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MQ7 CO Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Demarrage...");
  lcd.setCursor(0, 2);
  lcd.print("WiFi / MQTT");
  lcd.setCursor(0, 3);
  lcd.print("Veuillez patienter");

  // WiFi + MQTT
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi OK");
  lcd.setCursor(0, 1);
  lcd.print("MQTT connexion");

  reconnect_mqtt();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pret");
}

// ---------- LOOP UNIQUE ----------
void loop() {
  // Maintien / verification WiFi
  wifiMulti.run();

  if (!client.connected()) {
    reconnect_mqtt();
  }
  client.loop();

  unsigned long now = millis();
  if (now - lastMeasure > MESURE_INTERVAL) {
    lastMeasure = now;

    // Lecture brute du MQ-7 (0-4095 sur ESP32)
    int rawValue = analogRead(MQ7_PIN);

    // Conversion approximative en "pseudo ppm"
    float pseudoPPM = (rawValue / 4095.0) * 1000.0;

    // Affichage série
    Serial.print("MQ7 brut = ");
    Serial.print(rawValue);
    Serial.print("  ~");
    Serial.print(pseudoPPM);
    Serial.println(" ppm (approx)");

    // Affichage LCD 20x4
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CO MQ7");

    lcd.setCursor(0, 1);
    lcd.print("Brut : ");
    lcd.print(rawValue);

    lcd.setCursor(0, 2);
    lcd.print("Approx : ");
    lcd.print((int)pseudoPPM);
    lcd.print(" ppm");

    lcd.setCursor(0, 3);
    lcd.print("WiFi: ");
    lcd.print(WiFi.SSID());

    // Publication MQTT (envoi de la valeur brute)
    char payload[32];
    snprintf(payload, sizeof(payload), "%d", rawValue);
    client.publish(mqtt_topic, payload);
  }
}
  */

  /*
* Étalonnage du capteur MQ7 (CO) sur ESP32 WROOM
 * 
 * Connexions:
 * - VCC MQ7 -> 5V (si disponible) ou 3.3V
 * - GND MQ7 -> GND
 * - AOUT MQ7 -> GPIO 34 (ADC1_CH6)
 * 
 * Le MQ7 nécessite un cycle de chauffage:
 * - 60 secondes à 5V (haute température)
 * - 90 secondes à 1.4V (basse température pour mesure)
 */

#define MQ7_PIN 34              // Pin analogique ADC
#define RL_VALUE 10             // Résistance de charge en kOhms
#define RO_CLEAN_AIR_FACTOR 27.5 // RS/RO dans l'air pur (27.5 typique pour MQ7)
#define CALIBRATION_SAMPLES 50  // Nombre d'échantillons pour calibration
#define CALIBRATION_INTERVAL 500 // Intervalle entre échantillons (ms)
#define ADC_RESOLUTION 4095.0   // Résolution ADC 12 bits ESP32

float Ro = 10.0;  // Résistance du capteur dans l'air pur (sera calculée)

void setup() {
  Serial.begin(115200);
  pinMode(MQ7_PIN, INPUT);
  
  Serial.println("=================================");
  Serial.println("Étalonnage du capteur MQ7");
  Serial.println("=================================");
  Serial.println();
  
  // Préchauffage du capteur
  Serial.println("Phase 1: Préchauffage du capteur");
  Serial.println("Veuillez patienter 3 minutes...");
  Serial.println("IMPORTANT: Le capteur doit être dans un air propre!");
  
  for(int i = 180; i > 0; i--) {
    Serial.print("Temps restant: ");
    Serial.print(i);
    Serial.println(" secondes");
    delay(1000);
  }
  
  Serial.println();
  Serial.println("Phase 2: Calibration en cours...");
  Serial.println("Collecte de " + String(CALIBRATION_SAMPLES) + " échantillons");
  Serial.println();
  
  // Calibration
  Ro = calibrateMQ7();
  
  Serial.println();
  Serial.println("=================================");
  Serial.println("CALIBRATION TERMINÉE");
  Serial.println("=================================");
  Serial.printf("Valeur Ro calculée: %.2f kOhms\n", Ro);
  Serial.println();
  Serial.println("Copiez cette valeur dans votre code:");
  Serial.printf("float Ro = %.2f;\n", Ro);
  Serial.println();
  Serial.println("=================================");
  Serial.println();
  Serial.println("Démarrage des mesures...");
  Serial.println();
}

void loop() {
  float rs = readRS();
  float ratio = rs / Ro;
  float ppm = calculatePPM(ratio);
  
  Serial.println("--- Lecture capteur MQ7 ---");
  Serial.printf("RS = %.2f kOhms\n", rs);
  Serial.printf("Ratio RS/Ro = %.2f\n", ratio);
  Serial.printf("CO estimé = %.2f ppm\n", ppm);
  Serial.println();
  
  delay(2000);
}

/*
 * Fonction de calibration du capteur
 * Retourne la valeur Ro (résistance dans l'air pur)
 */
float calibrateMQ7() {
  float rsSum = 0;
  
  for(int i = 0; i < CALIBRATION_SAMPLES; i++) {
    float rs = readRS();
    rsSum += rs;
    
    Serial.print("Échantillon ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.print(CALIBRATION_SAMPLES);
    Serial.print(" - RS = ");
    Serial.print(rs);
    Serial.println(" kOhms");
    
    delay(CALIBRATION_INTERVAL);
  }
  
  float rsAvg = rsSum / CALIBRATION_SAMPLES;
  float ro = rsAvg / RO_CLEAN_AIR_FACTOR;
  
  Serial.println();
  Serial.printf("RS moyen: %.2f kOhms\n", rsAvg);
  
  return ro;
}

/*
 * Lecture de la résistance RS du capteur
 */
float readRS() {
  int adcValue = analogRead(MQ7_PIN);
  
  // Conversion ADC vers tension (0-3.3V sur ESP32)
  float voltage = (adcValue / ADC_RESOLUTION) * 3.3;
  
  // Calcul de RS selon: RS = [(Vc x RL) / Vout] - RL
  // Où Vc = 3.3V, RL = résistance de charge
  float rs = ((3.3 * RL_VALUE) / voltage) - RL_VALUE;
  
  return rs;
}

/*
 * Calcul de la concentration de CO en PPM
 * Formule basée sur la courbe caractéristique du MQ7
 * Pour MQ7: log(PPM) = [(log(RS/Ro) - b) / m]
 * Où m ≈ -0.46 et b ≈ 0.42 pour le CO
 */
float calculatePPM(float ratio) {
  // Constantes de la courbe MQ7 pour le CO
  float m = -0.46;  // Pente
  float b = 0.42;   // Ordonnée à l'origine (log scale)
  
  // Calcul du PPM
  float ppm = pow(10, ((log10(ratio) - b) / m));
  
  return ppm;
}

/*
 * Fonction alternative pour lire plusieurs échantillons
 * et calculer une moyenne (optionnel, pour plus de stabilité)
 */
float readRSAverage(int samples) {
  float sum = 0;
  for(int i = 0; i < samples; i++) {
    sum += readRS();
    delay(50);
  }
  return sum / samples;
}

