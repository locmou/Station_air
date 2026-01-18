#include <Arduino.h>

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
  
