#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_ADS1X15.h>
#include <EEPROM.h>



// information a modifier

const char* name_machine = "BRM";
const char* ID = "CateurCourantBRM";
const char* topic = "CLICK/FabLab/Laser/BRM/timer";



// 'Image1', 128x32px

const unsigned char epd_bitmap_Image1 [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xf1, 0xf8, 0x00, 0x7e, 0x00, 0x03, 0xc3, 0xff, 0xff, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0xf1, 0xf8, 0x00, 0x7e, 0x00, 0x1f, 0xc3, 0xff, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xf1, 0xf8, 0x00, 0x7e, 0x00, 0x7f, 0xc3, 0xff, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xf1, 0xf8, 0x00, 0x7e, 0x00, 0xff, 0xc3, 0xff, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xff, 0xf1, 0xf8, 0x00, 0x7e, 0x01, 0xff, 0xc3, 0xff, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xff, 0xe1, 0xf8, 0x00, 0x7e, 0x03, 0xff, 0xc3, 0xff, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xff, 0x01, 0xf8, 0x00, 0x7e, 0x07, 0xfe, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xfc, 0x01, 0xf8, 0x00, 0x7e, 0x07, 0xf8, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf0, 0x01, 0xf8, 0x00, 0x7e, 0x0f, 0xf0, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf0, 0x01, 0xf8, 0x00, 0x7e, 0x0f, 0xe0, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xe0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0xc0, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0xc0, 0x03, 0xfe, 0x03, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0x80, 0x03, 0xfc, 0x07, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0x80, 0x03, 0xf8, 0x07, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x1f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0x80, 0x03, 0xf8, 0x0f, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0x80, 0x03, 0xf8, 0x07, 0xe0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0x80, 0x03, 0xfc, 0x07, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xc0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0xc0, 0x03, 0xfe, 0x03, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x0f, 0xe0, 0x01, 0xf8, 0x00, 0x7e, 0x1f, 0xc0, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf0, 0x01, 0xf8, 0x00, 0x7e, 0x0f, 0xe0, 0x03, 0xff, 0x80, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xf0, 0x01, 0xf8, 0x00, 0x7e, 0x0f, 0xf0, 0x03, 0xff, 0xc0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x07, 0xfc, 0x01, 0xf8, 0x00, 0x7e, 0x07, 0xf8, 0x03, 0xff, 0xe0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x03, 0xff, 0x01, 0xfc, 0x00, 0x7e, 0x07, 0xfc, 0x03, 0xff, 0xf0, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x01, 0xff, 0xe1, 0xff, 0xfc, 0x7e, 0x03, 0xff, 0xc3, 0xff, 0xf8, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0xff, 0xf1, 0xff, 0xfc, 0x7e, 0x01, 0xff, 0xc3, 0xff, 0xfc, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x7f, 0xf1, 0xff, 0xfc, 0x7e, 0x00, 0xff, 0xc3, 0xff, 0xfe, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x3f, 0xf1, 0xff, 0xfc, 0x7e, 0x00, 0x7f, 0xc3, 0xff, 0xff, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x0f, 0xf1, 0xff, 0xfc, 0x7e, 0x00, 0x1f, 0xc3, 0xff, 0xff, 0x80, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x03, 0xf1, 0xff, 0xfc, 0x7e, 0x00, 0x07, 0xc3, 0xff, 0xff, 0xc0, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Array of all bitmaps for convenience. (Total bytes used to store images in PROGMEM = 528)
const int epd_bitmap_allArray_LEN = 1;
const unsigned char* epd_bitmap_allArray[1] = {
	epd_bitmap_Image1
};





// Parametres MQTT
const char* ssid = "FabLab";
const char* password = "FabLab2023";
const char* mqtt_server = "192.168.1.102";
const int mqtt_port = 1883;

// Moyenne glissante (buffer) existant
const int bufferSize = 100;  
float buffer[bufferSize];
int bufferIndex = 0;
bool bufferFull = false;

// Ajout filtre EMA (MME)
const float alpha = 0.2;//2.0 / (bufferSize + 1);  // coefficient EMA
float ema_power = 0.0;


// Variable pre calculee
float racine_de_2 = 1.41421356;


// Affichage des informations
long time_show_current = 350;
long time_last_show_current = -1;


// Pin
const int led_bleu_pin = 33;
const int led_verte_pin = 32;
const int led_rouge_pin = 27;
const int button_pin_gauche = 34;
const int button_pin_droite = 35;


//timer interuptible
long last_time = 0; // time after reset but before new timer
bool state_machine = 0;
bool last_state_machine = 0;
long start_current_timer = 0; //millis de debut du timer actuel
long timing_current = 0; //last time + millis - start_current_timer
bool machine_start_from_reset = 0;


// Communication MQTT
WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;
long time_lastsend = 0;
const long time_of_send = 100;
bool callback_value = 0;
char last_string[7] = {'H', 'e', 'l', 'l', 'o', '1','\0'};


// Filtre Kalman
float kalman_estimate = 0;
float kalman_error_estimate = 1;
const float kalman_q = 0.01;   // bruit du processus
const float kalman_r = 7.0;    // bruit de la mesure


// Declaration des fonctions
void setup_wifi();
void reconnect();
float get_timer();
char* get_timer_hh_mm();
void callback(char* topic, byte* payload, unsigned int length);
long get_full_timer();
float mesure_courant_eff();

float get_power_consumption ();
float get_treshold(float min, float maxi);
void gestion (bool button_gauche_a, bool button_droite_a, float current_mode);

void led_startup();
void is_machin_running();

// I2c
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);
Adafruit_ADS1015 ads;


// bouton
bool button_gauche_push = 0;
bool button_droite_push = 0;
volatile bool button_gauche_interrupt = 0;
volatile bool button_droite_interrupt = 0;
long dernier_appui_boutton_gauche = 0;
long dernier_appui_boutton_droite = 0;
const long time_antibounce = 300;


// interuptions
void IRAM_ATTR interruptionBoutongauche() {
  if(millis()-dernier_appui_boutton_gauche > time_antibounce){
    button_gauche_interrupt = true;
    dernier_appui_boutton_gauche = millis();
  }
}

void IRAM_ATTR interruptionBoutondroite() {
  if(millis()-dernier_appui_boutton_droite > time_antibounce){
    button_droite_interrupt = true;
    dernier_appui_boutton_droite = millis();
  }
}


//gestion du mode du systeme
int state_code = 0;
int state_code_pre = 25640;


//valeurs de detection
float power_veille;
float power_marche;
float treshold = -1;


//mise a jour des valeurs de detection
int moyenne_faible = 0;
int compteur_moyenne_faible = 0;
int moyenne_fonctionnement = 0;
int compteur_moyenne_fonctionnement = 0;
float valeur_veille = -1;
float valeur_fonctionnement = -1;

long time_setup_wifi;


void setup() {
  // Configure les pins
  pinMode(led_rouge_pin, OUTPUT);
  pinMode(led_verte_pin, OUTPUT);
  pinMode(led_bleu_pin, OUTPUT);
  pinMode(button_pin_gauche, INPUT_PULLUP);
  pinMode(button_pin_droite, INPUT_PULLUP);
  
  // Creation des intruptions
  attachInterrupt(digitalPinToInterrupt(button_pin_gauche), interruptionBoutongauche, FALLING);
  attachInterrupt(digitalPinToInterrupt(button_pin_droite), interruptionBoutondroite, FALLING);

  // Lancement de differentes choses
  EEPROM.begin(512);
  Wire.begin(21, 22); // Set your SDA and SCL pins here if different
  Serial.begin(115200);
  delay(200);

  //Recuperation des valeurs de la mémoires
  power_veille = EEPROM.readFloat(0);
  power_marche = EEPROM.readFloat(4);
  treshold = get_treshold(power_veille, power_marche);
  Serial.println(treshold);
  

  //display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.drawBitmap(0, 0, epd_bitmap_Image1, 128, 32, WHITE);
  display.display();


  delay(3000);

  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  //mqtt et wifi
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  // ADS setup
  ads.setGain(GAIN_FOUR);  // ±1.024V = 0.5mV/bit
  ads.begin();

  delay(1500);

  display.clearDisplay();
  display.display();
  display.setTextSize(1);
  display.setTextColor(WHITE);
}


void loop() {
  // MQTT
  if (!client.connected()) {
    reconnect();
  }

  client.loop();


  // Lecture des boutons, afin de les lires une fois et de rerendre la valeur interuption libre
  if (button_droite_interrupt){
    button_droite_push = 1;
    button_droite_interrupt = 0;
  }

  if (button_gauche_interrupt){
    button_gauche_push = 1;
    button_gauche_interrupt = 0;
  }


  // mesure brute
  float value = mesure_courant_eff();

  // Met à jour EMA (MME)
  ema_power = alpha * value + (1 - alpha) * ema_power;

  // met a jour le buffer existant
  buffer[bufferIndex] = value; 
  bufferIndex = (bufferIndex + 1) % bufferSize;  
  if (bufferIndex == 0) bufferFull = true; 

  // vérifie si la machine est en fonctionnement en utilisant MME
  is_machin_running();


  // met a jour le timer en fonction de l'etat machine
  get_full_timer();

  // gere le système (affichage, communication, etc.)
  gestion(button_gauche_push, button_droite_push, value);

  //Remise des bouton à zero pour la boucle suivante
  button_gauche_push = 0;
  button_droite_push = 0;

  delay(10);  
  
}


// put function definitions here:

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  display.println();
  display.println("Connecting to ");
  display.println(ssid);
  display.display();

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println(WiFi.localIP());

}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(ID)) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(topic);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}


void callback(char* topic, byte* payload, unsigned int length){
  // Convertir payload en chaîne de caractères C
  callback_value = 1;
  char message[length + 1]; // +1 pour le caractère nul
  memcpy(message, payload, length);
  message[length] = '\0'; // Ajouter le caractère de fin de chaîne
  if (strcmp(message, "reset") == 0) {
    Serial.println("Commande RESET reçue !");
    last_time = 0;
    start_current_timer = millis();
  }
}


char* get_timer_hh_mm(){
  int timer = int(get_timer());
  int minutes = timer / 60;
  int secondes = timer - minutes * 60;
  static char result[7];
  if (minutes > 99){
    snprintf(result, sizeof(result), "%03d:%02d", minutes, secondes);
    return result;
  }

  else 
  {
    snprintf(result, sizeof(result), "%02d:%02d", minutes, secondes);
    return result;
  }
}


float get_timer(){
  long timer = 0;
  timer = timing_current;
  float secondes = (float)timer /1000;
  return secondes;
}


int actual_consuption(){
  int voltage = 0;
  return voltage;
}


float kalman_filter(float measurement) {
  // Prediction
  kalman_error_estimate += kalman_q;

  // Kalman gain
  float kalman_k = kalman_error_estimate / (kalman_error_estimate + kalman_r);

  // Update
  kalman_estimate = kalman_estimate + kalman_k * (measurement - kalman_estimate);
  kalman_error_estimate = (1 - kalman_k) * kalman_error_estimate;

  return kalman_estimate;
}


float get_power_consumption() {
  float current = ema_power * 20 / racine_de_2;
  float puissance = current * 230;
  
  // Application du filtre Kalman sur la puissance
  return kalman_filter(puissance);
}



void is_machin_running(){
  last_state_machine = state_machine;
  float current_machine = get_power_consumption();
  //Serial.println(puissance);Serial.println(current_machine);

  if (current_machine > treshold){
    state_machine = 1;
  }

  else {
    state_machine = 0;
  }
}


float get_treshold(float min, float maxi){
  return (min + (maxi-min) *0.65);
}


long get_full_timer(){
  if (machine_start_from_reset == 0){
    timing_current = 0;
  }
  if(state_machine == 0 && last_state_machine == 1){
    last_time = last_time + millis() - start_current_timer; 
    timing_current = last_time;
    return timing_current;
  }
  else if(state_machine == 1 && last_state_machine == 0){
    start_current_timer = millis(); 
    machine_start_from_reset = 1;
    return timing_current;
  }
  else if( state_machine == 1){
    timing_current = last_time + millis() - start_current_timer; 
    return timing_current;
  }
  else if( state_machine == 0){
    timing_current = last_time; 
    return timing_current;
  }
  long t = 0;
  return t;
}


float mesure_courant_eff(){ // return tension RMS
  float sommeCarre = 0;
  int16_t val = 0;
  float value = 0;

  for (int i = 0; i < 16; i++){
    val = ads.readADC_Differential_0_3();
    float value = (long)val * (long)val;
    sommeCarre += value;
    Serial.println(value); 
  }
  float meanCarre = sommeCarre / 16.0;

  float rms = sqrt(meanCarre);

  // Convertir la valeur ADC rms en tension
  float tension = rms * 0.0005; 

  return tension;
}


void gestion (bool button_gauche_a, bool button_droite_a, float current_mode){
  int button_gauche = button_gauche_a;
  int button_droite =  button_droite_a;
  //Serial.println(current_mode);
  float current = current_mode * 20 / racine_de_2;
  float puissance = current * 230;


  if (state_code == 0){
    if (millis() - time_lastsend > time_of_send){
      if (callback_value){
        callback_value = 0;
        return;
      }
      char* result = get_timer_hh_mm();
      if (strcmp(result, last_string) != 0){
        client.publish(topic, result);
        strncpy(last_string, result, sizeof(last_string));
        last_string[sizeof(last_string) - 1] = '\0';
        time_lastsend = millis();
      }
    }
    if (state_code != state_code_pre)  {
        display.setCursor(0, 0);
        display.clearDisplay();
        display.setCursor(0, 0);
        display.print("Machine : ");
        display.println(name_machine);
        display.print("Compteur : ");
        display.print(get_timer_hh_mm());
        display.display();

    }
    if (millis() - time_last_show_current > time_show_current){
      display.fillRect(0, 8, 128, 16, BLACK); 
      display.setCursor(0, 16);  // Ligne 3 (16 pixels), premier caractère (x = 0)
      display.print("Consommation : ");
      display.print(get_power_consumption());
      display.setCursor(0, 8);
      display.print("Compteur : ");
      display.print(get_timer_hh_mm());
      display.display(); 
    }
    if (button_gauche == 1){
      state_code = 10;
      button_gauche = 0;
    } 
    if (button_droite == 1){
      state_code = 30;
      button_droite = 0;
    }
  }

  state_code_pre = state_code;
  if (state_code == 10){
    moyenne_faible += puissance;
    compteur_moyenne_faible += 1;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Mesure en veille :");
    float v_display = moyenne_faible / compteur_moyenne_faible;
    display.println("Valeur actuelle: ");
    display.println(v_display);
    display.println("Valider  <->  Quitter");
    
    display.display();

    if (button_gauche == 1){
      state_code = 16;
      button_gauche = 0;
      valeur_veille = v_display;
      compteur_moyenne_faible = 0;
      moyenne_faible = 0;
    }
    if (button_droite == 1){
      state_code = 20;
      button_droite =0;
      compteur_moyenne_faible = 0;
      moyenne_faible = 0;
    }
  }

  if (state_code == 11){
    moyenne_fonctionnement += puissance;
    compteur_moyenne_fonctionnement += 1;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Mesure en marche :");
    float v_display = moyenne_fonctionnement / compteur_moyenne_fonctionnement;
    display.println("Valeur actuelle: ");
    display.println(v_display);
    display.println("Valider  <->  Quitter");
    display.display();

 
    if (button_gauche == 1){
      
      state_code = 15;
      button_gauche = 0;
      valeur_fonctionnement = v_display;
      compteur_moyenne_fonctionnement = 0;
      moyenne_fonctionnement = 0;
    }
    if (button_droite == 1){
      state_code = 20;
      button_droite =0;
      compteur_moyenne_fonctionnement = 0;
      moyenne_fonctionnement = 0;
    }
  }

  if(state_code == 16){
    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("Mettre en marche ");
    display.println("la machine ");
    display.println(" ");
    display.println("Valider  <->  Quitter");
    display.display();

    if (button_gauche == 1){
      state_code = 11;
      button_gauche =0;
    }
    if (button_droite == 1){
      state_code = 0;
      button_droite =0;
    }
  }

  if(state_code == 15){
    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("Validation ");
    display.print("Valeur veille: ");
    display.println(valeur_veille);
    display.print("Valeur marche: ");
    display.println(valeur_fonctionnement);
    display.println("Valider  <->  Quitter");
    display.display();

    if (button_gauche == 1){
      state_code =12;
      button_gauche =0;
    }
    if (button_droite == 1){
      state_code = 0;
      button_droite =0;
    }
  }

  if(state_code == 12){
    display.clearDisplay();
    display.setCursor(0, 0);
    if (valeur_veille  != -1 && valeur_fonctionnement != -1){
      Serial.println(valeur_veille);
      Serial.println(valeur_fonctionnement);
      EEPROM.writeFloat(0, valeur_veille);
      EEPROM.writeFloat(4, valeur_fonctionnement); 
      EEPROM.commit();

      power_marche = valeur_fonctionnement;
      power_veille = valeur_veille;
      treshold = get_treshold (power_veille, power_marche);

      valeur_veille = -1;
      valeur_fonctionnement = -1;
      state_code = 13;
    }
    else{
      state_code = 0;
    }
  }

  if(state_code == 13){
    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("Valeurs enregistrees");
    display.print("Valeur veille: ");
    float veille;
    EEPROM.get(0, veille);
    display.println(veille);
    display.print("Valeur marche: ");
    float fonctionnement;
    EEPROM.get(4, fonctionnement);
    display.println(fonctionnement);
    display.display();

    if (button_gauche == 1){
      state_code = 0;
      button_gauche =0;
    }
    if (button_droite == 1){
      state_code = 0;
      button_droite =0;
    }
  }

  if (state_code == 20){
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Mise a jour annulee");
    display.println("Je vous souhaite");
    display.println("une bonne journee");
    display.println("!!!! :)");
    display.display();
    if (button_gauche == 1){
      button_gauche = 0;
      state_code = 0;
    }
    if (button_droite == 1){
      state_code = 0;
    }
  }

  if (state_code == 30){
    moyenne_faible += puissance;
    compteur_moyenne_faible += 1;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Valeurs generales");
    display.print("Conso veille : ");
    display.println(power_veille);
    display.print("Conso marche : ");
    display.println(power_marche);
    display.print("Activation a : ");
    display.print(treshold);
    display.display();

    if (button_gauche == 1){
      state_code = 0;
      button_gauche = 0;
      moyenne_faible = 0;
    }
    if (button_droite == 1){
      state_code = 0;
      moyenne_faible = 0;
      button_droite = 0;
    }

  } 
}

void led_startup(){

  // Allume les LEDs test
  // Aller
  digitalWrite(led_rouge_pin, HIGH);
  delay(200);
  digitalWrite(led_rouge_pin, LOW);
  delay(200);

  digitalWrite(led_bleu_pin, HIGH);
  delay(200);
  digitalWrite(led_bleu_pin, LOW);
  delay(200);

  digitalWrite(led_verte_pin, HIGH);
  delay(200);
  digitalWrite(led_verte_pin, LOW);
  delay(200);

  // Retour
  digitalWrite(led_bleu_pin, HIGH);
  delay(200);
  digitalWrite(led_bleu_pin, LOW);
  delay(200);

  digitalWrite(led_rouge_pin, HIGH);
  delay(200);
  digitalWrite(led_rouge_pin, LOW);
  delay(200);
}