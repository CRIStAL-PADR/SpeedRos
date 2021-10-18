/**************************************************************************************
 * Firmware arduino pour contrôler les robots T-quad avec ROS
 * @auteur : Mamadou Sarifou Diallo
 * @email : diallo.msdpro@gmail.com
 * @version : 1.0.0
 **************************************************************************************/

//============ Insertions des bibliothèques ===========================================

// Bibliothèque pour commander les moteurs
#include "Motor.h"

// Biblithèque pour la gestion du capteur ultrason
#include <NewPing.h>

// Bibliothèque pour lire plus rapidement les entrées et sorties digitales
#include <digitalWriteFast.h>

//
#include <EnableInterrupt.h>

// Bibliothèque et message ros
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/time.h>

//============ Définitions et déclarations des moteurs et des encodeurs ================
#define Nmoy 1
#define Nmoy2 10
/*************** Moteur avant gauche ********************************/
Motor moteurGaucheAvant(32,31,46);
#define mesurePWM_MoteurAvantGauche  14
#define mesureCourant_MoteurAvantGauche 9
#define codeurAvantGauchePinA 18
#define codeurAvantGauchePinB 23
volatile int8_t ticksCodeurAvantGauche = 0;
int8_t codeurAvantGaucheDeltaPos;
uint8_t indiceTicksCodeurAvantGauche = 0;
int8_t ticksCodeurAvantGaucheTab[Nmoy2];

/*************** Moteur arriere gauche ******************************/
Motor moteurGaucheArriere(7,34,6);
#define mesurePWM_MoteurArriereGauche  12
#define mesureCourant_MoteurArriereGauche 1
#define codeurArriereGauchePinA 3
#define codeurArriereGauchePinB 9
volatile int8_t ticksCodeurArriereGauche = 0;
int8_t codeurArriereGaucheDeltaPos;
uint8_t indiceTicksCodeurArriereGauche = 0;
int8_t ticksCodeurArriereGaucheTab[Nmoy2];

/*************** Moteur avant droit ********************************/
Motor moteurDroitAvant(33,30,44);
#define mesurePWM_MoteurAvantDroit  15
#define mesureCourant_MoteurAvantDroit 8
#define codeurAvantDroitPinA 19
#define codeurAvantDroitPinB 49
volatile int8_t ticksCodeurAvantDroit = 0;
int8_t codeurAvantDroitDeltaPos;
uint8_t indiceTicksCodeurAvantDroit = 0;
int8_t ticksCodeurAvantDroitTab[Nmoy2];

/*************** Moteur arriere droit ********************************/
Motor moteurDroitArriere(36,4,5);
#define mesurePWM_MoteurArriereDroit  13
#define mesureCourant_MoteurArriereDroit 0
#define codeurArriereDroitPinA 2
#define codeurArriereDroitPinB 8
volatile int8_t ticksCodeurArriereDroit = 0;
int8_t codeurArriereDroitDeltaPos;
uint8_t indiceTicksCodeurArriereDroit = 0;
int8_t ticksCodeurArriereDroitTab[Nmoy2];


//============ Définitions et déclarations du capteurs ultrasons ======================
// Déclarations du capteurs ultrason
#define TRIGGER_PIN  11  // Envoi de l'impulsion ultrason
#define ECHO_PIN     10  // Réception de l'écho
#define MAX_DISTANCE 200 // Distance max en cm
NewPing sonar(TRIGGER_PIN,ECHO_PIN,MAX_DISTANCE);
volatile float sonar_range = 0;


//============ Définitions et déclarations du capteurs de ligne ======================
// Pin des capteurs de ligne
#define left 2
#define middle 3
#define right 4

//============ Définition du pin de la Batterie ======================
#define pin_bat 5

// message ros pour publier les valeurs des capteurs
std_msgs::Float32MultiArray pub_msgs;
char pub_label[]= "serial_pub_values";

// Callback pour le contrôle des moteurs
void subscriber_callback(const std_msgs::Float32MultiArray &value) {
  // Commande des moteurs
  moteurGaucheAvant.run(value.data[0]);
  moteurDroitAvant.run(value.data[1]);
  moteurDroitArriere.run(value.data[2]);
  moteurGaucheArriere.run(value.data[3]);
}

ros::NodeHandle nh;
// Subscriber pour le contrôle des moteurs du Tquad
ros::Subscriber<std_msgs::Float32MultiArray> serial_subscriber("tquad/serial_subscriber", subscriber_callback);
// Publisher pour les valeurs des capteurs du Tquad
ros::Publisher serial_publisher("tquad/serial_publisher", &pub_msgs);

void setup() {
  // Initialisation des moteurs
  init_motors();
  //Initialisation des encodeurs
  init_encoder() ;
  
  setPubArray();

  // Initialisation du handler ros
  nh.getHardware()->setBaud(115200); // Vitesse de communication fixée à 115200 bauds
  nh.initNode();
  
  // Initilisation du publisher
  nh.advertise(serial_publisher);
  // Initialisation du subscriber des moteurs
  nh.subscribe(serial_subscriber);
}

void loop() {
  publisher();
  nh.spinOnce();
  delay(10);
}

float getSense() {
  sonar_range = sonar.ping_cm();
  return sonar_range ;
}
void init_motors() {
  moteurDroitAvant.init();
  moteurDroitArriere.init();
  moteurGaucheAvant.init();
  moteurGaucheArriere.init();
}
void publisher() {
  pub_msgs.data[0]= getSense();
  pub_msgs.data[1] = analogRead(middle);
  pub_msgs.data[2] = analogRead(left);
  pub_msgs.data[3] = analogRead(right);
  pub_msgs.data[4] = getBatteryVoltage();
  pub_msgs.data[5] = ticksCodeurAvantDroit;
  pub_msgs.data[6] = ticksCodeurArriereDroit;
  pub_msgs.data[7] = ticksCodeurAvantGauche;
  pub_msgs.data[8] = ticksCodeurArriereGauche;
  serial_publisher.publish(&pub_msgs);
}

void setPubArray() {
  pub_msgs.layout.dim[0].label = pub_label;
  pub_msgs.layout.dim[0].size = 9;
  pub_msgs.layout.data_offset = 0;
  pub_msgs.data = (float*)malloc(sizeof(float) * 9);
  pub_msgs.data_length = 9;
}

/*************** Fonction pour le voltage de la battery ********************************/
uint16_t getBatteryVoltage() {
  return (int)(1000. * 3. * (5. * (float)analogRead(pin_bat) / 1024.));
}

/*************** Initialisation des encodeurs ********************************/
void init_encoder() {
  // Codeur incrémental moteur arrière droit
  pinMode(codeurArriereDroitPinA, INPUT_PULLUP);
  pinMode(codeurArriereDroitPinB, INPUT_PULLUP);
  enableInterrupt(codeurArriereDroitPinA, GestionInterruptionCodeurArriereDroitPinA, CHANGE);

  // Codeur incrémental moteur arrière gauche
  pinMode(codeurArriereGauchePinA, INPUT_PULLUP);
  pinMode(codeurArriereGauchePinB, INPUT_PULLUP);
  enableInterrupt(codeurArriereGauchePinA, GestionInterruptionCodeurArriereGauchePinA, CHANGE);

  // Codeur incrémental moteur avant droit
  pinMode(codeurAvantDroitPinA, INPUT_PULLUP);
  pinMode(codeurAvantDroitPinB, INPUT_PULLUP);
  enableInterrupt(codeurAvantDroitPinA, GestionInterruptionCodeurAvantDroitPinA, CHANGE);

  // Codeur incrémental moteur avant gauche
  pinMode(codeurAvantGauchePinA, INPUT_PULLUP);
  pinMode(codeurAvantGauchePinB, INPUT_PULLUP);
  enableInterrupt(codeurAvantGauchePinA, GestionInterruptionCodeurAvantGauchePinA, CHANGE);
}

/*************** Routine d'interruption de la voie A de l'encodeur du moteur arrière droit ********************************/
void GestionInterruptionCodeurArriereDroitPinA() {
  if (digitalReadFast2(codeurArriereDroitPinA) == digitalReadFast2(codeurArriereDroitPinB)) {
      ticksCodeurArriereDroit++;
  }
  else {
      ticksCodeurArriereDroit--;
  }
}

/*************** Routine d'interruption de la voie A de l'encodeur du moteur arrière gauche ********************************/
void GestionInterruptionCodeurArriereGauchePinA() {
  if (digitalReadFast2(codeurArriereGauchePinA) == digitalReadFast2(codeurArriereGauchePinB)) {
      ticksCodeurArriereGauche++;
  }
  else {
      ticksCodeurArriereGauche--;
  }
}

/*************** Routine d'interruption de la voie A de l'encodeur du moteur avant droit ********************************/
void GestionInterruptionCodeurAvantDroitPinA() {
  if (digitalReadFast2(codeurAvantDroitPinA) == digitalReadFast2(codeurAvantDroitPinB)) {
      ticksCodeurAvantDroit++;
  }
  else {
      ticksCodeurAvantDroit--;
  }
}

/*************** Routine d'interruption de la voie A de l'encodeur du moteur avant gauche ********************************/
void GestionInterruptionCodeurAvantGauchePinA() {
  if (digitalReadFast2(codeurAvantGauchePinA) == digitalReadFast2(codeurAvantGauchePinB)) {
      ticksCodeurAvantGauche++;
  }
  else {
      ticksCodeurAvantGauche--;
  }
}
