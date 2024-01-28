#define LIGHT_SENSOR_PIN A0

#define PROX_SENSOR_L_PIN A1
#define PROX_SENSOR_R_PIN A2
#define PROX_SENSOR_FL_PIN A3
#define PROX_SENSOR_FR_PIN A4
#define PROX_SENSOR_RL_PIN A5
#define PROX_SENSOR_RR_PIN 12
#define PROX_SENSOR_DL_PIN 6
#define PROX_SENSOR_DR_PIN 9

#define MOTOR_RF_PIN 2
#define MOTOR_RB_PIN 4
#define MOTOR_R_SPEED 3  // entre 0 et 255
#define MOTOR_LF_PIN 7
#define MOTOR_LB_PIN 8
#define MOTOR_L_SPEED 5  // entre 0 et 255


// Définir vitesse robot
int const vitesse = 125;
// Définir une marge pour ajuster la trajectoire en fonction des valeurs des capteurs
int marge = 700;
// Définir les valeurs des capteurs de proximité
  float val_sensorL;
  float val_sensorR;
  float val_sensorFL;
  float val_sensorFR;
// Définir couleurs
enum Couleur {
  Noir,
  Rouge,
  Blanc
};
//initialisation couleur 
Couleur couleurActuelle = Rouge; 
float val_couleur;
// je suis sur du noir ? 
bool estNoir=0;
// Définir timers
unsigned long timerN;
unsigned long timerR;

// états du parcours
bool estDansCaseNoire;
bool estDansCaseRouge;
bool noireTrouvee=false;
bool sortieCaseDepart=false;
bool rougeTrouvee=false;

void hardware_setup() {
  new DCMotor_Hbridge(MOTOR_RF_PIN, MOTOR_RB_PIN, MOTOR_R_SPEED, "ePuck_rightJoint", 2.5, 3 * 3.14159, 1);
  new DCMotor_Hbridge(MOTOR_LF_PIN, MOTOR_LB_PIN, MOTOR_L_SPEED, "ePuck_leftJoint", 2.5, 3 * 3.14159, 1);

  new VisionSensor(LIGHT_SENSOR_PIN, "ePuck_lightSensor", 0.1);

  new ProximitySensor(PROX_SENSOR_FL_PIN, "ePuck_proxSensor3", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_FR_PIN, "ePuck_proxSensor4", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_L_PIN, "ePuck_proxSensor1", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_R_PIN, "ePuck_proxSensor6", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RL_PIN, "ePuck_proxSensor7", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_RR_PIN, "ePuck_proxSensor8", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DL_PIN, "ePuck_proxSensor2", 0.1, 1);
  new ProximitySensor(PROX_SENSOR_DR_PIN, "ePuck_proxSensor5", 0.1, 1);
}

void setup() {
  Serial.begin(4800);

  pinMode(MOTOR_RF_PIN, OUTPUT);
  pinMode(MOTOR_RB_PIN, OUTPUT);
  pinMode(MOTOR_R_SPEED, OUTPUT);
  pinMode(MOTOR_LF_PIN, OUTPUT);
  pinMode(MOTOR_LB_PIN, OUTPUT);
  pinMode(MOTOR_L_SPEED, OUTPUT);

  // Set speed to max
  analogWrite(MOTOR_R_SPEED, vitesse);
  analogWrite(MOTOR_L_SPEED, vitesse);

  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);

}

void loop() {
  
  // j'appelle les fonctions permettant de détecter les murs et les couleurs
  readSensorsValues();
  readColors();
  
  // premier parcours : je longe les murs d'enceinte, si je trouve la case noire alors demi tour, fonctionne très bien
  parcour1();

  // si jamais 'ai fait le tour sans trouver la case noire, alors trajectoire aléatoire, fonctionne selon le labirynthe
  if (rougeTrouvee==true && noireTrouvee==false){
    parcour2();
  }

}

// fonctions directionnelles
void forward(){
  analogWrite(MOTOR_R_SPEED, vitesse);
  analogWrite(MOTOR_L_SPEED, vitesse);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  Serial.println("Tout droit");
}
void turnLeft(){
  analogWrite(MOTOR_R_SPEED, vitesse);
  analogWrite(MOTOR_L_SPEED, vitesse * 0.5);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  Serial.println("Tourne à gauche");
}
void turnRight(){
  analogWrite(MOTOR_R_SPEED, vitesse * 0.5);
  analogWrite(MOTOR_L_SPEED, vitesse);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  Serial.println("Tourne à droite");
}
void turnRightOnPlace(){
  analogWrite(MOTOR_R_SPEED, vitesse * 0.5);
  analogWrite(MOTOR_L_SPEED, vitesse * 0.5);
  digitalWrite(MOTOR_RF_PIN, LOW);
  digitalWrite(MOTOR_RB_PIN, HIGH);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  Serial.println("Tourne à droite");
}
void turnLeftOnPlace(){
  analogWrite(MOTOR_R_SPEED, vitesse * 0.5);
  analogWrite(MOTOR_L_SPEED, vitesse * 0.5);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, LOW);
  digitalWrite(MOTOR_LB_PIN, HIGH);
  Serial.println("Tourne à gauche");
}
void stop(){
  analogWrite(MOTOR_R_SPEED, 0);
  analogWrite(MOTOR_L_SPEED, 0);
  digitalWrite(MOTOR_RF_PIN, HIGH);
  digitalWrite(MOTOR_RB_PIN, LOW);
  digitalWrite(MOTOR_LF_PIN, HIGH);
  digitalWrite(MOTOR_LB_PIN, LOW);
  Serial.println("Stop");
}


// Lecture capteurs
void readSensorsValues(){
  val_sensorL = analogRead(PROX_SENSOR_L_PIN);
  val_sensorR = analogRead(PROX_SENSOR_R_PIN);
  val_sensorFL = analogRead(PROX_SENSOR_FL_PIN);
  val_sensorFR = analogRead(PROX_SENSOR_FR_PIN);
  Serial.print("val_sensorL = ");
  Serial.println(val_sensorL);
  Serial.print("val_sensorR = ");
  Serial.println(val_sensorR);
  Serial.print("val_sensorFL = ");
  Serial.println(val_sensorFL);
  Serial.print("val_sensorFR = ");
  Serial.println(val_sensorFR);
  Serial.print("marge = ");
  Serial.println(marge);
}
void readColors(){
  val_couleur = analogRead(LIGHT_SENSOR_PIN);

    if (val_couleur <= 150)
    {
      couleurActuelle=Noir;
      Serial.println("couleur : Noir ");
    }
   
    if (val_couleur >= 250 && val_couleur <= 300)
    {
      couleurActuelle=Rouge;
      Serial.println("couleur : Rouge ");
    }
   
    if (val_couleur >= 1000 && val_couleur <= 1023)
    {
      couleurActuelle=Blanc;
      Serial.println("couleur : Blanc ");
    }
}

void caseNoire(){
  
  if (couleurActuelle==Blanc){
    estDansCaseNoire=false;
  }
  else if (couleurActuelle==Noir){
    if (estDansCaseNoire==false){
      timerN=millis();
      estDansCaseNoire = true;
    }
    if (millis()-timerN>1500){
      stop();
      if (noireTrouvee==false){
        Serial.println("Je fais demi-tour");
        while (val_sensorR >= marge){
          
          turnLeftOnPlace();
          val_sensorR = analogRead(PROX_SENSOR_R_PIN);
        }
      }
      noireTrouvee=true;
    }
  }
}

// Protocole longement des murs d'enceinte
void parcour1(){  
  caseNoire();
  if(couleurActuelle== !Rouge){
    sortieCaseDepart = true;
  }

  if(sortieCaseDepart == false){
    suiviContour();
  }

  else if(sortieCaseDepart==true){
    if (couleurActuelle==Blanc || couleurActuelle==Noir){
      estDansCaseRouge=false;
      suiviContour();
    }
    else if (couleurActuelle==Rouge){
      if (estDansCaseRouge==false){
        timerR=millis();
        estDansCaseRouge = true;
      }
      if (millis()-timerR>1000){
        stop();
        rougeTrouvee=true;
      }
    }
  }
}

void suiviContour(){
  if (noireTrouvee==false){
    // Si les capteurs avant gauche, avant droit et gauche détectent un mur, tourner à droite
    if ((val_sensorFL < marge) && ((val_sensorFR < marge) || (val_sensorL < marge))) {
      turnRightOnPlace();
    }
    // Si le capteur gauche détecte un mur
    else if (val_sensorL < marge) {
      // Si le capteur avant gauche ou le capteur avant droit détectent un mur, tourner à droite
      if ((val_sensorFL < marge) || (val_sensorFR < marge)) {
        turnRight();
      }
      // Sinon, aller tout droit
      else {
        forward();
      }
    }
    // Si le capteur gauche ne détecte pas de mur
    else {
      
      // Si ni le capteur avant gauche ni le capteur avant droit ne détectent un mur, tourner à gauche
      if ((val_sensorFL > marge) && (val_sensorFR > marge)) {
        turnLeft();
      }
      // Sinon, aller tout droit
      else {
        forward();
      }
    }
  }
    
    
  //case noire trouvée ? je suis maintenant les murs droits
  else {
    // Si les capteurs avant gauche, avant droit et droit détectent un mur, tourner à gauche
    if ((val_sensorFL < marge) && ((val_sensorFR < marge) || (val_sensorR < marge))) {
      turnLeftOnPlace();
    } 
    // Si le capteur droit détecte un mur
    else if (val_sensorR < marge) {
      // Si le capteur avant gauche ou le capteur avant droit détectent un mur, tourner à gauche
      if ((val_sensorFL < marge) || (val_sensorFR < marge)) {
        turnLeft();
      }
      // Sinon, aller tout droit
      else {
        forward();
      }
    }
    // Si le capteur droit ne détecte pas de mur
    else {
      // Si ni le capteur avant gauche ni le capteur avant droit ne détectent un mur, tourner à droite
      if ((val_sensorFL > marge) && (val_sensorFR > marge)) {
        turnRight();
      }
      // Sinon, aller tout droit
      else {
        forward();
      }
    }
      
    
  }
}

// Parcours aléatoire, on change très légerement les instructions précédentes
void crazyrobot(){
  if ((val_sensorFL < marge) && ((val_sensorFR < marge) || (val_sensorL < marge))) {
    turnRightOnPlace();
  }
  else if (val_sensorL < marge) {
    if ((val_sensorFL < marge) || (val_sensorFR < marge)) {
      turnRight();
     }
    else {
      forward();
    }
  }
  else {
    if ((val_sensorFL > marge) && (val_sensorFR > marge)) {
      forward();
    }
  }
}


void parcour2(){
  caseNoire();
  if (noireTrouvee==false){
    crazyrobot();
  }
  else{
    if(couleurActuelle== !Rouge){
      sortieCaseDepart = true;
    }

    if(sortieCaseDepart == false){
      crazyrobot();
    }

    else if(sortieCaseDepart==true){
      if (couleurActuelle==Blanc || couleurActuelle==Noir){
        estDansCaseRouge=false;
        crazyrobot();
      }
      else if (couleurActuelle==Rouge){
        if (estDansCaseRouge==false){
          timerR=millis();
          estDansCaseRouge = true;
        }
        if (millis()-timerR>1000){
          stop();
          rougeTrouvee=true;
        }
      }
    }
  }
}

    