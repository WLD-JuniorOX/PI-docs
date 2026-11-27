#include <Dynamixel_GR11.h>

const byte Dir = 2;     // Broche DIR du 74LS241
const byte Id = 11;     // ID du servomoteur
byte ServoError = 0;
bool wheel_mode = 0;    // 0 = mode servo, 1 = mode continu

AX12A Servo(Serial2, Id, Dir);

// Affiche la position, la température et la tension
void afficherInfos() {
  Serial.print("Current Position : ");
  Serial.print(Servo.CurrentPosition());
  Serial.println("°");
  Serial.print("Current Temperature : ");
  Serial.print(Servo.CurrentTemperature());
  Serial.println("°C");
  Serial.print("Current Voltage : ");
  Serial.print(Servo.CurrentVoltage());
  Serial.println("V");
}

void setup() {
  Serial.begin(115200);
  Servo.begin(1000000);
  if (Servo.Ping()) Serial.println("Dynamixel detected");
  else Serial.println("Error! Dynamixel not detected");
  Servo.ActivateTorque();
  Servo.SetSpeed(512);
  Servo.SetStatusReturnLevel(1);  // Répond uniquement aux lectures
}

void loop() {
  // ----- Mode servo -----
  wheel_mode = 0;
  Servo.SetSpeed(0);                  // Arrêt de la rotation continue
  Servo.SetRotationMode(wheel_mode);
  Servo.SetSpeed(512);                // Vitesse valide en mode servo

  Servo.Move(300);
  delay(2000);
  afficherInfos();

  Servo.Move(100);
  delay(2000);
  afficherInfos();

  // ----- Mode continu -----
  wheel_mode = 1;
  Servo.SetRotationMode(wheel_mode);

  Servo.SetSpeed(1023);               // Sens anti-horraire
  delay(2000);
  Serial.print("Current speed : ");
  Serial.println(Servo.CurrentSpeed());
  delay(2000);

  Servo.SetSpeed(-510);               // Sens horaire
  delay(2000);
  Serial.print("Current speed : ");
  Serial.println(Servo.CurrentSpeed());
  delay(2000);

  // ----- Erreurs -----
  ServoError = Servo.GetError();
  if (ServoError & AX12A::ERR_OVERHEAT) Serial.println("Servo trop chaud");
  Servo.PrintError(Serial);           // ou Affiche toutes les erreurs détectées une fois pour toute directement
  Serial.println("--------------------");
}