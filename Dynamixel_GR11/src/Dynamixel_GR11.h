#ifndef DYNAMIXEL_GR11_H
#define DYNAMIXEL_GR11_H

#include <Arduino.h>

class AX12A {
public:
    enum ErrorFlag {
        ERR_NONE        = 0x00, // Aucune erreur
        ERR_VOLTAGE     = 0x01, // Tension d'alimentation hors limites
        ERR_ANGLE_LIMIT = 0x02, // Angle demandé hors des limites CW/CCW
        ERR_OVERHEAT    = 0x04, // Température trop élevée
        ERR_RANGE       = 0x08, // Valeur envoyée hors plage
        ERR_CHECKSUM    = 0x10, // Checksum de la trame envoyée incorrect
        ERR_OVERLOAD    = 0x20, // Surcharge
        ERR_INSTRUCTION = 0x40, // Instruction inconnue
        ERR_COMM        = 0x80  // Pas de réponse / trame invalide (détecté par la bibliothèque)
    };

    AX12A(HardwareSerial &serial, byte id, byte Dir);

    bool begin(long baudrate = 1000000);   // true si le servo répond au Ping
    bool Ping();
    void ActivateTorque();
    void DeactivateTorque();
    bool Move(float angle);                // 0 à 300 degrés
    void SetSpeed(int speed);
    void SetStatusReturnLevel(byte level); // 0, 1 ou 2. Écrit en EEPROM
    bool SetRotationMode(bool wheelMode); // true = mode roue (rotation continue), false = mode articulation.

    int   CurrentStatus();                 // 0, 1 ou 2 ; -1 si erreur
    float CurrentPosition();               // Degrés ; -1 si erreur
    int   CurrentSpeed();                  // Signée : >0 CCW, <0 CW (~0.111 tr/min par unité)
    int   CurrentLoad();                   // Signée : >0 CCW, <0 CW (1023 = 100 % du couple max)
    int   CurrentTemperature();            // °C ; -1 si erreur
    float CurrentVoltage();                // Volts ; -1 si erreur
    int   IsMoving();                      // 1 = en mouvement, 0 = arrêté, -1 si erreur
    int   CurrentRotationMode();           // 0 = joint mode, 1 = wheel mode ; -1 si erreur

    byte GetError();                       // Dernier code d'erreur
    void PrintError(Print &out);

private :
    HardwareSerial *_serial;
    byte _id;
    byte _Dir;
    byte _Statut_Return_Level; // Niveau de réponse supposé du servo (0, 1 ou 2)
    byte _Rotation_Mode;       // 0 = joint mode, 1 = wheel mode
    byte _Error;
    void WritePacket(byte *packet, byte Length);
    bool ReadPacket(byte *response, byte nbParameters);

    void WriteRegister(byte address, int value, byte size);
    int ReadRegister(byte address, byte size);
};
#endif
