#include "Dynamixel_GR11.h"

// #define DYNA_DEBUG

// Codes d'instruction du protocole Dynamixel 1.0
const byte INST_PING  = 0x01;
const byte INST_READ  = 0x02;
const byte INST_WRITE = 0x03;

// Adresses de la table de contrôle de l'AX-12A
const byte CW_Limit_Address            = 0x06; // EEPROM, 2 octets (L puis H)
const byte CCW_Limit_Address           = 0x08; // EEPROM, 2 octets (L puis H)
const byte Status_Return_Address       = 0x10; // EEPROM
const byte Enable_Torque_Address       = 0x18;
const byte Goal_Position_Address       = 0x1E;
const byte Moving_Speed_Address        = 0x20;
const byte Current_Position_Address    = 0x24;
const byte Current_Speed_Address       = 0x26;
const byte Current_Load_Address        = 0x28;
const byte Current_Voltage_Address     = 0x2A;
const byte Current_Temperature_Address = 0x2B;
const byte Moving_Address              = 0x2E;

// Mémorisation du port série et de l'ID du servo
AX12A::AX12A(HardwareSerial &serial, byte id, byte Dir) {
    _serial = &serial;
    _id = id;
    _Dir = Dir;
    _Statut_Return_Level = 2; // Default status return level
    _Rotation_Mode = 0;       // Default rotation mode (joint mode)
    _Error = 0;
}

// Initialisation du port série. Retourne true si le servo répond au Ping.
// Les messages ne sont affichés que si DYNA_DEBUG est défini (Serial.begin() doit alors précéder begin()).
bool AX12A::begin(long baudrate) {
    delay(250);
    _serial->begin(baudrate);
    pinMode(_Dir, OUTPUT);
    digitalWrite(_Dir, LOW);

    _Statut_Return_Level = 2;
    if (!Ping()) {
        #ifdef DYNA_DEBUG
        Serial.println(F("Dynamixel: no servo detected"));
        #endif
        return false; // Aucun servo détecté, on garde les valeurs par défaut
    }

    int status = -1;
    for (byte i = 0; i < 3 && status == -1; i++) {
        status = CurrentStatus();
    }
    // Pas de réponse aux lectures => le servo est au niveau 0
    _Statut_Return_Level = (status == -1) ? 0 : status;
    _Error = 0;
    #ifdef DYNA_DEBUG
    Serial.print(F("Dynamixel: status return level = "));
    Serial.println(_Statut_Return_Level);
    #endif

    // Le mode de rotation ne peut être lu que si le servo répond aux lectures
    if (_Statut_Return_Level != 0) {
        int rotationMode = CurrentRotationMode();
        if (rotationMode != -1) _Rotation_Mode = rotationMode;
        _Error = 0;
        #ifdef DYNA_DEBUG
        if (rotationMode == -1) Serial.println(F("Dynamixel: cannot read rotation mode, assuming joint mode"));
        else Serial.println(_Rotation_Mode == 0 ? F("Dynamixel: joint mode") : F("Dynamixel: wheel mode"));
        #endif
    }
    return true;
}

bool AX12A::Ping() {
    byte packet[5];
    packet[0] = 0xFF;      // Header
    packet[1] = 0xFF;      // Header
    packet[2] = _id;       // ID
    packet[3] = 2;         // Length
    packet[4] = INST_PING; // Instruction
    while (_serial->available()) _serial->read(); // Clear the serial buffer
    WritePacket(packet, 5);
    return ReadPacket(packet, 0);
}

// Mode roue : limites CW et CCW à 0. Mode articulation : CW = 0, CCW = 1023.
// Chaque limite est écrite en une seule trame (octets L et H).
// Renvoie true si le mode demandé est confirmé (ou non vérifiable au niveau 0).
bool AX12A::SetRotationMode(bool wheelMode) {
    bool verifiable = (_Statut_Return_Level != 0);
    // Au niveau 0 on ne peut pas connaître le vrai mode : on écrit toujours
    if (verifiable && _Rotation_Mode == wheelMode) return true;

    WriteRegister(CW_Limit_Address, 0, 2);
    WriteRegister(CCW_Limit_Address, wheelMode ? 0 : 1023, 2);

    if (!verifiable) {
        _Rotation_Mode = wheelMode;
        return true;
    }
    delay(20); // Temps d'écriture EEPROM
    int mode = CurrentRotationMode();
    if (mode == -1) return false; // Le servo ne répond pas : on garde l'ancien mode mémorisé
    _Rotation_Mode = mode;
    return mode == (wheelMode ? 1 : 0);
}

// Mode roue : les deux limites (CW et CCW) valent 0.
int AX12A::CurrentRotationMode() {
    int cwLimit = ReadRegister(CW_Limit_Address, 2);
    if (cwLimit == -1) return -1;
    int ccwLimit = ReadRegister(CCW_Limit_Address, 2);
    if (ccwLimit == -1) return -1;
    return (cwLimit == 0 && ccwLimit == 0) ? 1 : 0; // 1 = wheel mode, 0 = joint mode
}

int AX12A::CurrentStatus() {
    return ReadRegister(Status_Return_Address, 1);
}

void AX12A::ActivateTorque() {
    WriteRegister(Enable_Torque_Address, 1, 1);
}

void AX12A::DeactivateTorque() {
    WriteRegister(Enable_Torque_Address, 0, 1);
}

// En mode roue la position cible est ignorée par le servo : on n'envoie rien.
bool AX12A::Move(float angle) {
    if (_Rotation_Mode) return false;
    angle = constrain(angle, 0.0, 300.0);
    int position = (int)(angle * 1023.0 / 300.0 + 0.5); // Arrondi au plus proche
    WriteRegister(Goal_Position_Address, position, 2);
    return true;
}

void AX12A::SetSpeed(int speed) {
    if (!_Rotation_Mode) {
        // Joint : 0 voudrait dire "vitesse maximale", on impose au moins 1
        speed = constrain(abs(speed), 1, 1023);
    } else {
        // Roue : bit 10 = sens (1024 + valeur = CW), 0 = arrêt
        int value = constrain(abs(speed), 0, 1023);
        speed = (speed < 0) ? (1024 + value) : value;
    }
    WriteRegister(Moving_Speed_Address, speed, 2);
}

// Attention : ce registre est en EEPROM (nombre d'écritures limité).
void AX12A::SetStatusReturnLevel(byte level) {
    level = constrain(level, 0, 2);

    // Déjà au bon niveau : on évite une écriture EEPROM inutile
    if (CurrentStatus() == level) {
        _Statut_Return_Level = level;
        return;
    }

    // Le servo peut changer de comportement dès l'écriture : on n'attend pas
    // de réponse (niveau 1 = pas de réponse aux écritures), puis on vide le port.
    _Statut_Return_Level = 1;
    WriteRegister(Status_Return_Address, level, 1);
    delay(5);
    while (_serial->available()) _serial->read();
    _Statut_Return_Level = level;
}

float AX12A::CurrentPosition() {
    int position = ReadRegister(Current_Position_Address, 2);
    if (position == -1) {
        return -1; // Error reading the position
    }
    return (float)position * 300.0 / 1023.0;
}

// Bit 10 = sens (0 : CCW, 1 : CW). Unité : ~0.111 tr/min.
int AX12A::CurrentSpeed() {
    int raw = ReadRegister(Current_Speed_Address, 2);
    if (raw < 0) return -1; // Erreur : voir GetError() & ERR_COMM
    int value = raw & 0x3FF;
    return (raw & 0x400) ? -value : value;
}

// Bit 10 = sens (0 : CCW, 1 : CW). 1023 = 100 % du couple max.
int AX12A::CurrentLoad() {
    int raw = ReadRegister(Current_Load_Address, 2);
    if (raw < 0) return -1; // Erreur : voir GetError() & ERR_COMM
    int value = raw & 0x3FF;
    return (raw & 0x400) ? -value : value;
}

int AX12A::CurrentTemperature() {
    int temperature = ReadRegister(Current_Temperature_Address, 1);
    return temperature;
}

float AX12A::CurrentVoltage() {
    int raw = ReadRegister(Current_Voltage_Address, 1);
    if (raw == -1) return -1; // Error reading the voltage
    return raw / 10.0;        // Le registre est en dixièmes de volt
}

int AX12A::IsMoving() {
    return ReadRegister(Moving_Address, 1);
}

byte AX12A::GetError() {
    return _Error;
}

void AX12A::WriteRegister(byte address, int value, byte size) {
    _Error = 0;
    if (size > 2) size = 2;   // Au plus 2 octets de données
    byte packet[8];           // 6 + 2 octets de données (le checksum est ajouté dans WritePacket)
    packet[0] = 0xFF;         // Header
    packet[1] = 0xFF;         // Header
    packet[2] = _id;          // ID
    packet[3] = 3 + size;     // Length
    packet[4] = INST_WRITE;   // Instruction
    packet[5] = address;      // Address
    for (int i = 0; i < size; i++) {
        packet[6 + i] = ((value >> (8 * i)) & 0xFF);
    }
    while (_serial->available()) _serial->read(); // Clear the serial buffer
    WritePacket(packet, 6 + size);
    if (_Statut_Return_Level == 2) {
        byte response[4];
        ReadPacket(response, 0); // Met _Error à jour (ERR_COMM si pas de réponse)
    }
}

void AX12A::WritePacket(byte *packet, byte Length) {
    byte somme = 0;
    for (int i = 2; i < Length; i++) {
        somme += packet[i];
    }
    byte checksum = ~somme;
    #ifdef DYNA_DEBUG
    Serial.print("Sending packet: ");
    for (int i = 0; i < Length; i++) {
        Serial.print(packet[i], HEX);
        Serial.print(" ");
    }
    Serial.println(checksum, HEX);
    #endif
    digitalWrite(_Dir, HIGH); // Signal Haut pour l'envoi
    for (int i = 0; i < Length; i++) {
        _serial->write(packet[i]);
    }
    _serial->write(checksum);
    _serial->flush();
    digitalWrite(_Dir, LOW); // Signal Bas pour la réception
}

int AX12A::ReadRegister(byte address, byte size) {
    if (_Statut_Return_Level == 0) {
        _Error = ERR_COMM;
        return -1; // No status return, cannot read
    }
    _Error = 0;
    if (size > 2) size = 2;   // Au plus 2 octets de données
    byte packet[7];
    packet[0] = 0xFF;         // Header
    packet[1] = 0xFF;         // Header
    packet[2] = _id;          // ID
    packet[3] = 4;            // Length
    packet[4] = INST_READ;    // Instruction
    packet[5] = address;      // Address
    packet[6] = size;         // Size
    while (_serial->available()) _serial->read(); // Clear the serial buffer
    WritePacket(packet, 7);
    byte response[6];         // 4 + 2 octets de données
    if (ReadPacket(response, size)) {
        int value = 0;
        for (int i = 0; i < size; i++) {
            value |= ((int)response[3 + i]) << (8 * i);
        }
        return value;
    }
    else return -1; // Error reading the response (_Error = ERR_COMM)
}

// Lit un paquet de statut : [ID, Length, Error, params..., Checksum]
// _Error vaut ERR_COMM tant qu'aucune trame valide n'a été reçue.
bool AX12A::ReadPacket(byte *response, byte nbParameters) {
    _Error = ERR_COMM;
    const byte n = 4 + nbParameters;
    byte FF_Count = 0;
    unsigned long t0 = millis();
    while (millis() - t0 < 10) {
        if (!_serial->available()) continue;
        byte Octet = _serial->read();
        if (Octet != 0xFF) {
            FF_Count = 0;
            continue;
        }
        if (++FF_Count < 2) continue;

        // Les deux 0xFF du header sont reçus : lecture du reste de la trame
        for (byte i = 0; i < n; i++) {
            unsigned long t1 = millis();
            while (!_serial->available()) {
                if (millis() - t1 > 10) return false; // Timeout
            }
            response[i] = _serial->read();
        }
        #ifdef DYNA_DEBUG
        Serial.print("Received packet: ");
        for (int j = 0; j < n; j++) {
            Serial.print(response[j], HEX);
            Serial.print(" ");
        }
        Serial.println();
        #endif
        if (response[0] != _id) return false;
        if (response[1] != 2 + nbParameters) return false;
        byte somme = 0;
        for (byte i = 0; i < 3 + nbParameters; i++) {
            somme += response[i];
        }
        byte checksum = ~somme;
        if (checksum != response[3 + nbParameters]) return false;
        _Error = response[2]; // Octet d'erreur renvoyé par le servo
        return true;
    }
    return false;
}

void AX12A::PrintError(Print &out) {
    if (_Error == ERR_NONE) {
        out.println(F("Aucune erreur"));
        return;
    }
    if (_Error & ERR_VOLTAGE)     out.println(F("Tension hors limites"));
    if (_Error & ERR_ANGLE_LIMIT) out.println(F("Angle hors limites"));
    if (_Error & ERR_OVERHEAT)    out.println(F("Surchauffe"));
    if (_Error & ERR_RANGE)       out.println(F("Valeur hors plage"));
    if (_Error & ERR_CHECKSUM)    out.println(F("Checksum errone"));
    if (_Error & ERR_OVERLOAD)    out.println(F("Surcharge"));
    if (_Error & ERR_INSTRUCTION) out.println(F("Instruction invalide"));
    if (_Error & ERR_COMM)        out.println(F("Pas de reponse / trame invalide"));
}