#include <iostream>
#include <cstring>
#include <cmath>
#include <thread>
#include <chrono>
#include "ControlCAN.h"
#include "JointController.h"
using namespace std;

// --- T1: Constructor
JointController::JointController(BYTE nodeId, int countsPerRev) : nodeId_(nodeId), countsPerRev_(countsPerRev) {}

// --- Functie ajutatoare: secventa completa de initializare ---
void JointController::initialize() {
    sendNMTCommand();
    resetFault();
    configurePositionMode();
    enableMotor();
    // Miscarea trebuie apelata separat cu parametri specifici
    sendSetPointAcknowledge();
    queryStatusWord();
}

// --- T7: pasii de miscare (Target Position / Velocity / Acceleration) ---
void JointController::setMotionParameters(int position, int velocity, int acceleration) {
    BYTE data[8];

    // 1. Set Target Position (0x607A)
    data[0] = 0x23;
    data[1] = 0x7A;
    data[2] = 0x60;
    data[3] = 0x00;
    // Extragem cei 4 octeti si ii punem in ordine inversa (Little Endian: facem operatia de shiftare spre dreapta si AND cu 0xFF pentru a obtine fiecare octet)
    data[4] = position & 0xFF;
    data[5] = (position >> 8) & 0xFF;
    data[6] = (position >> 16) & 0xFF;
    data[7] = (position >> 24) & 0xFF;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();

    // 2. Set Profile Velocity (0x6081)
    data[0] = 0x23;
    data[1] = 0x81;
    data[2] = 0x60;
    data[3] = 0x00;
    // Extragem cei 4 octeti si ii punem in ordine inversa (Little Endian: facem operatia de shiftare spre dreapta si AND cu 0xFF pentru a obtine fiecare octet)
    data[4] = velocity & 0xFF;
    data[5] = (velocity >> 8) & 0xFF;
    data[6] = (velocity >> 16) & 0xFF;
    data[7] = (velocity >> 24) & 0xFF;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();

    // 3. Set Profile Acceleration (0x6083)
    data[0] = 0x23;
    data[1] = 0x83;
    data[2] = 0x60;
    data[3] = 0x00;
    // Extragem cei 4 octeti si ii punem in ordine inversa (Little Endian: facem operatia de shiftare spre dreapta si AND cu 0xFF pentru a obtine fiecare octet)
    data[4] = acceleration & 0xFF;
    data[5] = (acceleration >> 8) & 0xFF;
    data[6] = (acceleration >> 16) & 0xFF;
    data[7] = (acceleration >> 24) & 0xFF;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
}

// --- T7-8: miscarea articulatiei (grade -> counts -> mesaje SDO) ---
void JointController::moveToAngle(double degrees, int velocity, int acceleration) {
    // --- Parametri gleznă ---
    // Range mecanic aproximativ: [-35°, +35°]  <->  [-9500, +9500] counts
    const double A_MAX_ANGLE_DEG = 35.0;
    const double A_MAX_COUNTS = 9500.0;
    const double A_COUNTS_PER_DEG = A_MAX_COUNTS / A_MAX_ANGLE_DEG; // ≈ 271.43

    // --- Parametri genunchi ---
    // Drept:  0      -> 0
    //         flexie -> -280000
    //         extensie -> +490000
    // Diferență de ~140° între flexie max și extensie max -> 770000/140 = 5500 counts/deg
    const double K_COUNTS_PER_DEG = 770000.0 / 140.0;         // = 5500.0
    const double K_MAX_FLEX_DEG = -280000.0 / K_COUNTS_PER_DEG; // ≈ -50.9°
    const double K_MAX_EXT_DEG = 490000.0 / K_COUNTS_PER_DEG; // ≈  89.1°

    // --- Parametri șold ---
    // 0°  = flexie (0 counts)
    // +40° ≈ extensie maximă (-400000 counts)
    const double H_MIN_DEG = 0.0;      // flexie (0 counts)
    const double H_MAX_DEG = 40.0;     // extensie maximă
    const double H_COUNTS_PER_DEG = 400000.0 / 40.0;  // = 10000.0

    int targetCounts = 0;

    // --- GLEZNA DREAPTĂ (nodeId_ == 7) ---
    if (nodeId_ == 7) {
        double d = degrees;
        if (d > A_MAX_ANGLE_DEG) d = A_MAX_ANGLE_DEG;
        if (d < -A_MAX_ANGLE_DEG) d = -A_MAX_ANGLE_DEG;

        // +deg (aduci vârful spre tine) -> -counts
        targetCounts = static_cast<int>(std::lround(-d * A_COUNTS_PER_DEG));
    }

    // --- GLEZNA STÂNGĂ (nodeId_ == 4) ---
    else if (nodeId_ == 4) {
        double d = degrees;
        if (d > A_MAX_ANGLE_DEG) d = A_MAX_ANGLE_DEG;
        if (d < -A_MAX_ANGLE_DEG) d = -A_MAX_ANGLE_DEG;

        // +deg (aduci vârful spre tine) -> +counts (oglindă față de dreapta)
        targetCounts = static_cast<int>(std::lround(d * A_COUNTS_PER_DEG));
    }

    // --- GENUNCHI DREPT (nodeId_ == 6) ---
    else if (nodeId_ == 6) {
        double d = degrees;

        // limităm la intervalul de lucru aproximativ [-50.9°, +89.1°]
        if (d < K_MAX_FLEX_DEG) d = K_MAX_FLEX_DEG;
        if (d > K_MAX_EXT_DEG)  d = K_MAX_EXT_DEG;

        // 0° -> 0, flexie (negativ) -> -counts, extensie (pozitiv) -> +counts
        // counts_R = 5500 * deg
        targetCounts = static_cast<int>(std::lround(K_COUNTS_PER_DEG * d));
    }

    // --- GENUNCHI STÂNG (nodeId_ == 3) ---
    else if (nodeId_ == 3) {
        double d = degrees;

        // folosim același interval de unghiuri ca la genunchiul drept
        if (d < K_MAX_FLEX_DEG) d = K_MAX_FLEX_DEG;
        if (d > K_MAX_EXT_DEG)  d = K_MAX_EXT_DEG;

        // oglindă față de drept: flexie (negativ) -> +counts, extensie (pozitiv) -> -counts
        // counts_L = -5500 * deg
        targetCounts = static_cast<int>(std::lround(-K_COUNTS_PER_DEG * d));
    }

    // --- ȘOLD DREPT sau STÂNG (nodeId_ == 5 sau 2) ---
    else if (nodeId_ == 5 || nodeId_ == 2) {
        double d = degrees;

        // limităm la [0°, +40°] : 0 = flexie, +40 = extensie max
        if (d < H_MIN_DEG) d = H_MIN_DEG;
        if (d > H_MAX_DEG) d = H_MAX_DEG;

        // extensie (+deg) -> counts negative
        // 0°   -> 0
        // 40°  -> -400000
        targetCounts = static_cast<int>(std::lround(-H_COUNTS_PER_DEG * d));
    }

    // --- Caz generic (dacă apare alt nodeId_) ---
    else {
        // fallback simplu: grade -> rotații -> counts folosind countsPerRev_
        double rev = degrees / 360.0;
        targetCounts = static_cast<int>(std::lround(rev * static_cast<double>(countsPerRev_)));
    }

    std::cout << "[Joint " << (int)nodeId_ << "] Move to " << degrees
        << "°  → " << targetCounts << " counts" << std::endl;

    setMotionParameters(targetCounts, velocity, acceleration);
}

// --- T5: comanda NMT Set Operational ---
bool JointController::sendNMTCommand() {
    VCI_CAN_OBJ nmt = {};
    nmt.ID = 0x000;
    nmt.DataLen = 2;
    nmt.Data[0] = 0x01;
    nmt.Data[1] = nodeId_;

    ULONG tran = VCI_Transmit(VCI_USBCAN2, 0, 0, &nmt, 1);

    if (tran) {
        cout << "NMT command sent successfully" << endl;
        cout << "Node-ID: 0x" << hex << (int)nmt.Data[1] << endl;
    }
    else {
        return false;
    }

    return true;
}

// --- T4: reset fault ---
void JointController::resetFault() {
    BYTE data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x80, 0x00, 0x00, 0x00 };
    bool success = sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
    if (success) {
        cerr << "Error reset command sent successfully" << endl;
    }
    else {
        cout << "Error: failed to send error reset command" << endl;
    }
}

// --- T6: setare Profile Position Mode ---
void JointController::configurePositionMode() {
    BYTE data[8] = { 0x2F, 0x60, 0x60, 0x00, 0x01, 0x00, 0x00, 0x00 };
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
}

// --- T4: enable motor ---
void JointController::enableMotor() {
    // Control Word
    BYTE data[8] = { 0 };
    data[0] = 0x2B;
    data[1] = 0x40;
    data[2] = 0x60;
    data[3] = 0x00;
    //Ready to switch on
    data[4] = 0x06;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
    // Switched on
    data[4] = 0x07;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
    //Operation Enabled
    data[4] = 0x0F;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
}

// --- Functie ajutatoare: setpoint acknowledge ---
void JointController::sendSetPointAcknowledge() {
    BYTE data[8] = { 0x2B, 0x40, 0x60, 0x00, 0x3F, 0x00, 0x00, 0x00 };
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
}

// --- Functie ajutatoare: citire Status Word ---
void JointController::queryStatusWord() {
    BYTE data[8] = { 0x40, 0x41, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00 };
    cout << "Querying Status Word..." << endl;
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
    if (!receiveCANMessage()) {
        cerr << "Failed to read Status Word for node " << (int)nodeId_ << endl;
    }
    else {
        cout << "Status Word read successfully." << endl;
    }
}

// --- T2: transmitere mesaj CAN ---
bool JointController::sendCANMessage(UINT cobID, BYTE data[], BYTE dataLen) {
    VCI_CAN_OBJ obj = {};

    obj.ID = cobID;
    obj.SendType = 0;
    obj.RemoteFlag = 0;
    obj.ExternFlag = 0;
    obj.DataLen = dataLen;
    memcpy(obj.Data, data, dataLen);

    ULONG res = VCI_Transmit(VCI_USBCAN2, 0, 0, &obj, 1);

    if (!res) {
        cerr << "Error: failed to transmit CAN message with COB-ID 0x"
            << hex << obj.ID << endl;
        return false;
    }
    else {
        cout << "COB-ID: 0x" << hex << obj.ID << endl;
        cout << "Message content: ";
        for (int i = 0; i < obj.DataLen; i++) {
            cout << hex << (int)obj.Data[i] << " ";
        }
    }
    return true;
}

// --- Functie ajutatoare: receptie mesaje CAN ---
bool JointController::receiveCANMessage() {
    VCI_CAN_OBJ receiveObj[50] = {};
    ULONG received = VCI_Receive(VCI_USBCAN2, 0, 0, receiveObj, 50, 100);

    if (received == 0) {
        cerr << "No messages received." << endl;
        return false;
    }

    for (ULONG i = 0; i < received; ++i) {
        cout << "Received: COB-ID: 0x" << hex << receiveObj[i].ID << ", Data: ";
        for (int j = 0; j < receiveObj[i].DataLen; ++j) {
            cout << hex << (int)receiveObj[i].Data[j] << " ";
        }
        cout << endl;

        if (receiveObj[i].ID == (0x580 + nodeId_)) {
            cout << "[Node " << (int)nodeId_ << "] SDO Response Received." << endl;
        }
        else if (receiveObj[i].ID == (0x80 + nodeId_)) {
            cout << "[Node " << (int)nodeId_ << "] Emergency Object Received! Error Code: "
                << hex << (int)receiveObj[i].Data[0]
                << (int)receiveObj[i].Data[1] << endl;
        }
    }
    return true;
}

int JointController::readActualPosition()
{
    BYTE data[8] = { 0x40, 0x64, 0x60, 0x00, 0,0,0,0 }; // 0x6064: Position Actual Value

    // Trimitem SDO read request
    sendCANMessage(0x600 + nodeId_, data, 8);

    VCI_CAN_OBJ recv[10] = {};
    ULONG received = VCI_Receive(VCI_USBCAN2, 0, 0, recv, 10, 100);

    if (received == 0) {
        return INT32_MIN; // semnalăm eroare
    }

    for (ULONG i = 0; i < received; ++i) {
        if (recv[i].ID == (0x580 + nodeId_)) {
            // 4 bytes little-endian începând cu Data[4]
            int32_t pos = 0;
            pos = (int32_t)recv[i].Data[4];
            pos |= ((int32_t)recv[i].Data[5] << 8);
            pos |= ((int32_t)recv[i].Data[6] << 16);
            pos |= ((int32_t)recv[i].Data[7] << 24);
            return (int)pos;
        }
    }

    return INT32_MIN;
}

bool JointController::waitUntilMovementStops(int stable_ms, int max_wait_ms)
{
    const int step_ms = 20;   // verificăm mai des (20 ms în loc de 50)
    const int threshold = 50;   // toleranță de poziție (counts) mai relaxată

    int elapsed = 0;
    int stable_elapsed = 0;

    int last_pos = 0;
    bool have_last = false;

    while (elapsed < max_wait_ms) {

        int pos = readActualPosition();
        if (pos != INT32_MIN) {

            if (!have_last) {
                // Prima citire
                last_pos = pos;
                have_last = true;
                stable_elapsed = 0;
            }
            else {
                int diff = abs(pos - last_pos);

                // actualizăm mereu poziția anterioară, ca să urmărim drift-ul fin
                last_pos = pos;

                if (diff > threshold) {
                    // încă se mișcă semnificativ -> resetăm timpul de „stabil”
                    stable_elapsed = 0;
                }
                else {
                    // poziția se schimbă foarte puțin -> creștem timpul de stabilitate
                    stable_elapsed += step_ms;
                    if (stable_elapsed >= stable_ms) {
                        cout << "[Joint " << (int)nodeId_
                            << "] Movement stopped (position stable)." << endl;
                        return true;
                    }
                }
            }
        }

        this_thread::sleep_for(chrono::milliseconds(step_ms));
        elapsed += step_ms;
    }

    cout << "[Joint " << (int)nodeId_
        << "] TIMEOUT waiting for movement to stop!" << endl;
    return false;
}


void JointController::sendControlWord(WORD cw)
{
    BYTE data[8] = {
        0x2B, 0x40, 0x60, 0x00,
        (BYTE)(cw & 0xFF),
        (BYTE)((cw >> 8) & 0xFF),
        0x00, 0x00
    };
    sendCANMessage(0x600 + nodeId_, data, 8);
    receiveCANMessage();
}

// --- T1: print la informatiile clasei ---
void JointController::printInfo() const {
    cout << "Node ID:" << (int)nodeId_ << endl;
    cout << "countsPerRev:" << countsPerRev_ << endl;
}
