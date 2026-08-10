#ifndef JOINTCONTROLLER_H
#define JOINTCONTROLLER_H

#include <cstdint>
using BYTE = uint8_t;
using UINT = uint32_t;

class JointController {
public:
    JointController(BYTE nodeId, int countsPerRev);
    void printInfo() const;
    bool sendCANMessage(UINT cobID, BYTE data[], BYTE dataLen);
    bool receiveCANMessage();
    void resetFault();
    void enableMotor();
    bool sendNMTCommand();
    void configurePositionMode();
    void sendSetPointAcknowledge();
    void queryStatusWord();
    void setMotionParameters(int position, int velocity, int acceleration);
    void moveToAngle(double degrees, int velocity, int acceleration);
    void initialize();

    bool waitUntilMovementStops(int stable_ms, int max_wait_ms);
    void sendControlWord(WORD cw);
    int readActualPosition();
private:
    BYTE nodeId_;
    int countsPerRev_;
};

#endif