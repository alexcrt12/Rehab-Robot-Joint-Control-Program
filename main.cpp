#include <iostream>
#include <cmath>
#include <chrono>
#include <thread>
#include "ControlCAN.h"
#include "JointController.h"
#define PI 3.14159265358979323846
using namespace std;

struct LegAngles {
    double hip;
    double knee;
    double ankle;
};

struct rangeOfMotion {
    double hMin = 0.0;
    double hMax = 25.0;
    double kMin = -5.0;
    double kMax = 40.0;
    double aMin = -10.0;
    double aMax = 10.0;
};

double mapSineToRange(double phase, double min_angle, double max_angle, double phase_offset = 0.0) {
    double amplitude = (max_angle - min_angle) / 2.0;
    double midpoint = (max_angle + min_angle) / 2.0;

    return midpoint + amplitude * sin(phase + phase_offset);
}

LegAngles computeCyclingAngles(double phase, double hip_min, double hip_max, double knee_min, double knee_max, double ankle_min, double ankle_max) {
    LegAngles angles;
    
    angles.hip = mapSineToRange(phase, hip_min, hip_max, 0.0);
    angles.knee = mapSineToRange(phase, knee_min, knee_max, -PI / 2.0);
    angles.ankle = mapSineToRange(phase, ankle_min, ankle_max, PI / 4.0);

    return angles;
}

void moveAndStart(JointController& joint, double angle, int vel, int acc)
{
    joint.moveToAngle(angle, vel, acc);
    joint.sendControlWord(0x0F);
    joint.sendControlWord(0x3F);
}

void moveAndRetreat(JointController& rightHip, JointController& leftHip, JointController& rightAnkle, JointController& leftAnkle, JointController& rightKnee, JointController& leftKnee, double angleK, double angleA, double angleH, int velHK, int accHK, int velA, int accA) {
    double angle0 = 0.0;

    cout << "Moving the left hip to " << angleH << "degrees\n";
    moveAndStart(leftHip, angleH, velHK, accHK);
    leftHip.waitUntilMovementStops(1500, 20000);

    moveAndStart(leftHip, angle0, velHK, accHK);
    leftHip.waitUntilMovementStops(1500, 20000);

    cout << "Moving the left knee to " << angleK << "degrees\n";
    moveAndStart(leftKnee, angleK, velHK, accHK);
    leftKnee.waitUntilMovementStops(1500, 20000);

    moveAndStart(leftKnee, angle0, velHK, accHK);
    leftKnee.waitUntilMovementStops(1500, 20000);

    cout << "Moving the left ankle to " << angleA << "degrees\n";
    moveAndStart(leftAnkle, angleA, velA, accA);
    leftAnkle.waitUntilMovementStops(1500, 20000);

    moveAndStart(leftAnkle, angle0, velA, accA);
    leftAnkle.waitUntilMovementStops(1500, 20000);

    cout << "Moving the right hip to " << angleH << "degrees\n";
    moveAndStart(rightHip, angleH, velHK, accHK);
    rightHip.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightHip, angle0, velHK, accHK);
    rightHip.waitUntilMovementStops(1500, 20000);

    cout << "Moving the right knee to " << angleK << "degrees\n";
    moveAndStart(rightKnee, angleK, velHK, accHK);
    rightKnee.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightKnee, angle0, velHK, accHK);
    rightKnee.waitUntilMovementStops(1500, 20000);

    cout << "Moving the right ankle to " << angleA << "degrees\n";
    moveAndStart(rightAnkle, angleA, velA, accA);
    rightAnkle.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightAnkle, angle0, velA, accA);
    rightAnkle.waitUntilMovementStops(1500, 20000);
}

// Pozitia de 0 pentru toate articulatiile
void moveAllToZero(JointController& leftAnkle, JointController& rightAnkle, JointController& leftKnee, JointController& rightKnee, JointController& leftHip, JointController& rightHip, int velHK, int accHK, int velA, int accA)
{
    double angle0 = 0.0;

    // Glezne
    moveAndStart(leftAnkle, angle0, velA, accA);
    leftAnkle.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightAnkle, angle0, velA, accA);
    rightAnkle.waitUntilMovementStops(1500, 20000);

    leftAnkle.waitUntilMovementStops(1500, 20000);
    rightAnkle.waitUntilMovementStops(1500, 20000);

    // Genunchi
    moveAndStart(leftKnee, angle0, velHK, accHK);
    leftKnee.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightKnee, angle0, velHK, accHK);
    rightKnee.waitUntilMovementStops(1500, 20000);

    leftKnee.waitUntilMovementStops(1500, 20000);
    rightKnee.waitUntilMovementStops(1500, 20000);

    // Solduri
    moveAndStart(leftHip, angle0, velHK, accHK);
    leftHip.waitUntilMovementStops(1500, 20000);

    moveAndStart(rightHip, angle0, velHK, accHK);
    rightHip.waitUntilMovementStops(1500, 20000);

    leftHip.waitUntilMovementStops(1500, 20000);
    rightHip.waitUntilMovementStops(1500, 20000);
}

void cycleMovement(JointController& rightHip, JointController& leftHip, JointController& rightKnee, JointController& leftKnee, JointController& leftAnkle, JointController& rightAnkle, int velHK, int accHK, int velA, int accA) {
    rangeOfMotion Rom;
    int totalCycles = 3;
    int framesPerCycle = 10;
    double phaseStep = (2 * PI) / framesPerCycle;

    for (int cycle = 0; cycle < totalCycles; cycle++) {
        for (int frame = 0; frame < framesPerCycle; frame++) {
            double currentPhase = frame * phaseStep;

            LegAngles rightLeg = computeCyclingAngles(currentPhase, Rom.hMin, Rom.hMax, Rom.kMin, Rom.kMax, Rom.aMin, Rom.aMax);
            LegAngles leftLeg = computeCyclingAngles(currentPhase + PI, Rom.hMin, Rom.hMax, Rom.kMin, Rom.kMax, Rom.aMin, Rom.aMax);

            moveAndStart(rightHip, rightLeg.hip, velHK, accHK);
            moveAndStart(rightKnee, rightLeg.knee, velHK, accHK);
            moveAndStart(rightAnkle, rightLeg.ankle, velA, accA);
            moveAndStart(leftHip, leftLeg.hip, velHK, accHK);
            moveAndStart(leftKnee, leftLeg.knee, velHK, accHK);
            moveAndStart(leftAnkle, leftLeg.ankle, velA, accA);

            this_thread::sleep_for(chrono::milliseconds(15));
        }
    }

    moveAllToZero(leftAnkle, rightAnkle, leftKnee, rightKnee, leftHip, rightHip, velHK, accHK, velA, accA);

}

int main() {
    VCI_INIT_CONFIG config = {};
    config.AccCode = 0;
    config.AccMask = 0xFFFFFFFF;
    config.Reserved = 0;
    config.Filter = 0;
    config.Timing0 = 0x00;
    config.Timing1 = 0x14;
    config.Mode = 0;

    JointController leftKnee(3, 4096);
    JointController rightKnee(6, 4096);
    JointController leftAnkle(4, 4096);
    JointController rightAnkle(7, 4096);
    JointController leftHip(2, 4096);
    JointController rightHip(5, 4096);
    leftKnee.printInfo();
    rightKnee.printInfo();
    leftAnkle.printInfo();
    rightAnkle.printInfo();
    leftHip.printInfo();
    rightHip.printInfo();
    UINT cobID_LK = 0x603;
    UINT cobID_RK = 0x606;
    UINT cobID_LA = 0x604;
    UINT cobID_RA = 0x607;
    UINT cobID_LH = 0x602;
    UINT cobID_RH = 0x605;
    unsigned char dataLK[8] = { 0 };
    unsigned char dataRK[8] = { 0 };
    unsigned char dataLA[8] = { 0 };
    unsigned char dataRA[8] = { 0 };
    unsigned char dataLH[8] = { 0 };
    unsigned char dataRH[8] = { 0 };
    double angleK = 60.0;
    double angleA = 30.0;
    double angleH = 15.0;
    int velHK = 300000;
    int accHK = 80000;
    int velA = 9000;
    int accA = 4500;

    DWORD open = VCI_OpenDevice(VCI_USBCAN2, 0, 0);
    if (open != STATUS_OK) {
        cout << "Error: failed to open CAN device" << endl;
        return 1;
    }
    DWORD init = VCI_InitCAN(VCI_USBCAN2, 0, 0, &config);
    if (init != STATUS_OK) {
        cout << "Error: failed to initialize CAN device" << endl;
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 1;
    }
    DWORD start = VCI_StartCAN(VCI_USBCAN2, 0, 0);
    if (start != STATUS_OK) {
        cout << "Error: failed to start CAN device" << endl;
        VCI_CloseDevice(VCI_USBCAN2, 0);
        return 1;
    }

    leftKnee.sendCANMessage(cobID_LK, dataLK, 8);
    rightKnee.sendCANMessage(cobID_RK, dataRK, 8);
    leftAnkle.sendCANMessage(cobID_LA, dataLA, 8);
    rightAnkle.sendCANMessage(cobID_RA, dataRA, 8);
    leftHip.sendCANMessage(cobID_LH, dataLH, 8);
    rightHip.sendCANMessage(cobID_RH, dataRH, 8);
    
    leftKnee.receiveCANMessage();
    rightKnee.receiveCANMessage();
    leftAnkle.receiveCANMessage();
    rightAnkle.receiveCANMessage();
    leftHip.receiveCANMessage();
    rightHip.receiveCANMessage();

    leftKnee.initialize();
    rightKnee.initialize();
    leftAnkle.initialize();
    rightAnkle.initialize();
    leftHip.initialize();
    rightHip.initialize();

    // moveAndRetreat(rightHip, leftHip, rightAnkle, leftAnkle, rightKnee, leftKnee, angleK, angleA, angleH, velHK, accHK, velA, accA);
    cycleMovement(rightHip, leftHip, rightKnee, leftKnee, leftAnkle, rightAnkle, velHK, accHK, velA, accA);

    VCI_CloseDevice(VCI_USBCAN2, 0);
    cout << "Device closed!\n";

    return 0;
}