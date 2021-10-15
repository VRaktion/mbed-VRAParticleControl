#ifndef VRA_PARTICLE_CONTROL_H
#define VRA_PARTICLE_CONTROL_H

#include "BLEService.h"

// #include "VRASettings.h"
// #include "VRAStorage.h"
#include "BLENotifyCharacteristic.h"

#include "Sps30.h"

class VRAParticleControl : public BLEService
{
public:
    enum class Characteristics : uint16_t
    {
        ParticleMass = 0xFF00,
        ParticleConc = 0xFF01,
        ParticleClean = 0xFF02
    };

    VRAParticleControl(UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c, PinName vEnable = NC);

    void init();

    void initCharacteristics();
    void pastBleInit();

private:
    void initParticleSens();

    void onStateOff();
    void onStateStandby();
    void onStateOn();

    void startParticleMeas();
    void getParticleMeas();

    void cleanWriteCb();
    void cleanSensor();
    void stopClean();

    EventQueue *eq;
    // VRASettings *settings;
    // VRAStorage *storage;

    I2C *i2c;

    Sps30 *particleSens;
    DigitalOut *vEnable;
    bool particleSensRunning{false};

    static constexpr int sps30StartupTime{200};
    static constexpr int minRunTime{30000};
    static constexpr int defaultInterval{600000};
    static constexpr int minInterval{35000};
    static constexpr int maxInterval{6000000};

    BLECharacteristic *cleanCharacteristic;

    volatile bool blocked{false};
};

#endif //VRA_PARTICLE_CONTROL_H