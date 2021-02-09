#include "VRAParticleControl.h"    
    
VRAParticleControl::VRAParticleControl(
    UUID *p_uuid, EventQueue *p_eq, StateChain *p_stateChain, I2C *p_i2c, PinName vEnable):
    BLEService("prtclCtrl", p_uuid, p_eq, p_stateChain),
    eq(p_eq),
    i2c(p_i2c){
    this->vEnable = new DigitalOut(vEnable);
    this->particleSens = new Sps30(this->i2c);
}

void VRAParticleControl::init(){
    printf("[prtclCtrl] init\r\n");

}

void VRAParticleControl::initParticleSens(){
    printf("[prtclCtrl] init particle Sensor\r\n");
    this->particleSens->InitSPS30();
    this->particleSensRunning = true;
    // printf("[prtclCtrl] Serial Number: ");
    // for (int i = 0; i < 33; i++)
    // {
    //     printf("%x", particleSens->sn[i]);
    // }
    printf("\r\n");
}

void VRAParticleControl::initCharacteristics(){
    printf("[prtclCtrl] init Characteristics\r\n");
    this->addCharacteristic(
        new BLENotifyCharacteristic(
            (uint16_t) VRAParticleControl::Characteristics::ParticleMass,
            16,//size
            this->eq,
            this->defaultInterval,
            this->minInterval,
            this->maxInterval,
            callback(this, &VRAParticleControl::startParticleMeas)
        )
    );

    this->addCharacteristic(
        new BLECharacteristic(
            (uint16_t) VRAParticleControl::Characteristics::ParticleConc,
            GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ 
            | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY,
            20
        )
    );
}

void VRAParticleControl::pastBleInit(){
    printf("[prtclCtrl] pastBleInit\r\n");
}

void VRAParticleControl::onStateOff(){
    printf("[prtclCtrl] off\r\n");
}

void VRAParticleControl::onStateStandby(){
    printf("[prtclCtrl] standby\r\n");
}

void VRAParticleControl::onStateOn(){
    printf("[prtclCtrl] on\r\n");
}

void VRAParticleControl::startParticleMeas(){
    *this->vEnable = 1;
    this->eq->call_in(this->sps30StartupTime, callback(this, &VRAParticleControl::initParticleSens));
    this->eq->call_in(this->sps30StartupTime + this->minRunTime, callback(this, &VRAParticleControl::getParticleMeas));
}

void VRAParticleControl::getParticleMeas(){
    printf("[prtclCtrl] getParticleMass\r\n");
    if(!this->particleSensRunning){
        return;
    }
    this->particleSens->PollSPS30();
    this->particleSens->StopMeasurement();
    this->particleSens->SoftReset();
    this->vEnable = 0;
    float mass[4]{
        this->particleSens->mass_1p0_f,
        this->particleSens->mass_2p5_f,
        this->particleSens->mass_4p0_f,
        this->particleSens->mass_10p0_f
    };

    float conc[5]{
        this->particleSens->num_0p5_f,
        this->particleSens->num_1p0_f,
        this->particleSens->num_2p5_f,
        this->particleSens->num_4p0_f, 
        this->particleSens->num_10p0_f
    };

    printf("particle mass pm1: %d pm2.5: %d pm4:%d pm10:%d\r\n", 
    (int) mass[0], (int) mass[1], 
    (int) mass[2], (int) mass[3]);
    this->setGatt(
        (uint16_t) VRAParticleControl::Characteristics::ParticleMass,
        mass, 4
    );

    printf("particle conc pm 0.5: %d pm1: %d pm2.5: %d pm4:%d pm10:%d\r\n", 
    (int) conc[0], (int) conc[1], 
    (int) conc[2], (int) conc[3], 
    (int) conc[4]);

    this->setGatt(
        (uint16_t) VRAParticleControl::Characteristics::ParticleConc,
        conc, 5
    );
}