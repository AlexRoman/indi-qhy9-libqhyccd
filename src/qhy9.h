#pragma once

#include <libindi/indiccd.h>

#include <math.h>

#include <qhyccd.h>

#include <time.h>

class QHY9 : public INDI::CCD {
public:
    QHY9();
    ~QHY9();

    // INDI::DefaultDevice methods that must be implemented follow
    void ISGetProperties (const char *dev);
    bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
    bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
    bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);
    bool ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n);

    bool Connect();
    bool Disconnect();
    const char *getDefaultName();

    void TimerHit();

    bool StartExposure(float duration);
    bool AbortExposure();

    bool UpdateCCDBin(int hor, int ver);

    bool initProperties();
    bool updateProperties();

    int SetTemperature(double temperature);

private:
    INumberVectorProperty q9CCDSettingsV;
    INumber q9CCDSettings[2];

    ISwitchVectorProperty q9SpeedSwitchV;
    ISwitch q9SpeedSwitch[2];

    int m_timerId;
    bool m_exposing;
    double m_targetTemp;
    qhyccd_handle *m_hdl;
    struct timeval ts_exposure_start;
    float currentExposureLength;
};
