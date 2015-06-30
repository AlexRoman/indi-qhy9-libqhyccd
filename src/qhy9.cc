#include "qhy9.h"

#include <memory>

#include <qhyccd.h>

#include <unistd.h>

#include <float.h>

#include <time.h>

QHY9::QHY9() : INDI::CCD(),
    m_timerId(-1),
    m_exposing(false),
    m_targetTemp(NAN),
    m_hdl(NULL)
{
    INDI::CCD::Capability cap;
    cap.hasGuideHead = false;
    cap.hasST4Port = false;
    cap.hasShutter = true;
    cap.hasCooler = true;
    cap.canBin = true;
    cap.canSubFrame = false;    // TODO: figure out how
    cap.canAbort = true;

    IDLog("%s()\n", __FUNCTION__);

    SetCapability(&cap);
}

QHY9::~QHY9() {
    IDLog("%s()\n", __FUNCTION__);
}

//
// INDI::DefaultDevice methods
//

bool QHY9::Connect() {
    IDLog("%s()\n", __FUNCTION__);

    int ret = InitQHYCCDResource();
    if (ret != QHYCCD_SUCCESS) {
        IDLog("OpenCamera() failed.\n");
        return false;
    }

    ret = ScanQHYCCD();
    if (ret < 0) {
        IDLog("Could not find a device (%d)\n", ret);
        ReleaseQHYCCDResource();
        return false;
    }

    char id[0x20] = {0};    // this is defined as 0x20 in qhyccd lib
    for (int i=0; i<ret; ++i) {
        ret = GetQHYCCDId(i, id);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not get CCD Id for index=%d\n", i);
            continue;
        }
        IDLog("Found camera with id '%s'\n", id);

        if (strncmp(id, "QHY9", 4) != 0)
            continue;

        IDLog("Opening camera...\n");

        // Found our camera!
        m_hdl = OpenQHYCCD(id);
        if (!m_hdl) {
            IDLog("Could not open camera with id '%s'\n", id);
            continue;
        }

        // Let's initialize it!
        ret = InitQHYCCD(m_hdl);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not initialize camera (%d)\n", ret);
            CloseQHYCCD(m_hdl);
            m_hdl = NULL;
            continue;
        }

        break;
    }

    if (!m_hdl) {
        IDLog("Could not find a matching camera\n");
        ReleaseQHYCCDResource();
        return false;
    }

    //SetCCDParams(896, 644, 16, 5.4f, 5.4f);

    SetQHYCCDBinMode(m_hdl, 1, 1);
    SetQHYCCDResolution(m_hdl, 3584, 2574);
    SetCCDParams(3584, 2574, 16, 5.4f, 5.4f);

    int nbuf = GetQHYCCDMemLength(m_hdl);
    if (nbuf < 0) {
        IDLog("Could not get max mem size\n");
    }
    IDLog("Setting primary CCD buffer size to %d\n", nbuf);
    PrimaryCCD.setFrameBufferSize(nbuf, true);

    // Try to set gain
    double gainMin, gainMax, gainStep;
    ret = GetQHYCCDParamMinMaxStep(m_hdl, CONTROL_GAIN, &gainMin, &gainMax, &gainStep);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for GAIN (%d)\n", ret);
    } else {
        IDLog("Gain settings (%.1f, %.1f, +%.1f), setting to 14.0\n", gainMin, gainMax, gainStep);
        ret = SetQHYCCDParam(m_hdl, CONTROL_GAIN, 14.0);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set gain to 14.0 (%d)!\n", ret);
        }
    }

    // Try to set offset
    ret = GetQHYCCDParamMinMaxStep(m_hdl, CONTROL_OFFSET, &gainMin, &gainMax, &gainStep);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not get min/max/step for OFFSET (%d)\n", ret);
    } else {
        IDLog("Offset settings (%.1f, %.1f, +%.1f), setting to 107.0\n", gainMin, gainMax, gainStep);
        ret = SetQHYCCDParam(m_hdl, CONTROL_OFFSET, 107.0);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("Could not set offset to 107.0 (%d)!\n", ret);
        }
    }

    SetQHYCCDParam(m_hdl, CONTROL_SPEED, 0);

    // Read current temperature
    double temp = GetQHYCCDParam(m_hdl, CONTROL_CURTEMP);
    IDLog("Current temp: %.1f   Target temp: %.1f\n", temp, m_targetTemp);
    TemperatureN[0].value = temp;			/* CCD chip temperatre (degrees C) */
    TemperatureNP.s = IPS_BUSY;
    IDSetNumber(&TemperatureNP, NULL);

    // Start timer for cooler, etc.
    SetTimer(500);

    return true;
}

bool QHY9::Disconnect() {
    IDLog("%s()\n", __FUNCTION__);

    CloseQHYCCD(m_hdl);
    m_hdl = NULL;
    ReleaseQHYCCDResource();

    return true;
}

bool QHY9::StartExposure(float duration) {
    IDLog("%s(%.1f) frame type = %d\n", __FUNCTION__, duration, PrimaryCCD.getFrameType());

    if (PrimaryCCD.getFrameType() == CCDChip::DARK_FRAME ||
        PrimaryCCD.getFrameType() == CCDChip::BIAS_FRAME) {
        IDLog("Setting shutter closed!\n");
        SetQHYCCDShutter(m_hdl, SHUTTER_CLOSED);
    } else {
        IDLog("Setting shutter open!\n");
        SetQHYCCDShutter(m_hdl, SHUTTER_OPEN);
    }

    int ret = SetQHYCCDParam(m_hdl, CONTROL_EXPOSURE, (int)(duration * 1000000));
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not set exposure time to %d us\n", (int)(duration * 1000000));
        return false;
    }

    IDLog("Set exposure time.. begin exposing\n");

    ret = ExpQHYCCDSingleFrame(m_hdl);
    if (ret != QHYCCD_SUCCESS) {
        IDLog("Could not start exposure (%d)\n", ret);
        return false;
    }

    gettimeofday(&ts_exposure_start, NULL);
    currentExposureLength = duration;

    IDLog("Started exposing\n");

    m_exposing = true;
    return true;
}

#define TIMEVAL_DIFF_MSEC(a,b) (((b).tv_usec / 1000 + (b).tv_sec * 1000) - ((a).tv_usec / 1000 + (a).tv_sec * 1000))

#define TIME_EXECUTION(t, code) {\
    struct timeval start, end; \
    gettimeofday(&start, NULL); \
    { code } \
    gettimeofday(&end, NULL); \
    t = TIMEVAL_DIFF_MSEC(start, end); \
}

void QHY9::TimerHit() {
    int ret;

    struct timeval ts_start, ts_end;
    long msec;

    gettimeofday(&ts_start, NULL);

    if (!m_hdl) {
        return;
    }

    if (m_exposing) {
        // Check if we should download the image.
        gettimeofday(&ts_end, NULL);
        long elapsed_msec = TIMEVAL_DIFF_MSEC(ts_exposure_start, ts_end);
        long msec_left = (long)currentExposureLength * 1000 - elapsed_msec;
        IDLog("%ld msec elapsed, %ld msec left\n", elapsed_msec, msec_left);
        PrimaryCCD.setExposureLeft(msec_left / 1000);

        if (msec_left <= 1000) {
            // Read the frame data
            int w = 0, h = 0, bpp = 0, channels = 0;
            unsigned char *buf = (unsigned char *)PrimaryCCD.getFrameBuffer();
            TIME_EXECUTION(msec, {ret = GetQHYCCDSingleFrame(m_hdl, &w, &h, &bpp, &channels, buf);});
            IDLog("GetQHYCCDSingleFrame() took %ld msec to execute\n", msec);
            if (ret >= 0) {
                IDLog("Done exposing!\n");
                IDLog("w = %d, h = %d, bpp=%d, channels=%d\n",
                        w, h, bpp, channels);
                ExposureComplete(&PrimaryCCD);
            }
            // Free the shutter
            IDLog("Freeing shutter\n");
            SetQHYCCDShutter(m_hdl, SHUTTER_FREE);

            m_exposing = false;
            usleep(500);
        }
    }

    // Call temp control
    if (!isnan(m_targetTemp)) {
        TIME_EXECUTION(msec, {ret = ControlQHYCCDTemp(m_hdl, m_targetTemp);});
        IDLog("ControlQHYCCDTemp() took %ld msec to execute\n", msec);
        if (ret != QHYCCD_SUCCESS) {
            IDLog("ControlQHYCCDTemp(0x%p, %.1f) failed (%d)\n",
                    (void *)m_hdl, m_targetTemp, ret);
        }
    }

    // Read current temperature
    double temp = GetQHYCCDParam(m_hdl, CONTROL_CURTEMP);
    IDLog("Current temp: %.1f   Target temp: %.1f\n", temp, m_targetTemp);
    TemperatureN[0].value = temp;			/* CCD chip temperatre (degrees C) */

    if (!isnan(m_targetTemp)) {
        if (fabs(temp - m_targetTemp) < 1.0) {
            TemperatureNP.s = IPS_OK;
        } else if (fabs(temp - m_targetTemp) < 4.0) {
            TemperatureNP.s = IPS_BUSY;
        } else {
            TemperatureNP.s = IPS_ALERT;
        }
        IDSetNumber(&TemperatureNP, NULL);
    }

    temp = GetQHYCCDParam(m_hdl, CONTROL_CURPWM);
    IDLog("Current PWM: %.1f\n", temp);

    gettimeofday(&ts_end, NULL);
    msec = TIMEVAL_DIFF_MSEC(ts_start, ts_end);
    IDLog("Timer code execution took %ld msec\n", msec);

    // Set next timer
    m_timerId = SetTimer(1000);
}

bool QHY9::AbortExposure() {
    return (QHYCCD_SUCCESS == StopQHYCCDExpSingle(m_hdl));
}

bool QHY9::UpdateCCDBin(int hor, int ver) {
    if ((hor != ver) || (hor > 4) || (hor < 1))
        return false;

    if (QHYCCD_SUCCESS != SetQHYCCDBinMode(m_hdl, hor, ver)) {
        return false;
    }

    SetCCDParams(3584 / hor, 2574 / ver, 16, 5.4f * hor, 5.4f * ver);

    return true;
}

int QHY9::SetTemperature(double temperature) {
    if (temperature < -40.0) {
        return -1;
    }

    m_targetTemp = temperature;

    return 0;
}

bool QHY9::initProperties() {
    IDLog("%s()\n", __FUNCTION__);

    if (!INDI::CCD::initProperties()) {
        return false;
    }

    IUFillNumber(&q9CCDSettings[0], "GAIN", "CCD gain (%)", "%d", 1, 100, 1, 14);
    IUFillNumber(&q9CCDSettings[1], "OFFSET", "CCD offset", "%d", 1, 255, 1, 107);
    IUFillNumberVector(&q9CCDSettingsV, q9CCDSettings, NARRAY(q9CCDSettings), getDefaultName(), "QHY9_CCD_SETTINGS", "CCD Settings", "QHY9", IP_RW, 2, IPS_OK);

    IUFillSwitch(&q9SpeedSwitch[0], "SPEED_LOW", "Low", ISS_ON);
    IUFillSwitch(&q9SpeedSwitch[1], "SPEED_HIGH", "High", ISS_OFF);
    IUFillSwitchVector(&q9SpeedSwitchV, q9SpeedSwitch, NARRAY(q9SpeedSwitch), getDefaultName(), "QHY9_READOUT_SPEED", "Readout speed", "QHY9", IP_RW, ISR_1OFMANY, 2, IPS_OK);

    IDLog("initProperties() done\n");

    return true;
}

bool QHY9::updateProperties() {
    IDLog("%s()\n", __FUNCTION__);

    if (!INDI::CCD::updateProperties()) {
        return false;
    }

    /* TODO */

    return true;
}

const char *QHY9::getDefaultName() {
    IDLog("%s()\n", __FUNCTION__);
    return "QHY9";
}

void QHY9::ISGetProperties (const char *dev) {
    IDLog("%s(%s)\n", __FUNCTION__, dev);

    INDI::CCD::ISGetProperties(dev);
}

bool QHY9::ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    IDLog("%s(%s, %s, n=%d)\n", __FUNCTION__, dev, name, n);

    if (strcmp(dev, getDeviceName()))
        return false;

    ISwitchVectorProperty *svp = getSwitch(name);

    if (0 == strcmp(svp->name, "QHY9_READOUT_SPEED")) {
        q9SpeedSwitchV.s = IPS_OK;
        IUUpdateSwitch(svp, states, names, n);

        saveConfig();

        int speed = -1;
        if (q9SpeedSwitch[0].s == ISS_ON) {
            IDLog("QHY9::%s() - LOW SPEED readout\n", __FUNCTION__);
            SetQHYCCDParam(m_hdl, CONTROL_SPEED, 0.0);
            speed = 0;
        }

        if (q9SpeedSwitch[1].s == ISS_ON) {
            IDLog("QHY9::%s() - HIGH SPEED readout\n", __FUNCTION__);
            SetQHYCCDParam(m_hdl, CONTROL_SPEED, 1.0);
            speed = 1;
        }

        IDSetSwitch(svp, "Readout speed is now %s", speed ? "HIGH" : "LOW");

        return true;
    }

    if (INDI::CCD::ISNewSwitch(dev, name, states, names, n))
        return true;

    return false;
}

bool QHY9::ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    IDLog("%s(%s, %s, n=%d)\n", __FUNCTION__, dev, name, n);

    if (strcmp(dev, getDeviceName()))
        return false;

    INumberVectorProperty *nvp = getNumber(name);

    int gain = 0, offset = 0;
    if (strcmp(nvp->name, "QHY9_CCD_SETTINGS") == 0) {
        for (int i = 0; i < n; ++i) {
            if (strcmp(names[i], "GAIN") == 0) {
                gain = values[i];
                SetQHYCCDParam(m_hdl, CONTROL_GAIN, values[i]);
            } else if (strcmp(names[i], "OFFSET") == 0) {
                offset = values[i];
                SetQHYCCDParam(m_hdl, CONTROL_OFFSET, values[i]);
            }

            IDLog("%s: Set '%s' to %.1f\n", __FUNCTION__, names[i], values[i]);
        }
        IDSetNumber(nvp, "Set GAIN to %d%% and OFFSET to %d", gain, offset);
    }

    if (INDI::CCD::ISNewNumber(dev, name, values, names, n))
        return true;

    return false;
}

bool QHY9::ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    IDLog("%s(%s, %s, n=%d)\n", __FUNCTION__, dev, name, n);

    if (strcmp(dev, getDeviceName()))
        return false;

    if (INDI::CCD::ISNewText(dev, name, texts, names, n))
        return true;

    return false;
}

bool QHY9::ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
    IDLog("%s(%s, %s, n=%d)\n", __FUNCTION__, dev, name, n);

    if (strcmp(dev, getDeviceName()))
        return false;

    if (INDI::CCD::ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n))
        return true;

    return false;
}

//
// General INDI driver link
//

static std::unique_ptr<QHY9> m_ccd_ptr(nullptr);

static void ccd_ptr_init (void) {
    if (m_ccd_ptr != nullptr)
        return;

    m_ccd_ptr.reset(new QHY9());
}

void ISGetProperties (const char *dev) {
    ccd_ptr_init();
    m_ccd_ptr->ISGetProperties(dev);
}

void ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n) {
    ccd_ptr_init();
    m_ccd_ptr->ISNewSwitch(dev, name, states, names, n);
}

void ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n) {
    ccd_ptr_init();
    m_ccd_ptr->ISNewNumber(dev, name, values, names, n);
}

void ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n) {
    ccd_ptr_init();
    m_ccd_ptr->ISNewText(dev, name, texts, names, n);
}

void ISNewBLOB(const char *dev, const char *name, int sizes[], int blobsizes[], char *blobs[], char *formats[], char *names[], int n) {
    ccd_ptr_init();
    m_ccd_ptr->ISNewBLOB(dev, name, sizes, blobsizes, blobs, formats, names, n);
}

void ISSnoopDevice(XMLEle *root) {
    ccd_ptr_init();
    INDI_UNUSED(root);
}
