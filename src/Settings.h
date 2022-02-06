#ifndef __SETTINGS_H__
#define __SETTINGS_H__

#include <GeoUtils.h>
#include <stdint.h>
#include <string.h>

#define SETTINGS_VERSION        200u
#define SETTINGS_MAX_NAME_LEN   15u

enum trackerMode_t { MODE_TRACK, MODE_CALIB, MODE_NONE };

enum altitudeMode_t { ALT_ABSOLUTE, ALT_RELATIVE, ALT_NONE };




class SettingsManager {
private:
  typedef struct {
    uint32_t magic;
    uint32_t version;

    float homeLatitude;
    float homeLongitude;
    float homeElevation;
    char  homeName[SETTINGS_MAX_NAME_LEN+1];

    float calibLatitude;
    float calibLongitude;
    float calibElevation;
    char  calibName[SETTINGS_MAX_NAME_LEN+1];

    int32_t pan;
    int32_t tilt;

    altitudeMode_t altitudeMode;
  } __attribute__((packed)) settings_t;

  settings_t _settings;
  trackerMode_t _trackerMode;

public:
    bool init();

    // Volatile settings
    inline trackerMode_t getTrackerMode() { return _trackerMode; }
    inline void setTrackerMode(trackerMode_t mode) { _trackerMode = mode; }

    // Persistante settings
    inline void  setHome(float lat, float lon, float elv) { _settings.homeLatitude = lat; _settings.homeLongitude = lon; _settings.homeElevation = elv; }
    inline void  setHome(GeoPt& home) { _settings.homeLatitude = home.getLatitude(); _settings.homeLongitude = home.getLongitude(); _settings.homeElevation = home.getElevation(); }
    inline void  setHomeName(const char* name) { strncpy(_settings.homeName, name, SETTINGS_MAX_NAME_LEN); _settings.homeName[SETTINGS_MAX_NAME_LEN] = '\0'; }
    inline GeoPt getHome() { return GeoPt(_settings.homeLatitude, _settings.homeLongitude, _settings.homeElevation); };
    inline const char* getHomeName() { return _settings.homeName; }
    inline float getHomeLatitude() { return _settings.homeLatitude; }
    inline float getHomeLongitude() { return _settings.homeLongitude; }
    inline float getHomeElevation() { return _settings.homeElevation; }

    inline void  setCalib(float lat, float lon, float elv) { _settings.calibLatitude = lat; _settings.calibLongitude = lon; _settings.calibElevation = elv; }
    inline void  setCalib(GeoPt& calib) { _settings.calibLatitude = calib.getLatitude(); _settings.calibLongitude = calib.getLongitude(); _settings.calibElevation = calib.getElevation(); }
    inline void  setCalibName(const char* name) { strncpy(_settings.calibName, name, SETTINGS_MAX_NAME_LEN); _settings.calibName[SETTINGS_MAX_NAME_LEN] = '\0'; }
    inline GeoPt getCalib() { return GeoPt(_settings.calibLatitude, _settings.calibLongitude, _settings.calibElevation); };
    inline const char* getCalibName() { return _settings.calibName; }
    inline float getCalibLatitude() { return _settings.calibLatitude; }
    inline float getCalibLongitude() { return _settings.calibLongitude; }
    inline float getCalibElevation() { return _settings.calibElevation; }

    inline void setTrackedMode(trackerMode_t mode) { _trackerMode = mode; }
    inline trackerMode_t getTrackedMode() { return _trackerMode; }

    inline void setPanOffset(int pan) { _settings.pan = pan; }
    inline void adjPanOffset(int16_t delta) { _settings.pan+= delta; }
    inline int  getPanOffset() { return _settings.pan; }

    inline void setTiltOffset(int tilt) { _settings.tilt = tilt; }
    inline void adjTiltOffset(int16_t delta) { _settings.tilt+=delta; }
    inline int  getTiltOffset() { return _settings.tilt; }

    inline altitudeMode_t getAltitudeMode() { return _settings.altitudeMode; }
    inline void setAltitudeMode(altitudeMode_t mode) { _settings.altitudeMode = mode; }


    bool store();
    bool load();
    void reset();


};



extern SettingsManager Settings;


#endif