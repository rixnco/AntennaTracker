#include <Arduino.h>
#include <EEPROM.h>
#include "Settings.h"


#define EEPROM_SIZE             256
#define SETTINGS_MAGIC          0xDEADBEEF

SettingsManager Settings __attribute__((aligned(4)));



bool SettingsManager::init() {
    reset();
    return EEPROM.begin(EEPROM_SIZE);
};

// Settings Stuff
void SettingsManager::reset()
{
    memset(&_settings, 0, sizeof(_settings));
    _settings.magic = SETTINGS_MAGIC;
    _settings.version = SETTINGS_VERSION;
    _trackerMode = MODE_TRACK;
    Serial.println(">>> settings reset'd");            
}

bool SettingsManager::store()
{
    EEPROM.put<settings_t>(0, _settings);
    bool res=EEPROM.commit();
    Serial.println(">>> settings stored");
    return res;
}

bool SettingsManager::load()
{
    uint32_t magic;
    EEPROM.get<uint32_t>(0, magic);
    if (magic == SETTINGS_MAGIC)
    {
        // We got some settings stored in EEPROM
        uint32_t version;
        EEPROM.get<uint32_t>(4, version);
        if (version == SETTINGS_VERSION)
        {
            // Just read settings
            EEPROM.get<settings_t>(0, _settings);
            Serial.println(">>> settings loaded");            
            return true;
        }
        else
        {
            // Need to convert stored settings to our version
            // For now just reset settings to our version's default.
        }
    }
    reset();

    return false;
}




