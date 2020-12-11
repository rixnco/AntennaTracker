#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include <Arduino.h>
#include <stdint.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_SSD1306.h>





class Display : public Print {
public:
    virtual void init() = 0;
    virtual void clear() = 0;
    virtual void setCursor(uint16_t x, uint16_t y) = 0;
    virtual void show() {};

    virtual size_t write(uint8_t) = 0;
};

class DisplayProxy : public Display {
public:
    DisplayProxy(Display* display=nullptr) : _display(display) {}

    void setDisplay(Display* display) { _display = display; }

    virtual void init() override { if(_display) _display->init(); };
    virtual void clear() override { if(_display) _display->clear(); };
    virtual void setCursor(uint16_t x, uint16_t y) override { if(_display) _display->setCursor(x,y); };
    virtual void show() override { if(_display) _display->show(); };;

    virtual size_t write(uint8_t c) override { if(_display) { return _display->write(c); } return 0; };
private:
    Display* _display;
};


class OLED_Display : public Display {
public:
    OLED_Display(uint8_t addr, uint8_t rstPin) : _oled(128, 32, &Wire, rstPin), _addr(addr), _initialized(false) {}

    virtual void init() override {
        _initialized = _oled.begin(SSD1306_SWITCHCAPVCC, _addr);
        if(_initialized) {
            _oled.cp437(true);
            _oled.clearDisplay();
            _oled.setTextSize(1,2);             // 1:2 pixel scale
            _oled.setTextColor(SSD1306_WHITE);  // Draw white text
            _oled.display();
        }
    }
    virtual void clear() { 
        if(!_initialized) return;
        _oled.clearDisplay(); 
    }
    virtual void setCursor(uint16_t x, uint16_t y) override {
        if(!_initialized) return;
        _oled.setCursor(x*8, y*16);
    }
    virtual void show() override {
        if(!_initialized) return;
        _oled.display();
    }

    virtual size_t write(uint8_t c) override {
        if(!_initialized) return 0;
        return _oled.write(c);
    }
private:
    Adafruit_SSD1306 _oled;
    uint8_t          _addr;
    bool             _initialized;
};

class LCD_Display : public Display {
public:
    LCD_Display(uint8_t addr) : _lcd(static_cast<PCF8574_address>(addr)), _initialized(false) {}

    virtual void init() override {
        _initialized = _lcd.begin(16,2);
        if(_initialized) {
            _lcd.clear();
            _lcd.setCursor(0, 0);
        }
    }
    virtual void clear() { 
        if(!_initialized) return;
        _lcd.clear(); 
    }
    virtual void setCursor(uint16_t x, uint16_t y) override {
        if(!_initialized) return;
        _lcd.setCursor(x, y);
    }

    virtual size_t write(uint8_t c) override {
        if(!_initialized) return 0;
        return _lcd.write(c);
    }
private:
    LiquidCrystal_I2C _lcd;
    bool              _initialized;
};



#endif //__DISPLAY_H__