// M5Stack StopWatch power-up: M5PM1 rails on, then the panel/touch resets on the M5IOE1
// expander. A register-for-register copy of M5GFX's "stopwatch_power_on"
// (src/board_detect/m5/pmic_ops.hpp) -- raw writes instead of the M5PM1/M5IOE1 libraries,
// which would only wrap these same ~15 registers.
#include "config.h"
#if defined(BOARD_STOPWATCH)
#include <Arduino.h>
#include <Wire.h>

// Both chips may NACK the first transfer after idle (they wake on it): retry.
static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *v) {
    for (int i = 0; i < 5; ++i) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) { delay(2); continue; }
        delayMicroseconds(500);   // the PM1 needs a moment before the read (M5PM1 library waits 500 us too)
        if (Wire.requestFrom(addr, (uint8_t)1) == 1) {
            *v = Wire.read();
            return true;
        }
        delay(2);
    }
    Serial.printf("[board] I2C 0x%02X reg 0x%02X read failed\n", addr, reg);
    return false;
}

// Multi-byte read in one transaction: the PM1 latches VBAT across its two bytes, and two
// single-byte reads returned the low byte only (106 mV with a charged cell).
static bool reg_read_n(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
    for (int i = 0; i < 5; ++i) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) { delay(2); continue; }
        delayMicroseconds(500);
        if (Wire.requestFrom(addr, n) == n) {
            for (uint8_t k = 0; k < n; ++k) buf[k] = Wire.read();
            return true;
        }
        delay(2);
    }
    Serial.printf("[board] I2C 0x%02X reg 0x%02X read %u failed\n", addr, reg, n);
    return false;
}

static void reg_write(uint8_t addr, uint8_t reg, uint8_t v) {
    for (int i = 0; i < 5; ++i) {
        Wire.beginTransmission(addr);
        Wire.write(reg);
        Wire.write(v);
        if (Wire.endTransmission() == 0) return;
        delay(2);
    }
    Serial.printf("[board] I2C 0x%02X reg 0x%02X write failed\n", addr, reg);
}

static void bits(uint8_t addr, uint8_t reg, uint8_t mask, bool on) {
    uint8_t v;
    if (!reg_read(addr, reg, &v)) return;
    reg_write(addr, reg, on ? (v | mask) : (v & ~mask));
}

static bool ack(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

void board_power_begin() {
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, 100000);   // PM1/IOE1 run at 100 kHz in M5GFX

    // M5PM1: I2C_CFG 0x09 = 400 kHz (SPD, bit 4), no I2C sleep -- M5GFX writes 0 (100 kHz), but
    // the bus runs at 400 kHz for touch afterwards and the PM1 then NACKs every read.
    // Clear 0x0A, enable CHG + DCDC 5V + LDO 3.3V (+bit4) in PWR_CFG 0x06.
    reg_write(I2C_ADDR_PM1, 0x09, 0x10);
    reg_write(I2C_ADDR_PM1, 0x0A, 0x00);
    bits(I2C_ADDR_PM1, 0x06, 0x17, true);

    pinMode(PIN_LCD_CS, OUTPUT);
    digitalWrite(PIN_LCD_CS, HIGH);

    // IOE1 boots after PM1 powers it: wait up to 500 ms for it to answer
    const uint32_t t0 = millis();
    while (!ack(I2C_ADDR_IOE1) && millis() - t0 < 500) delay(5);

    reg_write(I2C_ADDR_IOE1, 0x23, 0x00);
    bits(I2C_ADDR_IOE1, 0x13, 0x9D, false);
    bits(I2C_ADDR_IOE1, 0x03, 0x9D, true);     // outputs
    bits(I2C_ADDR_IOE1, 0x05, 0x99, true);     // high (panel power on, resets released)
    delay(10);
    bits(I2C_ADDR_IOE1, 0x05, 0x18, false);    // panel + touch reset low
    delay(8);
    bits(I2C_ADDR_IOE1, 0x05, 0x18, true);     // ...and released
    delay(2);
    bits(I2C_ADDR_IOE1, 0x14, 0x02, false);
    bits(I2C_ADDR_IOE1, 0x04, 0x02, true);
    bits(I2C_ADDR_IOE1, 0x06, 0x02, false);

    Serial.print("[board] I2C scan:");
    for (uint8_t a = 0x08; a < 0x78; ++a) if (ack(a)) Serial.printf(" 0x%02X", a);
    Serial.println();

    Wire.setClock(400000);
}

// ---------- Battery (M5PM1) ----------
#include "battery.h"
#define PM1_PWR_SRC  0x04   // [2:0] 0 = 5VIN (USB), 1 = 5VINOUT, 2 = battery
#define PM1_PWR_CFG  0x06   // [0] CHG_EN
#define PM1_GPIO_IN  0x12   // [2] CHG_STAT from the charger, low = charging (M5Unified)
#define PM1_VBAT_L   0x22   // battery mV, little-endian over 0x22/0x23

// Resting 1-cell LiPo curve, mV at 100, 95 ... 0 %. While charging the cell reads ~0.1 V
// high, so the charge stops a little before a true 80 %: on the safe side.
static const uint16_t kCurve[] = { 4200, 4150, 4110, 4080, 4020, 3980, 3950, 3910, 3870, 3850, 3840,
                                   3820, 3800, 3790, 3770, 3750, 3730, 3710, 3690, 3610, 3270 };

static int pct_from_mv(int mv) {
    const int n = sizeof(kCurve) / sizeof(kCurve[0]);
    if (mv >= kCurve[0]) return 100;
    for (int i = 1; i < n; ++i)
        if (mv >= kCurve[i])   // interpolate inside the 5 % step
            return (n - 1 - i) * 5 + 5 * (mv - kCurve[i]) / (kCurve[i - 1] - kCurve[i]);
    return 0;
}

static int s_mv = 0;
static int s_pct = -1;
static bool s_chgEn = false;
static bool s_usb = false;
static bool s_chgStat = false;   // the charger says it is charging right now

// Read the PM1 and apply the charge window. main.cpp calls battery_percent() every 5 s.
static void battery_update() {
    uint8_t v[2], src, cfg, gin;
    if (!reg_read_n(I2C_ADDR_PM1, PM1_VBAT_L, v, 2) || !reg_read(I2C_ADDR_PM1, PM1_PWR_SRC, &src) ||
        !reg_read(I2C_ADDR_PM1, PM1_PWR_CFG, &cfg) || !reg_read(I2C_ADDR_PM1, PM1_GPIO_IN, &gin))
        return;   // keep the last reading
    s_mv = v[0] | (v[1] << 8);
    s_chgStat = !(gin & 0x04);
    s_usb = (src & 0x07) != 2;
    s_pct = battery_present() ? pct_from_mv(s_mv) : -1;
    s_chgEn = cfg & 0x01;
    if (s_pct < 0) return;
    // CHG_EN itself is the hysteresis state, so a reboot mid-window just carries on
    const bool want = s_chgEn ? (s_pct < BATT_CHARGE_OFF_PCT) : (s_pct <= BATT_CHARGE_ON_PCT);
    if (want != s_chgEn) {
        bits(I2C_ADDR_PM1, PM1_PWR_CFG, 0x01, want);
        s_chgEn = want;
        if (!want) s_chgStat = false;   // CHG_STAT lags the switch by a moment
        Serial.printf("[batt] %d%% (%d mV): charging %s\n", s_pct, s_mv, want ? "on" : "off");
    }
}

bool battery_begin() {
    battery_update();
    Serial.printf("[batt] M5PM1: %d mV, %d%%, %s, CHG_EN %d, charger %s\n", s_mv, s_pct,
                  s_usb ? "USB" : "battery", s_chgEn, s_chgStat ? "charging" : "idle");
    return true;
}
bool battery_present()  { return s_mv > 2500; }   // no cell: the ADC reads near 0
int  battery_percent()  { battery_update(); return s_pct; }
bool battery_charging() { return s_chgStat; }
bool battery_vbus_in()  { return s_usb; }
void battery_enable_codec_rail() {}
void battery_power_off() { Serial.println("[batt] power off: not wired on the StopWatch"); }

#endif  // BOARD_STOPWATCH
