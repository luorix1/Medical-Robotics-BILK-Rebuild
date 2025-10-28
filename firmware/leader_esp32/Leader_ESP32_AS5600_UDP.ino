/* Leader_ESP32_AS5600_UDP_FIXED.ino - Corrected AS5600 Implementation */
#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "protocol.h"

// Hardware Configuration
#define I2C_SDA 21
#define I2C_SCL 22
#define I2C_FREQ 400000  // 400kHz I2C speed
#define TCA9548A_ADDR 0x70  // TCA9548A I2C multiplexer address
static const uint8_t kMuxChannels[4] = {0, 1, 2, 3};
static const uint8_t AS5600_ADDR = 0x36;  // AS5600 I2C address

// Control Pins
#define PIN_BTN_FINE 32
#define PIN_ESTOP_IN 33

// Network Configuration
const char* WIFI_SSID = "YOUR_SSID";
const char* WIFI_PASS = "YOUR_PASSWORD";
IPAddress HOST_IP(192, 168, 1, 100);
const uint16_t HOST_PORT = 9001;

// AS5600 Register Definitions (from datasheet)
#define AS5600_REG_RAW_ANGLE_HI 0x0C  // Raw angle high byte
#define AS5600_REG_RAW_ANGLE_LO 0x0D  // Raw angle low byte
#define AS5600_REG_STATUS 0x0B        // Status register
#define AS5600_REG_AGC 0x1A          // Automatic Gain Control
#define AS5600_REG_MAGNITUDE_HI 0x1B  // Magnitude high byte
#define AS5600_REG_MAGNITUDE_LO 0x1C  // Magnitude low byte

// System Configuration
WiFiUDP Udp;
bool wifi_ok = false;
static const uint32_t kSampleUs = 1000;  // 1kHz sampling rate
static const uint16_t AS5600_RESOLUTION = 4095;  // 12-bit resolution (0-4095)
static const float AS5600_RAD_PER_COUNT = 2.0f * PI / AS5600_RESOLUTION;

// Data Structures
typedef struct { 
    float q[4];    // Joint positions (radians)
    float qd[4];   // Joint velocities (rad/s)
} JointState;

typedef struct {
    float angle_raw;      // Raw angle from encoder
    float angle_unwrapped; // Unwrapped angle for continuous rotation
    float velocity;        // Calculated velocity
    bool valid;           // Encoder data validity
    uint16_t magnitude;   // Magnetic field strength
    uint8_t agc;          // Automatic gain control value
} EncoderState;

// Global Variables
static JointState js;
static EncoderState encoders[4];
static uint16_t lp_tau_ms = 10;  // Low-pass filter time constant
static float last_angle_raw[4] = {0};
static float unwrapped[4] = {0};

// Utility Functions
static inline float wrapPi(float x) {
    while (x <= -PI) x += 2 * PI;
    while (x > PI) x -= 2 * PI;
    return x;
}

static inline float lp_alpha(float dt, float tau) {
    if (tau <= 1e-6f) return 1.0f;
    return dt / (tau + dt);
}

// Multiplexer Control
bool mux_select(uint8_t channel) {
    if (channel > 7) return false;  // TCA9548A supports 8 channels (0-7)
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << channel);  // Set the appropriate bit for channel selection
    return Wire.endTransmission() == 0;
}

// AS5600 Reading Functions
bool as5600_read_register(uint8_t reg, uint8_t* data, uint8_t length) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    
    Wire.requestFrom(AS5600_ADDR, length);
    if (Wire.available() < length) return false;
    
    for (uint8_t i = 0; i < length; i++) {
        data[i] = Wire.read();
    }
    return true;
}

bool as5600_read_raw_angle(uint16_t& raw_angle) {
    uint8_t data[2];
    if (!as5600_read_register(AS5600_REG_RAW_ANGLE_HI, data, 2)) return false;
    
    raw_angle = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

bool as5600_read_status(uint8_t& status) {
    return as5600_read_register(AS5600_REG_STATUS, &status, 1);
}

bool as5600_read_magnitude(uint16_t& magnitude) {
    uint8_t data[2];
    if (!as5600_read_register(AS5600_REG_MAGNITUDE_HI, data, 2)) return false;
    
    magnitude = ((uint16_t)data[0] << 8) | data[1];
    return true;
}

bool as5600_read_agc(uint8_t& agc) {
    return as5600_read_register(AS5600_REG_AGC, &agc, 1);
}

// Comprehensive AS5600 reading with error checking
bool as5600_read_angle_rad(uint8_t encoder_idx, float& angle) {
    if (encoder_idx >= 4) return false;
    
    uint16_t raw_angle;
    uint8_t status;
    uint16_t magnitude;
    uint8_t agc;
    
    // Read raw angle
    if (!as5600_read_raw_angle(raw_angle)) {
        encoders[encoder_idx].valid = false;
        return false;
    }
    
    // Read status register for error checking
    if (!as5600_read_status(status)) {
        encoders[encoder_idx].valid = false;
        return false;
    }
    
    // Check for magnetic field issues
    if (!as5600_read_magnitude(magnitude)) {
        encoders[encoder_idx].valid = false;
        return false;
    }
    
    // Read AGC for additional diagnostics
    as5600_read_agc(agc);
    
    // Store diagnostic information
    encoders[encoder_idx].magnitude = magnitude;
    encoders[encoder_idx].agc = agc;
    
    // Check for magnetic field strength (typical range: 0-4095)
    // Values below 1000 may indicate weak magnetic field
    if (magnitude < 1000) {
        encoders[encoder_idx].valid = false;
        return false;
    }
    
    // Check status register for errors
    // Bit 3: MH (Magnet too strong), Bit 4: ML (Magnet too weak)
    if (status & 0x18) {  // Check bits 3 and 4
        encoders[encoder_idx].valid = false;
        return false;
    }
    
    // Convert raw angle to radians
    angle = (float)raw_angle * AS5600_RAD_PER_COUNT;
    angle = wrapPi(angle);
    
    encoders[encoder_idx].angle_raw = angle;
    encoders[encoder_idx].valid = true;
    
    return true;
}

// Protocol Functions
static const uint8_t PRE[4] = {'B', 'I', 'L', 'K'};

uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    }
    return crc;
}

void build_leader_state(uint32_t t_us, const JointState& s, uint8_t buttons, uint8_t* out, size_t& out_len) {
    struct __attribute__((packed)) P { 
        uint32_t t_us; 
        float q[4]; 
        float qd[4]; 
        uint8_t buttons; 
        uint8_t r[3]; 
    } p;
    
    p.t_us = t_us;
    for (int i = 0; i < 4; i++) {
        p.q[i] = s.q[i];
        p.qd[i] = s.qd[i];
    }
    p.buttons = buttons;
    p.r[0] = p.r[1] = p.r[2] = 0;
    
    uint8_t hdr[4] = {0x01, 0x01, (uint8_t)sizeof(p), (uint8_t)(sizeof(p) >> 8)};
    uint16_t crc = 0xFFFF;
    auto upd = [&](uint8_t b) {
        crc ^= (uint16_t)b << 8;
        for (uint8_t k = 0; k < 8; k++) {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
        }
    };
    
    for (int i = 0; i < 4; i++) upd(hdr[i]);
    const uint8_t* pb = (const uint8_t*)&p;
    for (size_t i = 0; i < sizeof(p); i++) upd(pb[i]);
    
    uint8_t* w = out;
    memcpy(w, PRE, 4); w += 4;
    memcpy(w, hdr, 4); w += 4;
    memcpy(w, &p, sizeof(p)); w += sizeof(p);
    *w++ = (uint8_t)(crc & 0xFF);
    *w++ = (uint8_t)(crc >> 8);
    out_len = (size_t)(w - out);
}

// Diagnostic Functions
void print_encoder_diagnostics() {
    Serial.println("=== AS5600 Encoder Diagnostics ===");
    for (int i = 0; i < 4; i++) {
        Serial.printf("Encoder %d: Valid=%s, Angle=%.3f rad, Mag=%d, AGC=%d\n", 
                     i, 
                     encoders[i].valid ? "YES" : "NO",
                     encoders[i].angle_raw,
                     encoders[i].magnitude,
                     encoders[i].agc);
    }
    Serial.println("==================================");
}

void setup() {
    Serial.begin(2000000);
    Serial.println("BILK Leader ESP32 - AS5600 Fixed Version");
    
    // Initialize pins
    pinMode(PIN_BTN_FINE, INPUT_PULLUP);
    pinMode(PIN_ESTOP_IN, INPUT_PULLUP);
    
    // Initialize I2C
    Wire.begin(I2C_SDA, I2C_SCL);
    Wire.setClock(I2C_FREQ);
    
    // Initialize encoders
    Serial.println("Initializing AS5600 encoders...");
    for (int i = 0; i < 4; i++) {
        mux_select(kMuxChannels[i]);
        delay(10);  // Allow multiplexer to settle
        
        float a = 0;
        if (as5600_read_angle_rad(i, a)) {
            last_angle_raw[i] = a;
            unwrapped[i] = a;
            js.q[i] = a;
            js.qd[i] = 0;
            Serial.printf("Encoder %d initialized: %.3f rad\n", i, a);
        } else {
            Serial.printf("Encoder %d initialization failed!\n", i);
            js.q[i] = 0;
            js.qd[i] = 0;
        }
    }
    
    // Print initial diagnostics
    print_encoder_diagnostics();
    
    // Initialize WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.println("Connecting to WiFi...");
    
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 10000) {
        delay(100);
        Serial.print(".");
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Udp.begin(0);
        wifi_ok = true;
        Serial.printf("\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("\nWiFi connection failed!");
    }
}

void loop() {
    static uint32_t t0 = micros();
    uint32_t t = micros();
    
    if ((uint32_t)(t - t0) >= kSampleUs) {
        float dt = (t - t0) * 1e-6f;
        t0 = t;
        
        // Read all encoders
        for (int i = 0; i < 4; i++) {
            mux_select(kMuxChannels[i]);
            delayMicroseconds(100);  // Allow multiplexer to settle
            
            float a = last_angle_raw[i];
            if (as5600_read_angle_rad(i, a)) {
                // Calculate angle difference with proper unwrapping
                float da = a - last_angle_raw[i];
                if (da > PI) da -= 2 * PI;
                if (da < -PI) da += 2 * PI;
                
                // Update unwrapped angle
                unwrapped[i] += da;
                last_angle_raw[i] = a;
                
                // Calculate velocity with low-pass filtering
                float raw_qd = da / dt;
                float alpha = lp_alpha(dt, lp_tau_ms / 1000.0f);
                js.qd[i] = alpha * raw_qd + (1.0f - alpha) * js.qd[i];
                js.q[i] = wrapPi(a);
            } else {
                // Encoder read failed - maintain last known position
                // In a real system, you might want to set a flag or use a different strategy
                Serial.printf("Encoder %d read failed at t=%u\n", i, t);
            }
        }
        
        // Read buttons
        uint8_t buttons = 0;
        if (digitalRead(PIN_BTN_FINE) == LOW) buttons |= 0x01;
        if (digitalRead(PIN_ESTOP_IN) == LOW) buttons |= 0x02;
        
        // Build and send frame
        uint8_t frame[256];
        size_t flen = 0;
        build_leader_state(t, js, buttons, frame, flen);
        
        // Send via WiFi
        if (wifi_ok) {
            Udp.beginPacket(HOST_IP, HOST_PORT);
            Udp.write(frame, flen);
            Udp.endPacket();
        }
        
        // Send via USB
        Serial.write(frame, flen);
        
        // Periodic WiFi reconnection check
        static uint32_t last_wifi_check = 0;
        if ((millis() - last_wifi_check) > 2000) {
            if (WiFi.status() != WL_CONNECTED) {
                WiFi.disconnect();
                WiFi.begin(WIFI_SSID, WIFI_PASS);
            }
            last_wifi_check = millis();
        }
        wifi_ok = (WiFi.status() == WL_CONNECTED);
        
        // Periodic diagnostics (every 5 seconds)
        static uint32_t last_diag = 0;
        if ((millis() - last_diag) > 5000) {
            print_encoder_diagnostics();
            last_diag = millis();
        }
    }
}
