# How to Run  BILK Option A (Hybrid, WiFi UDP Leader)

## Roles & Entry Points
- **Leader (ESP32):** `firmware/leader_esp32/Leader_ESP32_AS5600_UDP.ino`
- **Host (RPi/PC):** `tools/host_bridge_udp.py`
- **Follower (Arduino Mega):** `firmware/follower_arduino/Follower_Arduino.ino`

## 1) Leader (ESP32)
- Edit WiFi in the sketch: `WIFI_SSID`, `WIFI_PASS`, `HOST_IP` (Pi/PC).
- Flash to ESP32. It streams UDP :9001 and also mirrors over USB.

## 2) Follower (Arduino Mega)
- Open `Follower_Arduino.ino`, map your PWM/DIR pins and tune `Kp/Ki/Kd`.
- Flash to Mega. It parses `LeaderState` and applies PID @1 kHz.

## 3) Host (RPi/PC)
```bash
python3 -m venv .venv && source .venv/bin/activate
pip install pyserial numpy
python tools/host_bridge_udp.py /dev/ttyUSB_FOLLOWER /dev/ttyUSB_LEADER
```
- The second argument is optional (USB fallback from leader).

## Safety
- Host watchdog: if >100 ms with no forwardable frames  `HOLD`.
- Leader EStop (GPIO 33 LOW) forces `HOLD`.
- Keep a **hardwired** EStop relay that cuts motor power.
