# 🚗 PBL4 - Smart Autonomous Vehicle with AI

An autonomous vehicle project integrating AI-based traffic light recognition and automatic line following capabilities.

## 📋 Overview

The system consists of 3 main components:
- **ESP32-CAM**: Captures images and sends to server, receives commands from server and forwards to Arduino
- **Python Server**: AI processing for traffic light recognition (MobileNetV3) and lane detection
- **Arduino**: Motor control and line sensor processing

## 🏗️ System Architecture

```
ESP32-CAM ----[WebSocket]----> Python Server (FastAPI)
    |                               |
    |                          [AI Model]
    |                          - Traffic Light Detection
    |                          - Lane Detection
    |                               |
    └--------[UART]--------> Arduino UNO/NANO
                                    |
                              [Motor Driver]
```

## 📁 Project Structure

```
PBL4/
├── Server.py                          # FastAPI server with AI model
├── final_mobilenetv3_small_v7.keras   # Traffic light recognition AI model
├── ESP/
│   ├── ESP_Traffic_Light.ino         # ESP32-CAM code
│   ├── config.h                      # Server and camera configuration
│   └── secrets.h                     # WiFi credentials
└── arduino/
    └── arduino.ino                   # Motor control + line follower code
```

## 🚀 Features

### 1. Traffic Light Detection
- Model: **MobileNetV3-Small** (optimized for edge devices)
- Input: 224x224 RGB images
- Classes: `green`, `red`, `yellow`, `stopsign`, `none`
- Confidence threshold: 50%
- Voting mechanism: Uses 3 most recent predictions to stabilize results

### 2. Lane Detection
- Image processing with OpenCV: thresholding, morphology, contour detection
- Bird's eye view transformation (perspective transform)
- Automatic direction adjustment based on lane position

### 3. Operation Modes
- **Line Follow Mode**: Vehicle automatically follows line, AI only stops when encountering red light/STOP sign
- **Manual/Auto Mode**: Direct control from server

## 🔧 Hardware Configuration

### ESP32-CAM (AI Thinker)
- Camera: OV2640
- WiFi: 2.4GHz
- UART2 (TX: GPIO15, RX: GPIO13) → connects to Arduino

### Arduino UNO/NANO
- Motor Driver: L298N or equivalent
- Line Sensors: 3 sensors (Left, Middle, Right) - Digital
- UART: RX/TX → connects to ESP32

### Motor Driver Connections
```
ENA → Pin 5    IN1 → Pin 6    IN2 → Pin 7
ENB → Pin 10   IN3 → Pin 8    IN4 → Pin 9
```

### Line Sensors
```
LINE_L → Pin 3    (Left sensor)
LINE_M → Pin 12   (Middle sensor)
LINE_R → Pin 2    (Right sensor)
```

## ⚙️ Installation

### 1. Python Server

```bash
# Install dependencies
pip install fastapi uvicorn opencv-python numpy tensorflow websockets

# Run server
python Server.py
```

Server will run on `http://0.0.0.0:8000`

### 2. ESP32-CAM

1. Open `ESP/ESP_Traffic_Light.ino` in Arduino IDE
2. Configure WiFi in `ESP/secrets.h`:
```cpp
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";
```
3. Update server IP in `ESP/config.h`:
```cpp
const char* server_host = "192.168.88.243";  // IP of machine running Python Server
```
4. Upload code to ESP32-CAM

### 3. Arduino

1. Open `arduino/arduino.ino` in Arduino IDE
2. Upload code to Arduino UNO/NANO

## 📡 Communication Protocol

### WebSocket (ESP32 ↔ Server)

**ESP32 → Server**: Binary (JPEG image)
```
Frames sent every 100ms (10 FPS)
```

**Server → ESP32**: JSON
```json
{
  "command": "S",           // S=Stop, A=Allow, F/B/L/R=Manual
  "mode": "line_follow",    // line_follow / auto / manual
  "class": "red",           // green / red / yellow / stopsign / none
  "confidence": 0.95
}
```

### UART (ESP32 ↔ Arduino)

Single character commands:
- `S` - Stop (LINE_FOLLOW mode - red light/STOP sign)
- `A` - Allow (LINE_FOLLOW mode - allow to run)
- `F` - Forward (Manual mode)
- `B` - Backward (Manual mode)
- `L` - Turn Left (Manual mode)
- `R` - Turn Right (Manual mode)

## 🎛️ Tunable Parameters

### Server.py - AI Inference
```python
INFERENCE_INTERVAL = 0.05    # 50ms between inferences
MIN_CONFIDENCE = 0.5         # Confidence threshold
VOTING_WINDOW = 3            # Number of predictions for voting
```

### Server.py - Lane Detection
```python
V_THRESHOLD = 140            # Lane detection threshold (HSV Value)
ROI_TOP_RATIO = 0.5          # Crop bottom half of image
DEADBAND_PCT = 0.03          # Deadband for steering
```

### arduino.ino - Motor Speed
```cpp
const int BASE_SPEED = 255;         // Manual mode speed
const int LINE_FOLLOW_SPEED = 170;  // Line following speed
const int TURN_SPEED = 255;         // Turn speed
```

## 🔍 Operation Flow

1. ESP32-CAM captures image from camera
2. Sends image via WebSocket to Python Server
3. Server processes:
   - Traffic light recognition (if any)
   - Lane detection processing
   - Decides control command
4. Server sends command back to ESP32
5. ESP32 forwards command via UART to Arduino
6. Arduino controls motors based on:
   - Line sensors (LINE_FOLLOW mode)
   - Commands from server (MANUAL/AUTO mode)

## 🛠️ Troubleshooting

### ESP32 cannot connect to WebSocket
- Check server IP in `config.h`
- Ensure server is running and accessible
- Verify WiFi credentials in `secrets.h`

### Vehicle doesn't stop at red lights
- Increase `MIN_CONFIDENCE` in Server.py
- Check environmental lighting conditions
- Review confidence logs on Serial Monitor

### Vehicle turns wrong direction during line following
- Adjust `V_THRESHOLD` (increase if sensors are too sensitive)
- Calibrate `DEADBAND_PCT`
- Check if line sensors are functioning correctly

## 📊 Model Information

- **Architecture**: MobileNetV3-Small
- **Input**: 224×224×3 (RGB)
- **Output**: 5 classes
- **Size**: ~11MB
- **Framework**: TensorFlow/Keras

## 🎯 Future Improvements

- [ ] Add object detection (obstacle detection)
- [ ] Integrate GPS navigation
- [ ] Web dashboard for real-time monitoring
- [ ] Train model with larger dataset
- [ ] Optimize model for inference speed

**Note**: This is an educational/research project. Thorough testing in a safe environment is required before real-world deployment.
