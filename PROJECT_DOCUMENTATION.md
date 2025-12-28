# 🚗 IoT Race Car - Complete Project Documentation

**Project Name:** Bluetooth-Controlled Race Car with Obstacle Avoidance  
**Microcontroller:** ESP32 Dev Module  
**Date:** December 28, 2025  
**Status:** Documentation & Planning Phase

---

## 📑 Table of Contents
1. [System Architecture](#system-architecture)
2. [Hardware Components](#hardware-components)
3. [Pin Configuration](#pin-configuration)
4. [Component Wiring Diagram](#component-wiring-diagram)
5. [Software Architecture](#software-architecture)
6. [Main Loop Flow](#main-loop-flow)
7. [Communication Protocol](#communication-protocol)
8. [State Machine](#state-machine)
9. [Function Specifications](#function-specifications)
10. [Testing Strategy](#testing-strategy)

---

## 🏗️ System Architecture

```mermaid
graph TB
    subgraph "Control System"
        BT[Bluetooth Module<br/>DabbleESP32]
        ESP[ESP32 Dev Module<br/>Main Controller]
        US[HC-SR04<br/>Ultrasonic Sensor]
        BUZ[Active Buzzer<br/>Warning Alarm]
    end
    
    subgraph "Power System"
        USB[USB Power<br/>5V for ESP32]
        BAT[7.4V Battery<br/>Motor Power]
    end
    
    subgraph "Motor Control"
        L298[L298N Driver<br/>H-Bridge]
        ML[Left DC Motor]
        MR[Right DC Motor]
    end
    
    subgraph "External Interface"
        APP[Dabble App<br/>Gamepad Interface]
    end
    
    APP -->|Commands: F,B,L,R,S| BT
    BT <-->|Serial Data| ESP
    US -->|Distance Data| ESP
    ESP -->|Control Signals| L298
    ESP -->|Alarm Signal| BUZ
    ESP -->|Status Messages| BT
    BT -->|Feedback| APP
    L298 -->|PWM + Direction| ML
    L298 -->|PWM + Direction| MR
    USB -->|5V Logic| ESP
    USB -->|5V Sensor| US
    USB -->|5V Logic| L298
    BAT -->|7.4V Motor| L298
    
    style ESP fill:#4CAF50,stroke:#333,stroke-width:3px,color:#fff
    style L298 fill:#2196F3,stroke:#333,stroke-width:2px,color:#fff
    style US fill:#FF9800,stroke:#333,stroke-width:2px,color:#fff
    style BT fill:#9C27B0,stroke:#333,stroke-width:2px,color:#fff
```

---

## 🔧 Hardware Components

### **1. ESP32 Dev Module**
- **Function:** Main microcontroller
- **Features:**
  - Dual-core 240MHz processor
  - Built-in Bluetooth Classic & BLE
  - 34 GPIO pins
  - PWM support on all digital pins
  - 5V tolerant (with level shifters)
- **Power:** 5V via USB or VIN pin
- **Operating Voltage:** 3.3V logic

### **2. L298N Motor Driver**
- **Function:** H-Bridge motor controller
- **Specifications:**
  - Dual H-bridge circuit
  - Max motor voltage: 46V
  - Peak current: 2A per channel
  - Logic voltage: 5V
  - PWM frequency support: 0-40kHz
- **Connections:**
  - IN1-IN4: Direction control
  - ENA, ENB: Speed control (PWM)
  - OUT1-OUT4: Motor terminals

### **3. HC-SR04 Ultrasonic Sensor**
- **Function:** Distance measurement
- **Specifications:**
  - Range: 2cm - 400cm
  - Accuracy: ±3mm
  - Measuring angle: 15°
  - Trigger: 10µs pulse
  - Echo: Returns pulse width
  - Power: 5V, <15mA
- **Formula:** Distance (cm) = Echo Time (µs) / 58

### **4. Active Buzzer**
- **Function:** Audible warning for obstacle detection
- **Type:** Active buzzer (has built-in oscillator)
- **Specifications:**
  - Operating Voltage: 3-5V DC
  - Current Draw: ~20-30mA
  - Sound Level: ~85dB at 10cm
  - Frequency: ~2kHz (fixed)
  - Polarity: Positive (+) and Negative (-)
- **Resistor Required:** ❌ **NO** - ESP32's 3.3V GPIO is safe for most active buzzers
- **Note:** If buzzer is rated for 5V only, use a 100Ω resistor to limit current

### **5. DC Motors (x2)**
- **Function:** Vehicle propulsion
- **Specifications:**
  - Voltage: 3V - 12V
  - Recommended: 6V - 7.4V
  - Current: ~200-300mA each (no load)
  - Speed control via PWM
  - Bidirectional operation

### **6. Power Supply**
- **ESP32:** USB cable (5V, 500mA) or external 5V source
- **Motors:** 7.4V LiPo battery (2S) or 6x AA batteries
- **Common Ground:** Essential for all components

---

## 📌 Pin Configuration

```mermaid
graph LR
    subgraph "ESP32 GPIO Mapping"
        subgraph "Motor Control - Left"
            G16[GPIO 16<br/>IN1]
            G17[GPIO 17<br/>IN2]
            G4[GPIO 4<br/>ENA PWM]
        end
        
        subgraph "Motor Control - Right"
            G18[GPIO 18<br/>IN3]
            G19[GPIO 19<br/>IN4]
            G5[GPIO 5<br/>ENB PWM]
        end
        
        subgraph "Ultrasonic Sensor"
            G13[GPIO 13<br/>TRIG]
            G12[GPIO 12<br/>ECHO]
        end
        
        subgraph "Buzzer Alarm"
            G14[GPIO 14<br/>BUZZER]
        end
        
        subgraph "Power & Debug"
            V5[5V Pin<br/>To L298N]
            GND[GND Pin<br/>Common Ground]
            TX[Serial TX<br/>Debug Monitor]
            RX[Serial RX<br/>Debug Monitor]
        end
    end
    
    style G16 fill:#FF5722,color:#fff
    style G17 fill:#FF5722,color:#fff
    style G4 fill:#FFC107,color:#000
    style G18 fill:#2196F3,color:#fff
    style G19 fill:#2196F3,color:#fff
    style G5 fill:#03A9F4,color:#fff
    style G13 fill:#4CAF50,color:#fff
    style G12 fill:#8BC34A,color:#fff
```

### **Pin Assignment Table**

| Component | Pin Name | ESP32 GPIO | Signal Type | Description |
|-----------|----------|------------|-------------|-------------|
| **Left Motor** | IN1 | GPIO 16 | Digital Out | Forward control |
| | IN2 | GPIO 17 | Digital Out | Backward control |
| | ENA | GPIO 4 | PWM Out | Speed control (0-255) |
| **Right Motor** | IN3 | GPIO 18 | Digital Out | Forward control |
| | IN4 | GPIO 19 | Digital Out | Backward control |
| | ENB | GPIO 5 | PWM Out | Speed control (0-255) |
| **Ultrasonic** | TRIG | GPIO 13 | Digital Out | 10µs trigger pulse |
| | ECHO | GPIO 12 | Digital In | Distance echo signal |
| **Buzzer** | + (Positive) | GPIO 14 | Digital Out | Warning alarm signal |
| | - (Negative) | GND Pin | Ground | Buzzer ground |
| **Power** | 5V | 5V Pin | Power | Logic & sensor power |
| | GND | GND Pin | Ground | Common ground |

---

## 🔌 Component Wiring Diagram

```mermaid
graph TD
    subgraph "ESP32 Dev Module"
        ESP_5V[5V Pin]
        ESP_GND[GND Pin]
        ESP_G16[GPIO 16]
        ESP_G17[GPIO 17]
        ESP_G4[GPIO 4]
        ESP_G18[GPIO 18]
        ESP_G19[GPIO 19]
        ESP_G5[GPIO 5]
        ESP_G13[GPIO 13]
        ESP_G12[GPIO 12]
    end
    
    subgraph "L298N Motor Driver"
        L298_IN1[IN1]
        L298_IN2[IN2]
        L298_IN3[IN3]
        L298_IN4[IN4]
        L298_ENA[ENA]
        L298_ENB[ENB]
        L298_5V[+5V Logic]
        L298_GND[GND]
        L298_12V[+12V Motor]
        L298_MGND[Motor GND]
        L298_OUT1[OUT1]
        L298_OUT2[OUT2]
        L298_OUT3[OUT3]
        L298_OUT4[OUT4]
    end
    
    subgraph "HC-SR04"
        US_VCC[VCC]
        US_TRIG[TRIG]
        US_ECHO[ECHO]
        US_GND[GND]
    end
    
    subgraph "Active Buzzer"
        BUZ_POS[+ Positive]
        BUZ_NEG[- Negative]
    end
    
    subgraph "Motors"
        MOTOR_L1[Left Motor +]
        MOTOR_L2[Left Motor -]
        MOTOR_R1[Right Motor +]
        MOTOR_R2[Right Motor -]
    end
    
    subgraph "Power Sources"
        USB[USB 5V]
        BATTERY[7.4V Battery]
        BAT_POS[Battery +]
        BAT_NEG[Battery -]
    end
    
    %% ESP32 to L298N
    ESP_G16 -->|Direction| L298_IN1
    ESP_G17 -->|Direction| L298_IN2
    ESP_G4 -->|PWM Speed| L298_ENA
    ESP_G18 -->|Direction| L298_IN3
    ESP_G19 -->|Direction| L298_IN4
    ESP_G5 -->|PWM Speed| L298_ENB
    ESP_5V -->|5V Logic| L298_5V
    ESP_GND -->|Ground| L298_GND
    
    %% ESP32 to HC-SR04
    ESP_5V -->|5V Power| US_VCC
    ESP_G13 -->|Trigger| US_TRIG
    ESP_G12 -->|Echo| US_ECHO
    ESP_GND -->|Ground| US_GND
    
    %% ESP32 to Buzzer (No Resistor Needed)
    ESP_G14[GPIO 14] -->|3.3V Signal| BUZ_POS
    ESP_GND -->|Ground| BUZ_NEG
    
    %% L298N to Motors
    L298_OUT1 -->|Motor A| MOTOR_L1
    L298_OUT2 -->|Motor A| MOTOR_L2
    L298_OUT3 -->|Motor B| MOTOR_R1
    L298_OUT4 -->|Motor B| MOTOR_R2
    
    %% Power Connections
    USB -->|5V| ESP_5V
    BAT_POS -->|7.4V| L298_12V
    BAT_NEG -->|Ground| L298_MGND
    L298_MGND -->|Common GND| ESP_GND
    
    style ESP_5V fill:#F44336,color:#fff
    style ESP_GND fill:#000,color:#fff
    style L298_5V fill:#F44336,color:#fff
    style L298_12V fill:#FF0000,color:#fff
    style US_VCC fill:#F44336,color:#fff
```

### **Wiring Checklist**
- [ ] ESP32 powered via USB (5V)
- [ ] ESP32 GND → L298N GND → HC-SR04 GND → Battery GND (common ground)
- [ ] ESP32 5V → L298N 5V logic pin
- [ ] ESP32 5V → HC-SR04 VCC
- [ ] Battery 7.4V → L298N +12V motor power
- [ ] All 6 motor control pins (IN1-IN4, ENA, ENB) connected
- [ ] Both ultrasonic pins (TRIG, ECHO) connected
- [ ] Buzzer positive (+) to GPIO 14, negative (-) to GND
- [ ] Left motor to OUT1 & OUT2
- [ ] Right motor to OUT3 & OUT4

---

## 💻 Software Architecture

```mermaid
graph TB
    subgraph "Software Layers"
        subgraph "Application Layer"
            BT_CMD[Bluetooth Command Handler]
            LOGIC[Main Decision Logic]
            SAFETY[Safety Override System]
        end
        
        subgraph "Driver Layer"
            BT_DRV[DabbleESP32 Driver]
            MOTOR_DRV[Motor Control Driver]
            SENSOR_DRV[Ultrasonic Sensor Driver]
        end
        
        subgraph "Hardware Abstraction"
            GPIO[GPIO Control]
            PWM[PWM Generator]
            TIMER[Timing Functions]
        end
    end
    
    BT_CMD -->|Parse Commands| LOGIC
    SENSOR_DRV -->|Distance Data| SAFETY
    SAFETY -->|Override| LOGIC
    LOGIC -->|Movement Commands| MOTOR_DRV
    LOGIC -->|Feedback| BT_DRV
    
    BT_DRV --> GPIO
    MOTOR_DRV --> GPIO
    MOTOR_DRV --> PWM
    SENSOR_DRV --> GPIO
    SENSOR_DRV --> TIMER
    
    style SAFETY fill:#F44336,color:#fff
    style LOGIC fill:#4CAF50,color:#fff
    style BT_CMD fill:#9C27B0,color:#fff
```

---

## 🔄 Main Loop Flow

```mermaid
flowchart TD
    START([Program Start]) --> SETUP[Setup Function]
    SETUP --> INIT_SERIAL[Initialize Serial Monitor]
    INIT_SERIAL --> INIT_BT[Initialize Dabble Bluetooth<br/>Name: IoT_RaceCar]
    INIT_BT --> INIT_PINS[Configure GPIO Pins<br/>Motors & Sensor]
    INIT_PINS --> INIT_MOTORS[Stop All Motors<br/>Safe Initial State]
    INIT_MOTORS --> PRINT_READY[Print Ready Message]
    PRINT_READY --> LOOP_START([Enter Main Loop])
    
    LOOP_START --> PROCESS_DABBLE[Process Dabble Input<br/>Call processInput]
    PROCESS_DABBLE --> CHECK_BT{Command<br/>Received?}
    CHECK_BT -->|Yes| READ_CMD[Read Gamepad Command<br/>Store in lastCommand]
    CHECK_BT -->|No| MEASURE
    READ_CMD --> MEASURE[Measure Distance<br/>getDistance function]
    
    MEASURE --> CHECK_DIST{Distance less than<br/>STOPPING_DISTANCE?}
    
    CHECK_DIST -->|Yes - DANGER!| EMERGENCY[Emergency Stop]
    EMERGENCY --> STOP_MOTORS[Call stopCar function]
    STOP_MOTORS --> ACTIVATE_BUZZER[Activate Buzzer<br/>digitalWrite HIGH]
    ACTIVATE_BUZZER --> SEND_WARNING[Print: OBSTACLE! STOPPED<br/>to Serial Monitor]
    SEND_WARNING --> LOOP_START
    
    CHECK_DIST -->|No - Safe| DEACTIVATE_BUZZER[Deactivate Buzzer<br/>digitalWrite LOW]
    DEACTIVATE_BUZZER --> PARSE_CMD{Parse<br/>lastCommand}
    
    PARSE_CMD -->|'F'| CMD_F[Call moveForward function]
    PARSE_CMD -->|'B'| CMD_B[Call moveBackward function]
    PARSE_CMD -->|'L'| CMD_L[Call turnLeft function]
    PARSE_CMD -->|'R'| CMD_R[Call turnRight function]
    PARSE_CMD -->|'S'| CMD_S[Call stopCar function]
    PARSE_CMD -->|Other| LOOP_START
    
    CMD_F --> LOOP_START
    CMD_B --> LOOP_START
    CMD_L --> LOOP_START
    CMD_R --> LOOP_START
    CMD_S --> LOOP_START
    
    style START fill:#4CAF50,color:#fff
    style LOOP_START fill:#2196F3,color:#fff
    style EMERGENCY fill:#F44336,color:#fff
    style CHECK_DIST fill:#FF9800,color:#fff
    style PARSE_CMD fill:#9C27B0,color:#fff
```

---

## 📡 Communication Protocol

```mermaid
sequenceDiagram
    participant App as Dabble App
    participant BT as Bluetooth Module
    participant ESP as ESP32 Controller
    participant Sensor as HC-SR04
    participant Motors as Motor Driver
    
    App->>BT: Gamepad Button Press (Forward)
    BT->>ESP: Dabble.processInput()
    ESP->>ESP: Read Gamepad State
    ESP->>ESP: Store Command
    
    ESP->>Sensor: Trigger Pulse (10µs)
    Sensor->>ESP: Echo Response
    ESP->>ESP: Calculate Distance
    
    alt Distance >= 20cm (Safe)
        ESP->>Motors: Execute Forward Command
        Motors->>Motors: Both Motors Forward
        Note over ESP,Motors: Car moves forward
    else Distance < 20cm (Obstacle)
        ESP->>Motors: Emergency Stop
        Motors->>Motors: Both Motors Stop
        ESP->>ESP: Activate Buzzer (HIGH)
        Note over ESP: Audible warning beeps
        ESP->>ESP: Print "OBSTACLE! STOPPED"<br/>to Serial Monitor
        Note over ESP: Debug message for<br/>developer monitoring
    end
    
    Note over ESP: Loop repeats continuously<br/>No delay, non-blocking
```

### **Dabble Gamepad Command Mapping**

| Dabble Button | Command | Action | Motor States |
|---------------|---------|--------|--------------|
| **Up Arrow** | Forward | Move forward | Left: FWD, Right: FWD |
| **Down Arrow** | Backward | Move backward | Left: BACK, Right: BACK |
| **Left Arrow** | Left | Turn left | Left: STOP, Right: FWD |
| **Right Arrow** | Right | Turn right | Left: FWD, Right: STOP |
| **No Button Pressed** | Stop | Stop all motors | Both: STOP |

**Note:** Commands are read from Dabble's Gamepad module. The ESP32 continuously checks button states via `Dabble.processInput()`.

### **Bluetooth Messages**
- **From ESP32 (via Serial Monitor):**
  - `"IoT Race Car Ready!"` - System startup
  - `"OBSTACLE! STOPPED"` - Obstacle detected
  - `"Command: X"` - Command confirmation (optional debug)

**Note:** Status messages are sent to Serial Monitor for debugging. The Dabble app displays real-time gamepad input but does not receive text feedback from ESP32.

---

## 🎯 State Machine

```mermaid
stateDiagram-v2
    [*] --> Initialization
    Initialization --> Idle: Setup Complete
    
    Idle --> CheckingDistance: Loop Cycle
    
    CheckingDistance --> ObstacleDetected: Distance < 20cm
    CheckingDistance --> ProcessCommand: Distance >= 20cm
    
    ObstacleDetected --> EmergencyStop
    EmergencyStop --> SendWarning
    SendWarning --> CheckingDistance
    
    ProcessCommand --> MovingForward: Command 'F'
    ProcessCommand --> MovingBackward: Command 'B'
    ProcessCommand --> TurningLeft: Command 'L'
    ProcessCommand --> TurningRight: Command 'R'
    ProcessCommand --> Stopped: Command 'S'
    ProcessCommand --> Stopped: No Command
    
    MovingForward --> CheckingDistance
    MovingBackward --> CheckingDistance
    TurningLeft --> CheckingDistance
    TurningRight --> CheckingDistance
    Stopped --> CheckingDistance
    
    note right of ObstacleDetected
        Priority Override
        Stops all movement
        regardless of command
    end note
    
    note right of ProcessCommand
        Commands only execute
        when path is clear
    end note
```

---

## ⚙️ Function Specifications

### **1. Setup Functions**

```cpp
void setup()
```
**Purpose:** Initialize all hardware components and communication  
**Actions:**
- Initialize Serial Monitor (115200 baud)
- Initialize Dabble Bluetooth with device name "IoT_RaceCar" using `Dabble.begin()`
- Set all motor pins as OUTPUT
- Set ultrasonic pins (TRIG: OUTPUT, ECHO: INPUT)
- Set buzzer pin as OUTPUT
- Stop all motors (safe initial state)
- Print ready message for debugging

---

### **2. Sensor Functions**

```cpp
long getDistance()
```
**Purpose:** Measure distance using HC-SR04 ultrasonic sensor  
**Returns:** Distance in centimeters (long)  
**Algorithm:**
1. Set TRIG pin LOW (2µs)
2. Set TRIG pin HIGH (10µs)
3. Set TRIG pin LOW
4. Measure ECHO pulse duration using `pulseIn()`
5. Calculate: distance = duration / 58
6. Return distance value

**Non-blocking:** Uses `pulseIn()` with timeout to prevent hanging

---

### **3. Motor Control Functions**

```mermaid
graph LR
    subgraph "Motor Control API"
        A[stopCar] --> A1[Set all IN pins LOW<br/>Keep PWM active]
        B[moveForward] --> B1[IN1=HIGH, IN2=LOW<br/>IN3=HIGH, IN4=LOW]
        C[moveBackward] --> C1[IN1=LOW, IN2=HIGH<br/>IN3=LOW, IN4=HIGH]
        D[turnLeft] --> D1[IN1=LOW, IN2=LOW<br/>IN3=HIGH, IN4=LOW]
        E[turnRight] --> E1[IN1=HIGH, IN2=LOW<br/>IN3=LOW, IN4=LOW]
    end
    
    subgraph "Buzzer Control API"
        F[activateBuzzer] --> F1[digitalWrite HIGH<br/>Sound alarm]
        G[deactivateBuzzer] --> G1[digitalWrite LOW<br/>Silence alarm]
    end
    
    style A fill:#F44336,color:#fff
    style B fill:#4CAF50,color:#fff
    style C fill:#FF9800,color:#fff
    style D fill:#2196F3,color:#fff
    style E fill:#9C27B0,color:#fff
    style F fill:#FF5722,color:#fff
    style G fill:#607D8B,color:#fff
```

#### **a) void stopCar()**
```
IN1 = LOW
IN2 = LOW
IN3 = LOW
IN4 = LOW
ENA = MOTOR_SPEED (PWM active)
ENB = MOTOR_SPEED (PWM active)
```

#### **b) void moveForward()**
```
Left Motor:  IN1 = HIGH, IN2 = LOW (Forward)
Right Motor: IN3 = HIGH, IN4 = LOW (Forward)
ENA = MOTOR_SPEED
ENB = MOTOR_SPEED
```

#### **c) void moveBackward()**
```
Left Motor:  IN1 = LOW, IN2 = HIGH (Reverse)
Right Motor: IN3 = LOW, IN4 = HIGH (Reverse)
ENA = MOTOR_SPEED
ENB = MOTOR_SPEED
```

#### **d) void turnLeft()**
```
Left Motor:  IN1 = LOW, IN2 = LOW (Stop)
Right Motor: IN3 = HIGH, IN4 = LOW (Forward)
ENA = MOTOR_SPEED
ENB = MOTOR_SPEED
```

#### **e) void turnRight()**
```
Left Motor:  IN1 = HIGH, IN2 = LOW (Forward)
Right Motor: IN3 = LOW, IN4 = LOW (Stop)
ENA = MOTOR_SPEED
ENB = MOTOR_SPEED
```

#### **f) void activateBuzzer()**
```
digitalWrite(BUZZER_PIN, HIGH)
// Turns buzzer ON - produces continuous beep
// Called when obstacle detected
```

#### **g) void deactivateBuzzer()**
```
digitalWrite(BUZZER_PIN, LOW)
// Turns buzzer OFF - silence
// Called when path is clear
```

---

### **4. Communication Functions**

```cpp
// Process Dabble input (call in main loop)
Dabble.processInput();

// Read gamepad commands
if (GamePad.isUpPressed()) {
    lastCommand = 'F';  // Forward
} else if (GamePad.isDownPressed()) {
    lastCommand = 'B';  // Backward
} else if (GamePad.isLeftPressed()) {
    lastCommand = 'L';  // Left
} else if (GamePad.isRightPressed()) {
    lastCommand = 'R';  // Right
} else {
    lastCommand = 'S';  // Stop
}

// Send status message (to Serial Monitor)
Serial.println("OBSTACLE! STOPPED");
```

---

## 🧪 Testing Strategy

```mermaid
graph TD
    subgraph "Testing Phases"
        T1[Phase 1: Component Tests]
        T2[Phase 2: Integration Tests]
        T3[Phase 3: System Tests]
        T4[Phase 4: Race Track Tests]
    end
    
    T1 --> T1A[Test 1: Bluetooth Connection]
    T1 --> T1B[Test 2: Ultrasonic Reading]
    T1 --> T1C[Test 3: Motor Control]
    
    T2 --> T2A[Test 4: Command Execution]
    T2 --> T2B[Test 5: Obstacle Detection]
    T2 --> T2C[Test 6: Emergency Stop]
    
    T3 --> T3A[Test 7: Continuous Operation]
    T3 --> T3B[Test 8: Response Time]
    T3 --> T3C[Test 9: Edge Cases]
    
    T4 --> T4A[Test 10: Track Navigation]
    T4 --> T4B[Test 11: Turn Accuracy]
    T4 --> T4C[Test 12: Speed Tuning]
    
    style T1 fill:#4CAF50,color:#fff
    style T2 fill:#2196F3,color:#fff
    style T3 fill:#FF9800,color:#fff
    style T4 fill:#9C27B0,color:#fff
```

### **Test Cases**

#### **Phase 1: Component Tests**
1. **Dabble Bluetooth Connection Test**
   - Install Dabble app on Android/iOS device
   - Upload code to ESP32 and open Serial Monitor
   - Open Dabble app and enable Bluetooth
   - Select "IoT_RaceCar" from device list
   - Open Gamepad module in Dabble app
   - Press buttons and verify Serial Monitor shows command reception
   - Verify ESP32 responds to button presses

2. **Ultrasonic Sensor Test**
   - Place object at known distance (10cm, 20cm, 30cm)
   - Print distance values to Serial Monitor
   - Verify accuracy within ±3cm

3. **Motor Control Test**
   - Test each function individually: F, B, L, R, S
   - Verify correct rotation direction
   - Check PWM speed control
   - Measure current draw (should be <500mA per motor)

4. **Buzzer Test**
   - Set buzzer pin HIGH in setup, verify sound
   - Set buzzer pin LOW, verify silence
   - Test continuous beeping (on/off cycle)
   - Verify buzzer audible from 3+ meters

#### **Phase 2: Integration Tests**
5. **Command Execution Test**
   - Press each button on Dabble gamepad (Up, Down, Left, Right)
   - Verify correct motor response for each direction
   - Release button and verify car stops
   - Check command persistence (car continues movement while button held)

6. **Obstacle Detection Test**
   - Press forward button on Dabble gamepad
   - Place hand at 25cm - car should move forward
   - Place hand at 15cm - car should stop immediately
   - Verify "OBSTACLE! STOPPED" message in Serial Monitor
   - **Verify buzzer sounds when obstacle detected**
   - **Verify buzzer stops when obstacle removed**

7. **Emergency Stop Test**
   - Press forward button on Dabble gamepad
   - Move obstacle into path (<20cm)
   - Verify immediate stop (response time <100ms)
   - **Verify buzzer activates immediately**
   - Remove obstacle and verify car can move again

#### **Phase 3: System Tests**
8. **Continuous Operation Test**
   - Run for 10 minutes with random commands
   - Verify no crashes or hangs
   - Check for memory leaks

9. **Response Time Test**
   - Measure command-to-action latency
   - Measure obstacle detection latency
   - Target: <50ms for both

10. **Edge Cases Test**
   - Very close obstacles (2-5cm)
   - Objects at maximum range (400cm)
   - Rapid command changes
   - Continuous forward with obstacle

#### **Phase 4: Race Track Tests**
11. **Track Navigation**
    - Test on actual race track
    - Navigate corners and straightaways
    - Verify no false obstacle detections

12. **Turn Accuracy**
    - Measure turn radius
    - Adjust motor speeds if needed
    - Test left/right turn consistency

13. **Speed Tuning**
    - Adjust MOTOR_SPEED constant
    - Find optimal speed for:
      - Maximum velocity
      - Control precision
      - Battery efficiency

---

## 🔢 Constants & Configuration

```cpp
// Pin Definitions
#define MOTOR_LEFT_IN1    16
#define MOTOR_LEFT_IN2    17
#define MOTOR_LEFT_ENA    4
#define MOTOR_RIGHT_IN3   18
#define MOTOR_RIGHT_IN4   19
#define MOTOR_RIGHT_ENB   5
#define ULTRASONIC_TRIG   13
#define ULTRASONIC_ECHO   12
#define BUZZER_PIN        14

// Configuration Constants
#define STOPPING_DISTANCE_CM  20    // Obstacle detection threshold
#define MOTOR_SPEED          200    // PWM value (0-255)
#define SERIAL_BAUD         115200  // Debug monitor speed
#define BT_DEVICE_NAME "IoT_RaceCar" // Bluetooth name
#define PULSE_TIMEOUT       30000   // Ultrasonic timeout (µs)
```

### **Tunable Parameters**

| Parameter | Default | Range | Purpose |
|-----------|---------|-------|---------|
| `STOPPING_DISTANCE_CM` | 20 | 5-50 | Obstacle safety margin |
| `MOTOR_SPEED` | 200 | 100-255 | Motor PWM duty cycle |
| `PULSE_TIMEOUT` | 30000 | 20000-40000 | Sensor timeout (µs) |

---

## 📊 Performance Specifications

```mermaid
graph LR
    subgraph "Performance Metrics"
        A[Command Latency<br/><50ms] 
        B[Obstacle Response<br/><100ms]
        C[Loop Frequency<br/>>100Hz]
        D[Bluetooth Range<br/>~10 meters]
        E[Battery Life<br/>~30 minutes]
    end
    
    style A fill:#4CAF50,color:#fff
    style B fill:#F44336,color:#fff
    style C fill:#2196F3,color:#fff
    style D fill:#9C27B0,color:#fff
    style E fill:#FF9800,color:#fff
```

### **Expected Performance**
- **Command Response Time:** <50ms (from Bluetooth receive to motor action)
- **Obstacle Detection Time:** <100ms (from object approach to stop)
- **Main Loop Frequency:** >100Hz (>100 cycles per second)
- **Bluetooth Range:** ~10 meters (line of sight)
- **Ultrasonic Range:** 2cm - 400cm (effective: 5-200cm)
- **Battery Runtime:** 20-40 minutes (depends on battery capacity)
- **Motor Speed Control:** 0-255 PWM levels
- **Turn Radius:** ~30-50cm (depends on speed and turn logic)

---

## 🚨 Safety Features

```mermaid
graph TD
    subgraph "Safety Mechanisms"
        S1[Obstacle Override]
        S2[Initial Safe State]
        S3[Common Ground]
        S4[PWM Speed Limit]
        S5[Non-blocking Code]
    end
    
    S1 --> S1A[Absolute priority<br/>Stops all movement]
    S2 --> S2A[Motors stopped<br/>at startup]
    S3 --> S3A[Prevents voltage<br/>spikes]
    S4 --> S4A[Limits motor<br/>current]
    S5 --> S5A[Always responsive<br/>to obstacles]
    
    style S1 fill:#F44336,color:#fff
    style S2 fill:#FF9800,color:#fff
    style S3 fill:#4CAF50,color:#fff
    style S4 fill:#2196F3,color:#fff
    style S5 fill:#9C27B0,color:#fff
```

1. **Obstacle Priority Override:** Distance check has absolute priority over all commands
2. **Safe Initial State:** Motors stopped in setup before accepting commands
3. **Common Ground:** All components share common ground to prevent electrical issues
4. **PWM Speed Limiting:** Maximum speed capped at safe level
5. **Non-blocking Code:** No delays in main loop ensures instant obstacle response
6. **Battery Protection:** Separate motor battery prevents ESP32 brownouts
7. **Timeout Protection:** Ultrasonic sensor has timeout to prevent code hanging

---

## 🛠️ Troubleshooting Guide

```mermaid
graph TD
    START{Issue Type?}
    
    START --> BT_ISSUE[Bluetooth Issues]
    START --> MOTOR_ISSUE[Motor Issues]
    START --> SENSOR_ISSUE[Sensor Issues]
    START --> POWER_ISSUE[Power Issues]
    
    BT_ISSUE --> BT1{Can't find device?}
    BT1 -->|Yes| BT1A[Check ESP32 powered<br/>Verify code uploaded<br/>Check BT name]
    
    BT_ISSUE --> BT2{Commands not working?}
    BT2 -->|Yes| BT2A[Check Serial Monitor<br/>Verify command characters<br/>Test with Serial Monitor]
    
    MOTOR_ISSUE --> M1{Motors not spinning?}
    M1 -->|Yes| M1A[Check battery voltage<br/>Verify L298N connections<br/>Test with multimeter]
    
    MOTOR_ISSUE --> M2{Wrong direction?}
    M2 -->|Yes| M2A[Swap motor wires<br/>Or change IN1/IN2 logic]
    
    SENSOR_ISSUE --> S1{No distance reading?}
    S1 -->|Yes| S1A[Check sensor power<br/>Verify TRIG/ECHO pins<br/>Test with known distance]
    
    SENSOR_ISSUE --> S2{Inaccurate readings?}
    S2 -->|Yes| S2A[Check sensor mounting<br/>Avoid ultrasonic interference<br/>Verify 5V power stable]
    
    POWER_ISSUE --> P1{ESP32 restarting?}
    P1 -->|Yes| P1A[Use separate motor battery<br/>Check common ground<br/>Add capacitor to motor]
    
    style BT_ISSUE fill:#9C27B0,color:#fff
    style MOTOR_ISSUE fill:#F44336,color:#fff
    style SENSOR_ISSUE fill:#FF9800,color:#fff
    style POWER_ISSUE fill:#4CAF50,color:#fff
```

### **Common Issues & Solutions**

| Problem | Possible Cause | Solution |
|---------|---------------|----------|
| Can't connect to Dabble | Code not uploaded / wrong name / library not installed | Re-upload code, install DabbleESP32 library, check device name in Dabble app |
| Dabble buttons not working | Library not initialized / processInput() not called | Ensure Dabble.begin() in setup() and Dabble.processInput() in loop() |
| Motors not spinning | No battery power / loose wires | Check 7.4V battery, verify all connections |
| One motor not working | Loose connection / dead motor | Check specific motor wiring and connections |
| Wrong motor direction | Reversed polarity | Swap motor wires at L298N terminals |
| Sensor always 0 | No power / wrong pins | Verify 5V to sensor, check TRIG/ECHO pins |
| ESP32 keeps resetting | Motors drawing too much current | Use separate battery for motors |
| Car won't move forward | Obstacle always detected | Check sensor readings in Serial Monitor |
| Slow response | Code has delays | Remove all delay() from main loop |

---

## 📦 Bill of Materials (BOM)

| Qty | Component | Specifications | Approx. Cost |
|-----|-----------|----------------|--------------|
| 1 | ESP32 Dev Module | 38-pin, USB-C preferred | $6-10 |
| 1 | L298N Motor Driver | Dual H-Bridge | $3-5 |
| 2 | DC Geared Motors | 3-12V, with wheels | $5-10 |
| 1 | HC-SR04 Ultrasonic | Distance sensor | $2-3 |
| 1 | Active Buzzer | 3-5V DC, 85dB | $1-2 |
| 1 | 7.4V LiPo Battery | 2S, 1000-2500mAh | $10-20 |
| 1 | Battery Connector | XT60 or JST | $1-2 |
| 1 | Chassis | Acrylic or 3D printed | $5-15 |
| 1 | USB Cable | Micro-USB or USB-C | $2-5 |
| 20 | Jumper Wires | Male-to-Male & Male-to-Female | $2-5 |
| 1 | Breadboard (optional) | For prototyping | $2-5 |
| | **Total Estimated Cost** | | **$41-82** |

---

## 📚 Required Libraries

```cpp
#include <DabbleESP32.h>  // External library for Dabble app integration
```

**Installation:** 
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for **"DabbleESP32"**
4. Install the library by **STEMpedia** (or **Dabble**)
5. Alternatively, download from GitHub: https://github.com/STEMpedia/DabbleESP32

**Note:** The DabbleESP32 library handles all Bluetooth communication with the Dabble mobile app, including gamepad input processing.

### **Arduino IDE Setup**
1. Install ESP32 board support:
   - File → Preferences
   - Additional Board Manager URLs: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. Install DabbleESP32 library:
   - Sketch → Include Library → Manage Libraries
   - Search "DabbleESP32" → Install

3. Select board:
   - Tools → Board → ESP32 Arduino → ESP32 Dev Module

4. Configure settings:
   - Upload Speed: 115200
   - Flash Frequency: 80MHz
   - Partition Scheme: Default

### **Dabble Mobile App Setup**
1. **Download Dabble App:**
   - Android: Google Play Store
   - iOS: App Store (if available)
   - Search for "Dabble" by STEMpedia

2. **Connect to ESP32:**
   - Power on ESP32
   - Open Dabble app on mobile device
   - Enable Bluetooth on mobile device
   - Tap "Connect" in Dabble app
   - Select "IoT_RaceCar" from device list
   - Wait for connection confirmation

3. **Using Gamepad Module:**
   - In Dabble app, select "Gamepad" module
   - Use directional arrows to control the car
   - Up = Forward, Down = Backward, Left = Turn Left, Right = Turn Right

---

## 🎓 Key Concepts

### **1. H-Bridge Motor Control**
An H-Bridge allows bidirectional motor control using 4 switches (transistors):

```
Forward:  IN1=HIGH, IN2=LOW  → Current flows Motor+ to Motor-
Backward: IN1=LOW,  IN2=HIGH → Current flows Motor- to Motor+
Stop:     IN1=LOW,  IN2=LOW  → No current flow
Brake:    IN1=HIGH, IN2=HIGH → Short circuit (NOT USED)
```

### **2. PWM (Pulse Width Modulation)**
Controls motor speed by rapidly switching power on/off:
- **Duty Cycle:** Percentage of time signal is HIGH
- **0 (0%):** Motor off
- **128 (50%):** Half speed
- **255 (100%):** Full speed
- **Frequency:** ESP32 default ~5kHz

### **3. Ultrasonic Distance Measurement**
Uses sound wave travel time:
1. Send 10µs trigger pulse
2. Sensor emits 8x 40kHz ultrasonic pulses
3. Measure echo pulse width
4. Formula: Distance = (Echo Time × Speed of Sound) / 2
5. Simplified: Distance (cm) = Echo Time (µs) / 58

### **4. Non-blocking Programming**
Ensures continuous sensor monitoring:
- ❌ **Bad:** `delay(1000)` - blocks all execution
- ✅ **Good:** Check sensors every loop cycle
- Result: Instant obstacle response even during movement

---

## 📝 Next Steps - Implementation Checklist

- [x] Requirements analysis complete
- [x] Hardware component selection
- [x] Pin configuration designed
- [x] Software architecture planned
- [x] Flowcharts and diagrams created
- [ ] **Create Arduino .ino file with DabbleESP32 library**
- [ ] **Write motor control functions**
- [ ] **Implement sensor reading**
- [ ] **Implement main loop logic**
- [ ] **Upload and test on hardware**
- [ ] **Calibrate sensors and motors**
- [ ] **Race track testing**

---

## 🏁 Ready for Implementation

This documentation provides complete specifications for building the IoT Race Car. All components, connections, logic flow, and testing procedures are defined. 

**The next step is to create the actual Arduino code file implementing this design.**

---

**Document Version:** 1.0  
**Last Updated:** December 28, 2025  
**Status:** ✅ Planning Complete - Ready for Code Implementation
