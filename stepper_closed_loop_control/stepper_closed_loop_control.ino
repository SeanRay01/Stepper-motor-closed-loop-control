// ESP32 DRV8825引腳定義
#define STEP_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 4 // 改用GPIO4，避免與ADC衝突
#define MS1_PIN 16
#define MS2_PIN 17
#define MS3_PIN 5

// AS5600類比輸出引腳定義
#define AS5600_OUT_PIN 21 // AS5600的OUT引腳連接到ESP32 GPIO21

// 步進馬達參數
#define STEPS_PER_REV 200 // 基本步數
#define MICROSTEPS 16     // 微步設定 (1/16微步)

// AS5600類比讀取參數
#define ADC_RESOLUTION 4095    // ESP32 12位元ADC解析度
#define AS5600_MAX_VOLTAGE 3.3 // AS5600輸出最大電壓

// 控制變數
float targetAngle = 0;  // 目標角度 (度)
float currentAngle = 0; // 目前角度 (度)
float lastAngle = 0;    // 上次角度
float error = 0;        // 誤差
float integral = 0;     // 積分項
float derivative = 0;   // 微分項
float lastError = 0;    // 上次誤差

// PID參數
float kp = 2.0;  // 比例增益
float ki = 0.1;  // 積分增益
float kd = 0.05; // 微分增益

// 控制參數
float output = 0;
int stepDelay = 1000; // 步進延遲 (微秒)
unsigned long lastTime = 0;
float dt = 0;

// 狀態變數
bool motorEnabled = true;
int currentPosition = 0; // 目前步數位置

// 角度濾波參數
float filteredAngle = 0;
const float filterAlpha = 0.1; // 低通濾波器係數

void setup()
{
    Serial.begin(115200);

    // 設定AS5600類比輸出引腳為輸入
    pinMode(AS5600_OUT_PIN, INPUT);

    // 初始化ADC
    analogReadResolution(12); // 設定ADC為12位元解析度

    // 設定DRV8825引腳
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    // 設定微步模式 (1/16微步)
    setMicrostepMode(16);

    // 啟用馬達驅動器
    digitalWrite(ENABLE_PIN, LOW);

    // 讀取初始角度
    currentAngle = readAS5600Angle();
    lastAngle = currentAngle;
    filteredAngle = currentAngle;
    lastTime = micros();

    Serial.println("ESP32 步進馬達閉迴路控制系統啟動 (AS5600 OUT引腳模式)");
    Serial.println("輸入指令:");
    Serial.println("s<角度> - 設定目標角度 (例如: s90)");
    Serial.println("p<值> - 設定KP值 (例如: p2.5)");
    Serial.println("i<值> - 設定KI值 (例如: i0.1)");
    Serial.println("d<值> - 設定KD值 (例如: d0.05)");
    Serial.println("e - 啟用/停用馬達");
    Serial.println("r - 重設位置");
    Serial.println("c - 校準零點");
}

void loop()
{
    // 處理串列埠指令
    handleSerialCommands();

    // 執行閉迴路控制
    if (motorEnabled)
    {
        closedLoopControl();
    }

    // 顯示狀態資訊
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 100)
    { // 每100ms顯示一次
        printStatus();
        lastPrint = millis();
    }

    delay(1);
}

float readAS5600Angle()
{
    // 讀取ADC值
    int adcValue = analogRead(AS5600_OUT_PIN);

    // 轉換為電壓
    float voltage = (float)adcValue / ADC_RESOLUTION * AS5600_MAX_VOLTAGE;

    // 轉換為角度 (0-360度)
    float angle = (voltage / AS5600_MAX_VOLTAGE) * 360.0;

    // 應用低通濾波器減少雜訊
    filteredAngle = filterAlpha * angle + (1.0 - filterAlpha) * filteredAngle;

    return filteredAngle;
}

void closedLoopControl()
{
    // 讀取目前角度
    currentAngle = readAS5600Angle();

    // 處理角度跨越360度的情況
    float angleDiff = currentAngle - lastAngle;
    if (angleDiff > 180)
    {
        currentAngle -= 360;
    }
    else if (angleDiff < -180)
    {
        currentAngle += 360;
    }
    lastAngle = currentAngle;

    // 計算時間差
    unsigned long currentTime = micros();
    dt = (currentTime - lastTime) / 1000000.0; // 轉換為秒
    lastTime = currentTime;

    // 避免除以零或過小的時間差
    if (dt < 0.001)
        dt = 0.001;

    // 計算誤差
    error = targetAngle - currentAngle;

    // PID控制計算
    integral += error * dt;
    derivative = (error - lastError) / dt;

    // 防止積分飽和
    if (integral > 100)
        integral = 100;
    if (integral < -100)
        integral = -100;

    // PID輸出
    output = kp * error + ki * integral + kd * derivative;

    // 限制輸出範圍
    if (output > 100)
        output = 100;
    if (output < -100)
        output = -100;

    // 根據輸出控制步進馬達
    if (abs(error) > 0.5)
    { // 死區控制，避免震盪
        controlStepper(output);
    }

    lastError = error;
}

void controlStepper(float output)
{
    // 設定方向
    if (output > 0)
    {
        digitalWrite(DIR_PIN, HIGH);
    }
    else
    {
        digitalWrite(DIR_PIN, LOW);
        output = -output; // 取絕對值
    }

    // 計算步進速度
    int delayTime = map(constrain(output, 0, 100), 0, 100, 2000, 200);

    // 執行步進
    if (output > 5)
    { // 最小輸出閾值
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(delayTime);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(delayTime);

        // 更新位置計數
        if (digitalRead(DIR_PIN))
        {
            currentPosition++;
        }
        else
        {
            currentPosition--;
        }
    }
}

void setMicrostepMode(int microsteps)
{
    // 設定DRV8825微步模式
    switch (microsteps)
    {
    case 1:
        digitalWrite(MS1_PIN, LOW);
        digitalWrite(MS2_PIN, LOW);
        digitalWrite(MS3_PIN, LOW);
        break;
    case 2:
        digitalWrite(MS1_PIN, HIGH);
        digitalWrite(MS2_PIN, LOW);
        digitalWrite(MS3_PIN, LOW);
        break;
    case 4:
        digitalWrite(MS1_PIN, LOW);
        digitalWrite(MS2_PIN, HIGH);
        digitalWrite(MS3_PIN, LOW);
        break;
    case 8:
        digitalWrite(MS1_PIN, HIGH);
        digitalWrite(MS2_PIN, HIGH);
        digitalWrite(MS3_PIN, LOW);
        break;
    case 16:
        digitalWrite(MS1_PIN, HIGH);
        digitalWrite(MS2_PIN, HIGH);
        digitalWrite(MS3_PIN, HIGH);
        break;
    default:
        // 預設1/16微步
        digitalWrite(MS1_PIN, HIGH);
        digitalWrite(MS2_PIN, HIGH);
        digitalWrite(MS3_PIN, HIGH);
    }
}

void handleSerialCommands()
{
    if (Serial.available())
    {
        String command = Serial.readString();
        command.trim();

        if (command.startsWith("s"))
        {
            // 設定目標角度
            float angle = command.substring(1).toFloat();
            setTargetAngle(angle);
            Serial.print("目標角度設定為: ");
            Serial.println(angle);
        }
        else if (command.startsWith("p"))
        {
            // 設定KP值
            kp = command.substring(1).toFloat();
            Serial.print("KP設定為: ");
            Serial.println(kp);
        }
        else if (command.startsWith("i"))
        {
            // 設定KI值
            ki = command.substring(1).toFloat();
            Serial.print("KI設定為: ");
            Serial.println(ki);
        }
        else if (command.startsWith("d"))
        {
            // 設定KD值
            kd = command.substring(1).toFloat();
            Serial.print("KD設定為: ");
            Serial.println(kd);
        }
        else if (command == "e")
        {
            // 啟用/停用馬達
            motorEnabled = !motorEnabled;
            digitalWrite(ENABLE_PIN, motorEnabled ? LOW : HIGH);
            Serial.print("馬達: ");
            Serial.println(motorEnabled ? "啟用" : "停用");
        }
        else if (command == "r")
        {
            // 重設位置
            resetPosition();
            Serial.println("位置已重設");
        }
        else if (command == "c")
        {
            // 校準零點
            calibrateZero();
            Serial.println("零點校準完成");
        }
    }
}

void setTargetAngle(float angle)
{
    targetAngle = angle;
    // 重設積分項以避免突跳
    integral = 0;
}

void resetPosition()
{
    currentPosition = 0;
    currentAngle = readAS5600Angle();
    targetAngle = currentAngle;
    integral = 0;
    lastError = 0;
}

void calibrateZero()
{
    // 讀取多次取平均值來校準零點
    float sum = 0;
    for (int i = 0; i < 50; i++)
    {
        sum += readAS5600Angle();
        delay(10);
    }
    float zeroOffset = sum / 50.0;

    // 將目前位置設為零點
    targetAngle = 0;
    currentAngle = 0;
    filteredAngle = 0;
    integral = 0;
    lastError = 0;

    Serial.print("零點校準值: ");
    Serial.println(zeroOffset);
}

void printStatus()
{
    // 讀取原始ADC值用於診斷
    int rawADC = analogRead(AS5600_OUT_PIN);
    float voltage = (float)rawADC / ADC_RESOLUTION * AS5600_MAX_VOLTAGE;

    Serial.print("target: ");
    Serial.print(targetAngle, 1);
    Serial.print("° | Current: ");
    Serial.print(currentAngle, 1);
    Serial.print("° | Error: ");
    Serial.print(error, 2);
    Serial.print("° | Output: ");
    Serial.print(output, 1);
    Serial.print(" | Steps: ");
    Serial.print(currentPosition);
    Serial.print(" | ADC: ");
    Serial.print(rawADC);
    Serial.print(" | Voltage: ");
    Serial.println(voltage, 3);
}