/*
 * 硬體測試程式 - AS5600霍爾感測器 + DRV8825步進馬達驅動器
 * 用途：驗證硬體連接和基本功能
 */

// ESP32 DRV8825引腳定義
#define STEP_PIN 18
#define DIR_PIN 19
#define ENABLE_PIN 4
#define MS1_PIN 16
#define MS2_PIN 17
#define MS3_PIN 5

// AS5600類比輸出引腳定義
#define AS5600_OUT_PIN 36

// AS5600類比讀取參數
#define ADC_RESOLUTION 4095
#define AS5600_MAX_VOLTAGE 3.3

// 測試參數
int testMode = 0;
bool motorEnabled = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("\n=== 硬體測試程式啟動 ===");
    Serial.println("ESP32 + AS5600 + DRV8825 測試");

    // 設定AS5600類比輸出引腳
    pinMode(AS5600_OUT_PIN, INPUT);
    analogReadResolution(12);

    // 設定DRV8825引腳
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(ENABLE_PIN, OUTPUT);
    pinMode(MS1_PIN, OUTPUT);
    pinMode(MS2_PIN, OUTPUT);
    pinMode(MS3_PIN, OUTPUT);

    // 初始化DRV8825
    digitalWrite(ENABLE_PIN, HIGH); // 先停用馬達
    digitalWrite(DIR_PIN, LOW);
    digitalWrite(STEP_PIN, LOW);

    // 設定全步模式 (簡單測試)
    digitalWrite(MS1_PIN, LOW);
    digitalWrite(MS2_PIN, LOW);
    digitalWrite(MS3_PIN, LOW);

    Serial.println("\n指令說明:");
    Serial.println("1 - AS5600感測器測試");
    Serial.println("2 - 步進馬達基本測試 (正轉)");
    Serial.println("3 - 步進馬達基本測試 (反轉)");
    Serial.println("4 - 連續監控AS5600數值");
    Serial.println("5 - 馬達啟用/停用切換");
    Serial.println("0 - 停止所有測試");
    Serial.println();
}

void loop()
{
    // 處理串列埠指令
    if (Serial.available())
    {
        char cmd = Serial.read();

        switch (cmd)
        {
        case '1':
            testMode = 1;
            Serial.println(">>> AS5600感測器測試模式");
            testAS5600();
            break;

        case '2':
            testMode = 2;
            Serial.println(">>> 步進馬達正轉測試");
            testStepperForward();
            break;

        case '3':
            testMode = 3;
            Serial.println(">>> 步進馬達反轉測試");
            testStepperReverse();
            break;

        case '4':
            testMode = 4;
            Serial.println(">>> 連續監控AS5600 (按0停止)");
            break;

        case '5':
            toggleMotor();
            break;

        case '0':
            testMode = 0;
            digitalWrite(ENABLE_PIN, HIGH); // 停用馬達
            Serial.println(">>> 停止所有測試");
            break;
        }
    }

    // 執行對應的測試模式
    switch (testMode)
    {
    case 4:
        monitorAS5600();
        delay(100);
        break;
    }

    delay(10);
}

void testAS5600()
{
    Serial.println("正在測試AS5600感測器...");

    for (int i = 0; i < 10; i++)
    {
        int rawADC = analogRead(AS5600_OUT_PIN);
        float voltage = (float)rawADC / ADC_RESOLUTION * AS5600_MAX_VOLTAGE;
        float angle = (voltage / AS5600_MAX_VOLTAGE) * 360.0;

        Serial.print("讀取 ");
        Serial.print(i + 1);
        Serial.print(": ADC=");
        Serial.print(rawADC);
        Serial.print(", 電壓=");
        Serial.print(voltage, 3);
        Serial.print("V, 角度=");
        Serial.print(angle, 1);
        Serial.println("°");

        delay(500);
    }

    Serial.println("AS5600測試完成");
    Serial.println("請手動轉動磁鐵，觀察數值變化");
    Serial.println();
    testMode = 0;
}

void testStepperForward()
{
    Serial.println("測試步進馬達正轉 (200步)...");

    digitalWrite(ENABLE_PIN, LOW); // 啟用馬達
    digitalWrite(DIR_PIN, HIGH);   // 設定正轉方向

    for (int i = 0; i < 200; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);

        if (i % 50 == 0)
        {
            Serial.print("步數: ");
            Serial.println(i);
        }
    }

    digitalWrite(ENABLE_PIN, HIGH); // 停用馬達
    Serial.println("正轉測試完成");
    Serial.println();
    testMode = 0;
}

void testStepperReverse()
{
    Serial.println("測試步進馬達反轉 (200步)...");

    digitalWrite(ENABLE_PIN, LOW); // 啟用馬達
    digitalWrite(DIR_PIN, LOW);    // 設定反轉方向

    for (int i = 0; i < 200; i++)
    {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(1000);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(1000);

        if (i % 50 == 0)
        {
            Serial.print("步數: ");
            Serial.println(i);
        }
    }

    digitalWrite(ENABLE_PIN, HIGH); // 停用馬達
    Serial.println("反轉測試完成");
    Serial.println();
    testMode = 0;
}

void monitorAS5600()
{
    int rawADC = analogRead(AS5600_OUT_PIN);
    float voltage = (float)rawADC / ADC_RESOLUTION * AS5600_MAX_VOLTAGE;
    float angle = (voltage / AS5600_MAX_VOLTAGE) * 360.0;

    Serial.print("即時監控 - ADC: ");
    Serial.print(rawADC);
    Serial.print(" | 電壓: ");
    Serial.print(voltage, 3);
    Serial.print("V | 角度: ");
    Serial.print(angle, 1);
    Serial.println("°");
}

void toggleMotor()
{
    motorEnabled = !motorEnabled;
    digitalWrite(ENABLE_PIN, motorEnabled ? LOW : HIGH);

    Serial.print("馬達: ");
    Serial.println(motorEnabled ? "啟用" : "停用");

    if (motorEnabled)
    {
        Serial.println("警告: 馬達已啟用，請小心操作！");
    }
}