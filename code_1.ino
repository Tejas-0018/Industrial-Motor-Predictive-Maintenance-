/* * FINAL CODE: Industrial Guard (Project: pro_2)
 * -------------------------------------------------------------------
 * LABELS: "off", "normal", "shaft", "mount"
 * LOGIC: Uses SPIKE_LIMIT = 0.20 to match filtered training data.
 * FAULT LOGIC:
 * - IF (Shaft > 0) OR (0 < Mount < 0.90) -> SHAFT FAULT
 * - IF (Mount >= 0.90) -> MOUNT FAULT
 */

// !!! CHECK THIS LINE: Use the exact name of your new library header !!!
#include <Pro_2_inferencing.h> 
#include <Wire.h>
#include <MPU6050_light.h>
#include <WiFi.h>
#include <ThingSpeak.h>

// 1. USER SETTINGS
const char* ssid = "Tejas";          
const char* password = "123456789"; 

unsigned long myChannelNumber = 3239801;     
const char * myWriteAPIKey = "GEWA727F6FESXTCR"; 

// 2. SYSTEM GLOBALS
WiFiClient client;
MPU6050 mpu(Wire);
const int relay = 12;

// --- CRITICAL SETTING: SYMMETRY ---
float SPIKE_LIMIT = 0.20; 
// ----------------------------------

// Physics Gate: Below 0.05G, the machine is definitely OFF.
float NOISE_THRESHOLD = 0.05;   

// AI Variables
const unsigned long INTERVAL_US = 10000; // 100Hz
unsigned long lastSampleTime = 0;
#define EI_WINDOW_SIZE (EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE) 
float features[EI_WINDOW_SIZE];
int featureIndex = 0;
int faultCounter = 0; 

// Memory for Filter
float lastX = 0, lastY = 0, lastZ = 1.0;
float minX = 100, maxX = -100;

// Dual Core Communication
struct CloudPacket {
  float vibration;      
  int faultCount;       
  int statusCode;       
};
QueueHandle_t cloudQueue;

// 3. CORE 0 TASK: WI-FI & CLOUD
void TaskCloud(void *pvParameters) {
  Serial.print("Cloud Task running on Core: ");
  Serial.println(xPortGetCoreID());

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\nWiFi Connected!");
  Serial.println(WiFi.localIP());
  
  ThingSpeak.begin(client);
  CloudPacket packet;
  unsigned long lastUploadTime = 0;

  for (;;) { 
    if (xQueueReceive(cloudQueue, &packet, portMAX_DELAY) == pdPASS) {
      if (millis() - lastUploadTime > 15000) {
        Serial.println("Uploading to ThingSpeak...");
        ThingSpeak.setField(1, packet.vibration);
        ThingSpeak.setField(2, packet.faultCount);
        ThingSpeak.setField(3, packet.statusCode); 
        int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        if(x == 200) Serial.println("Upload Success");
        lastUploadTime = millis();
      } 
    }
  }
}

// 4. SETUP
void setup() {
  pinMode(relay, OUTPUT);
  Serial.begin(115200); 
  
  cloudQueue = xQueueCreate(5, sizeof(CloudPacket));
  xTaskCreatePinnedToCore(TaskCloud, "CloudTask", 10000, NULL, 1, NULL, 0);

  Wire.begin(21, 22);
  Wire.setClock(100000); 
  byte status = mpu.begin();
  if(status != 0){
    Serial.println("Sensor Missing!");
    while(1){ delay(10); } 
  }
  
  Serial.println("Calculating offsets...");
  delay(1000);
  mpu.calcOffsets(); 
  
  Serial.println("STARTING MOTOR...");
  digitalWrite(relay, HIGH); 
  
  Serial.println("Ignoring startup jerk (3.0s)...");
  delay(3000); 
  
  Serial.println("SYSTEM ACTIVE (Pro_2 Filtered Mode)");
  lastSampleTime = micros();
}

// 5. MAIN LOOP
void loop() {
  if (micros() - lastSampleTime >= INTERVAL_US) {
    lastSampleTime += INTERVAL_US; 
    mpu.update();
    
    // --- STEP 1: APPLY THE FILTER ---
    float rawX = mpu.getAccX(); 
    float rawY = mpu.getAccY(); 
    float rawZ = mpu.getAccZ();

    if (abs(rawX - lastX) < SPIKE_LIMIT) lastX = rawX;
    if (abs(rawY - lastY) < SPIKE_LIMIT) lastY = rawY;
    if (abs(rawZ - lastZ) < SPIKE_LIMIT) lastZ = rawZ;

    // --- STEP 2: FILL BUFFER ---
    if (featureIndex < EI_WINDOW_SIZE) {
        features[featureIndex++] = lastX; 
        features[featureIndex++] = lastY; 
        features[featureIndex++] = lastZ; 
        
        if (lastX < minX) minX = lastX;
        if (lastX > maxX) maxX = lastX;
    }
    
    // --- STEP 3: PROCESS BATCH ---
    else {
        float vibrationStrength = maxX - minX;
        int currentStatus = 0; 
        
        // NOISE GATE
        if (vibrationStrength < NOISE_THRESHOLD) {
             if (faultCounter > 0) Serial.println("Counter Reset.");
             faultCounter = 0; 
             currentStatus = 0; 
             Serial.println("Idle (Low Vibration)");
        }
        // AI CLASSIFICATION
        else {
            signal_t signal;
            numpy::signal_from_buffer(features, EI_WINDOW_SIZE, &signal);
            ei_impulse_result_t result;
            run_classifier(&signal, &result, false);
    
            String mainStatus = "Unknown";
            float maxConfidence = 0.0;
            
            float shaftScore = 0.0;
            float mountScore = 0.0;
            float normalScore = 0.0;
            float offScore = 0.0;
            
            // Extract Scores
            for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
                String label = result.classification[i].label;
                float value = result.classification[i].value;
                
                if (label == "shaft") shaftScore = value;
                if (label == "mount") mountScore = value;
                if (label == "normal") normalScore = value;
                if (label == "off") offScore = value;
            }

            // ===============================================
            //  USER REQUESTED LOGIC START
            // ===============================================

            // Safety: If Normal or Off is dominant (> 0.50), trust them first.
            if (normalScore > 0.50) {
                mainStatus = "normal";
            }
            else if (offScore > 0.50) {
                mainStatus = "off";
            }
            else {
                // APPLY AGGRESSIVE FAULT LOGIC
                // Logic: (Shaft > 0) OR (0 < Mount < 0.90) -> SHAFT
                
                // Note: We use > 0.01 instead of > 0 to avoid floating point noise
                bool conditionShaft = (shaftScore > 0.6) || (mountScore > 0.01 && mountScore < 0.90);
                
                if (conditionShaft) {
                    mainStatus = "shaft";
                }
                else if (mountScore >= 0.90) {
                    mainStatus = "mount";
                }
            }
            // ===============================================
            //  USER REQUESTED LOGIC END
            // ===============================================
            
            
            // --- ACT ON RESULT ---
            if (mainStatus == "shaft" || mainStatus == "mount") {
                faultCounter++; 
                currentStatus = 2; // Fault Code
                
                Serial.print("DETECTED: "); Serial.print(mainStatus);
                Serial.printf(" [Shaft:%.2f vs Mount:%.2f] | Strike %d/3\n", shaftScore, mountScore, faultCounter);
                
                if (faultCounter >= 3) {
                    Serial.println("CRITICAL FAILURE CONFIRMED!");
                    Serial.print("STOPPING MOTOR: "); Serial.println(mainStatus);
                    
                    digitalWrite(relay, LOW); 
                    
                    CloudPacket deathMsg; 
                    deathMsg.vibration=vibrationStrength; 
                    deathMsg.faultCount=3; 
                    deathMsg.statusCode=2;
                    xQueueSend(cloudQueue, &deathMsg, 0);
                    
                    while(1) { delay(100); } // Lock system
                }
            }
            else if (mainStatus == "normal") {
                 faultCounter = 0;
                 currentStatus = 1; 
                 Serial.println("Normal Operation");
            }
            else if (mainStatus == "off") {
                 faultCounter = 0;
                 currentStatus = 0; 
                 Serial.println("Idle (AI Detected)");
            }
        }
        
        // Send to Cloud Task
        CloudPacket msg;
        msg.vibration = vibrationStrength;
        msg.faultCount = faultCounter;
        msg.statusCode = currentStatus;
        xQueueSend(cloudQueue, &msg, 0); 

        // Reset
        featureIndex = 0;
        minX = 100; maxX = -100;
    }
  }
}