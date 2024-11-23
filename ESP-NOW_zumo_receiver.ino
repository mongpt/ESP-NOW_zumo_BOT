#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH THE MAC Address of your receiver
uint8_t broadcastAddress[] = {0x98, 0x3D, 0xAE, 0x53, 0x2A, 0x30};

esp_now_peer_info_t peerInfo;
volatile int dir = 0, L_speed = 0, R_speed = 0;
volatile bool weapon = false;
volatile u_long heartBeat = 0;
bool isFirstHb = false;

const uint8_t RDIR = 10;
const uint8_t LDIR = 9;
const uint8_t RPWM = 6;
const uint8_t LPWM = 7;
const uint8_t WEAPON = 4;
const int pwmFrequency = 1000;  // PWM frequency in Hz (1 kHz)
const int pwmResolution = 8;    // PWM resolution (8-bit, 0-255)


// Callback when data is received
void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* incomingData, int len) {
    Serial.print("Data received: ");
    Serial.println(incomingData[0]);
    uint8_t received_data = incomingData[0];
    switch (received_data) {
        case 0:
            L_speed = 0;
            R_speed = 0;
            break;
        case 1: //forward
            dir = 0;
            L_speed = 90;
            R_speed = 100;
            break;
        case 2: //backward
            dir = 1;
            L_speed = 90;
            R_speed = 100;
            break;
        case 11:    //forward left
            dir = 0;
            L_speed = 90;
            R_speed = 30;
            break;
        case 21:    //forward right
            dir = 0;
            L_speed = 30;
            R_speed = 100;
            break;
        case 12:    //backward left
            dir = 1;
            L_speed = 90;
            R_speed = 30;
            break;
        case 22:    //backward right
            dir = 1;
            L_speed = 30;
            R_speed = 100;
            break;
        case 30:    //weapon
            weapon = !weapon;
            break;
        case 100:   //heart beat signal
            heartBeat = millis();
            break;
        case 101:    //reset system when established connection with transmitter
            dir = 0;
            L_speed = 0;
            R_speed = 0;
            weapon = false;
            break;
        default:
            break;
    }
}

void setup() {
    pinMode(RDIR, OUTPUT);
    pinMode(LDIR, OUTPUT);
    pinMode(RPWM, OUTPUT);
    pinMode(LPWM, OUTPUT);
    pinMode(WEAPON, OUTPUT);
    digitalWrite(WEAPON, false);

    // Initialize the PWM pins with the new LEDC API
    ledcAttach(LPWM, pwmFrequency, pwmResolution);  // Assign GPIO6 to PWM
    ledcAttach(RPWM, pwmFrequency, pwmResolution);  // Assign GPIO7 to PWM

    // Init Serial Monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Register for a callback function that will be called when data is received
    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    int L_duty = 0;
    int R_duty = 0;
    bool weapon_state = false;

    if (isFirstHb){
        if (millis() - heartBeat <= 500) {
            L_duty = map(L_speed, 0, 100, 0, 255);
            R_duty = map(R_speed, 0, 100, 0, 255);
            weapon_state = weapon;
        } else {
            isFirstHb = false;
            heartBeat = 0;
        }
    } else {
        if (heartBeat > 0) {
            isFirstHb = true;
        }
    }
    digitalWrite(LDIR, dir);
    digitalWrite(RDIR, dir);
    ledcWrite(LPWM, L_duty);  // Write duty cycle to GPIO6
    ledcWrite(RPWM, R_duty);  // Write duty cycle to GPIO7
    digitalWrite(WEAPON, weapon_state);
}
