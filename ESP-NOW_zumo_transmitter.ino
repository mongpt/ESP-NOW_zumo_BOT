#include <esp_now.h>
#include <WiFi.h>
#include <Ticker.h>

// Button class definition
class Button {
public:
    Button(uint8_t btn, bool isInput, bool isPullup = true) : gpio(btn), mode(isInput), pullup(isPullup) {
        if (isInput) {
            pinMode(gpio, isPullup ? INPUT_PULLUP : INPUT);
        } else {
            pinMode(gpio, OUTPUT);
        }
    }
    bool held = false;

    bool state() const {
        return digitalRead(gpio) == HIGH;
    }

    bool isPressed() {
        const uint DEBOUNCE_THRESHOLD = 3;
        uint pressed = 0;
        uint released = 0;
        while (pressed < DEBOUNCE_THRESHOLD && released < DEBOUNCE_THRESHOLD) {
            if (!state()) {
                pressed++;
                released = 0;
            } else {
                released++;
                pressed = 0;
            }
            delay(10);
        }
        return pressed >= DEBOUNCE_THRESHOLD;
    }

private:
    int gpio;
    bool mode;
    bool pullup;
};

const uint8_t forward_btn = 5;
const uint8_t backward_btn = 6;
const uint8_t left_btn = 1;
const uint8_t right_btn = 0;
// Global button definitions
Button FORWARD(forward_btn, true);
Button BACKWARD(backward_btn, true);
Button LEFT(left_btn, true);
Button RIGHT(right_btn, true);

// ESP-NOW variables
uint8_t broadcastAddress[] = {0x98, 0x3D, 0xAE, 0x50, 0xF9, 0x64};
esp_now_peer_info_t peerInfo;

int forward_val = 0, backward_val = 0, left_val = 0, right_val = 0;
int last_data = 0;
volatile bool F_pressed = false;
volatile bool B_pressed = false;
volatile bool L_pressed = false;
volatile bool R_pressed = false;

// ISR
void IRAM_ATTR F_ISR() {
    F_pressed = true;  // Set flag
}
void IRAM_ATTR B_ISR() {
    B_pressed = true;  // Set flag
}
void IRAM_ATTR L_ISR() {
    L_pressed = true;  // Set flag
}
void IRAM_ATTR R_ISR() {
    R_pressed = true;  // Set flag
}

Ticker timer1;

void send_heartBeat(){
    uint8_t hb = 100;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&hb, 1);
    while (result != ESP_OK) {
        result = esp_now_send(broadcastAddress, (uint8_t*)&hb, 1);
    }
    Serial.println("Heartbeat sent");
}

void setup() {
    // Initialize serial monitor
    Serial.begin(115200);

    // Set device as a Wi-Fi station
    WiFi.mode(WIFI_STA);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    // Register peer
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(forward_btn), F_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(backward_btn), B_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(left_btn), L_ISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(right_btn), R_ISR, FALLING);

    //send data to reset receiver
    uint8_t reset = 101;
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&reset, 1);
    while (result != ESP_OK) {
        result = esp_now_send(broadcastAddress, (uint8_t*)&reset, 1);
    }
    Serial.println("System reset");

    //set timer to send hearbeat
    timer1.attach(0.48, send_heartBeat);    //480ms
}

void loop() {
    if (F_pressed) {
        if (FORWARD.isPressed()) {
            forward_val = 1;
        }
        F_pressed = false;
    }
    if (FORWARD.state()) {
        forward_val = 0;
    }

    if (B_pressed) {
        if (BACKWARD.isPressed()) {
            backward_val = 2;
        }
        B_pressed = false;
    }
    if (BACKWARD.state()) {
        backward_val = 0;
    }

    if (L_pressed) {
        if (LEFT.isPressed()) {
            left_val = 10;
        }
        L_pressed = false;
    }
    if (LEFT.state()) {
        left_val = 0;
    }

    if (R_pressed) {
        if (RIGHT.isPressed()) {
            right_val = 20;
        }
        R_pressed = false;
    }
    if (RIGHT.state()) {
        right_val = 0;
    }

    int data = forward_val + backward_val + left_val + right_val;
    if (data != last_data) {
        last_data = data;
        esp_err_t result = esp_now_send(broadcastAddress, (uint8_t*)&data, 1);
        while (result != ESP_OK) {
            result = esp_now_send(broadcastAddress, (uint8_t*)&data, 1);
        }
        Serial.print("Data sent: ");
        Serial.println(data);
    }

    delay(50); // Non-blocking delay
}

