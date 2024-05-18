#include <Arduino.h>
#include "esp32-a2dp/BluetoothA2DPSink.h"
#include "RotaryEncoder.h"
#include <WiFi.h>

#define VOLUME_MAX      128
#define VOLUME_STEP     4

#define DEBOUNCE_TIME   20

// pins
#define PIN_PREVIOUS_TRACK        GPIO_NUM_16
#define PIN_NEXT_TRACK            GPIO_NUM_17
#define PIN_PLAYPAUSE             GPIO_NUM_21
#define PIN_ROTARY_ENCODER_1      GPIO_NUM_5
#define PIN_ROTARY_ENCODER_2      GPIO_NUM_18

// bitwise
// PLAYPAUSE | PREVIOUS | NEXT | VOLUME_DOWN | VOLUME_UP
#define PLAYPAUSE       0b10000
#define PREVIOUS_TRACK  0b01000
#define NEXT_TRACK      0b00100
#define VOLUME_DOWN     0b00010
#define VOLUME_UP       0b00001
#define CONFIRM_PIN     PREVIOUS_TRACK | NEXT_TRACK

void volume_up();
void volume_down();
uint8_t volumeControl();
bool playing();
void playpause();
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr);

// https://github.com/pschatzmann/ESP32-A2DP

BluetoothA2DPSink a2dp_sink;
RotaryEncoder rotary_encoder(PIN_ROTARY_ENCODER_1, PIN_ROTARY_ENCODER_2, RotaryEncoder::LatchMode::TWO03);
uint8_t prev_cmd = 0b00000;

void setup() {
  // turn WIFI off to save power
  WiFi.setSleep(WIFI_PS_MAX_MODEM);
  WiFi.mode(WIFI_OFF);

  pinMode(PIN_PREVIOUS_TRACK, INPUT_PULLUP);
  pinMode(PIN_NEXT_TRACK, INPUT_PULLUP);
  pinMode(PIN_PLAYPAUSE, INPUT_PULLUP);
  pinMode(PIN_ROTARY_ENCODER_1, INPUT_PULLUP);
  pinMode(PIN_ROTARY_ENCODER_2, INPUT_PULLUP);

  // --------------
  // ---- I2S -----
  // --------------
  // BCK -> GPIO 26
  // DIN -> GPIO 22
  // SCK -> GND
  // LCK -> GPIO 25
  // --------------

  a2dp_sink.set_auto_reconnect(true, 10);
  a2dp_sink.set_reconnect_delay(100);
  a2dp_sink.set_bits_per_sample(16);
  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.activate_pin_code(true);
  a2dp_sink.start("Twizy");
}

void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  if(state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
    a2dp_sink.set_auto_reconnect(false);
  } else {
    a2dp_sink.set_auto_reconnect(true, 10);
  }
}

uint8_t volumeControl() {
  static int pos = 0;
  rotary_encoder.tick();

  int newPos = rotary_encoder.getPosition();
  if (pos != newPos) {
    pos = newPos;
    return (((int)rotary_encoder.getDirection()) > 0) ? VOLUME_UP : VOLUME_DOWN;
  }
  return 0b00000;
}

void loop() {
  uint8_t cmd = volumeControl();
  
  uint8_t button_mask = (!digitalRead(PIN_PLAYPAUSE) << 4 | !digitalRead(PIN_PREVIOUS_TRACK) << 3 | !digitalRead(PIN_NEXT_TRACK) << 2) & 0b11100;
  if(button_mask != prev_cmd) { // edge detection: run command only on button press, not while holding it!
    cmd |= button_mask;
    delay(50);
  }
  prev_cmd = button_mask;

  switch(cmd) {
    case NEXT_TRACK:
      a2dp_sink.next();
      break;

    case PREVIOUS_TRACK:
      a2dp_sink.previous();
      break;

    case PLAYPAUSE:
      playpause();
      break;

    case VOLUME_UP:
      volume_up();
      break;
    
    case VOLUME_DOWN:
      volume_down();
      break;

    case CONFIRM_PIN:
      if(a2dp_sink.pin_code() != 0) {
        a2dp_sink.confirm_pin_code();
      }
      break;
    
    default:
      break;
  }
  delay(5);
}

bool playing() {
  return a2dp_sink.get_audio_state() == ESP_A2D_AUDIO_STATE_STARTED;
}

void playpause() {
  if(playing()) {
    a2dp_sink.pause();
  } else {
    a2dp_sink.play();
  }
}

void volume_up() {
  uint8_t volume = a2dp_sink.get_volume();

  if(volume < VOLUME_MAX - VOLUME_STEP) {
    volume += VOLUME_STEP;
  } else {
    volume = VOLUME_MAX;
  }

  a2dp_sink.set_volume(volume);
}

void volume_down() {
  uint8_t volume = a2dp_sink.get_volume();

  if(volume > VOLUME_STEP) {
    volume -= VOLUME_STEP;
  } else {
    volume = 0;
  }

  a2dp_sink.set_volume(volume);
}