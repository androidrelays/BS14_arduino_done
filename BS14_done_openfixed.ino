//lvgl 8.3.11
#include <ArduinoBLE.h>
#include <Arduino_H7_Video.h>
#include <lvgl.h>
#include <stdint.h>
#include <font/lv_font.h>
#include "Arduino_GigaDisplayTouch.h"
// Sense selection state - defaults to Sense A on startup
// Will be synced from the app when Bluetooth connects
bool settingsLoaded = false;

// Bluetooth - Use custom UUIDs matching Flutter app expectations
BLEService breakerService("12345678-1234-1234-1234-123456789abc");
BLECharacteristic commandChar("87654321-4321-4321-4321-cba987654321", BLEWrite, 20);
BLECharacteristic statusChar("11011111-2222-3333-4444-555555555555", BLERead | BLENotify, 20);
// Lock BLE characteristic (read/write, 1 byte: 0=unlocked, 1=locked)
BLECharacteristic lockChar("22222222-3333-4444-5555-666666666666",
                           BLERead | BLEWrite | BLENotify, 1);
// Sense selection BLE characteristic (read/write, 1 byte: 0=sense A, 1=sense B)
BLECharacteristic senseChar("33333333-4444-5555-6666-777777777777",
                            BLERead | BLEWrite | BLENotify, 1);

// Hardware interfaces
Arduino_H7_Video Display;
Arduino_GigaDisplayTouch TouchDetector;

// UI objects
lv_obj_t *switch_69;
lv_obj_t *ui_container;
lv_obj_t *tight_container;
lv_obj_t *switch_container;
lv_obj_t *btn_open;
// Define LV_SYMBOL_LOCK for overlay lock symbol
#ifndef LV_SYMBOL_LOCK
#define LV_SYMBOL_LOCK "\xef\x80\xa3"
#endif
lv_obj_t *btn_close;
lv_obj_t *close_btn_overlay; // Overlay for disabled state symbol
lv_obj_t *open_btn_overlay;  // Overlay for disabled state symbol on open button
lv_obj_t *lock_icon_btn = NULL;   // Lock icon button
lv_obj_t *lock_icon_label = NULL; // Lock icon label
lv_obj_t *lock_container = NULL;  // Container for lock button
lv_obj_t *btn_settings = NULL;    // Settings button
lv_obj_t *settings_modal = NULL;  // Settings modal dialog
lv_obj_t *btn_sense_a = NULL;     // Sense A button
lv_obj_t *btn_sense_b = NULL;     // Sense B button
bool senseA_selected = true;      // Default to sense A
lv_obj_t *top_bar = NULL;         // Top bar container
lv_obj_t *top_bar_bt_status = NULL; // Bluetooth status in top bar
lv_obj_t *btn_rotate = NULL;      // Rotate button (moved to top bar)

// LED pins (active-low RGB)
const int redled = 86;
const int greenled = 87;
const int blueled = 88;

// Breaker control pins
const int sense = 37;   // Sense pin A (default)
const int senseB = 47;  // Sense pin B (alternative)
const int pin39 = 39;
const int pin41 = 41;
const int openInput = 45;
const int closeInput = 43;

// State variables
bool breakerstate = true;
bool locked = false;
bool switchToggled = true;
unsigned long lock_press_start = 0;
unsigned long last_switch_toggle = 0; // Debouncing for switch
unsigned long lock_button_restore_time = 0; // For restoring lock button appearance

bool bluetoothConnected = false;
unsigned long lastStatusSent = 0;

// Rotation state
int currentRotation = 0;

// Function declarations
static void set_leds();
static void set_breaker_state(bool open);
static void open_btn_cb(lv_event_t *e);
static void close_btn_cb(lv_event_t *e);
static void switch_toggled_cb(lv_event_t *e);
static void update_button_styles();
static void rotate_screen_cb(lv_event_t *e);
static void send_status_to_flutter();
static void lock_icon_event_cb(lv_event_t *e);
static void update_lock_icon();
static void settings_btn_cb(lv_event_t *e);
static void sense_a_btn_cb(lv_event_t *e);
static void sense_b_btn_cb(lv_event_t *e);
static void close_settings_cb(lv_event_t *e);
static void disconnect_bluetooth_cb(lv_event_t *e);
static void create_ui();
static void create_top_bar();
static void update_top_bar_bt_status();

// LVGL touch driver
static lv_indev_t *indev_touch = NULL;
static void lvgl_touch_read_cb(lv_indev_drv_t *drv, lv_indev_data_t *data) {
  GDTpoint_t points[5];
  int n = TouchDetector.getTouchPoints(points);
  if (n > 0) {
    data->point.x = points[0].x;
    data->point.y = points[0].y;
    data->state = LV_INDEV_STATE_PR;
  } else {
    data->state = LV_INDEV_STATE_REL;
  }
}

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("=== ARDUINO GIGA STARTING ===");

  // Initialize sense selection (will be synced from app on connection)
  // Default to Sense A on startup
  senseA_selected = true;
  settingsLoaded = true;

  Display.begin();
  TouchDetector.begin();

  // Register LVGL touch input
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = lvgl_touch_read_cb;
  indev_touch = lv_indev_drv_register(&indev_drv);

  // BLE init
  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("BS14");
  BLE.setDeviceName("BS14");

  BLE.setAdvertisedService(breakerService);
  breakerService.addCharacteristic(commandChar);
  breakerService.addCharacteristic(statusChar);
  breakerService.addCharacteristic(lockChar);
  breakerService.addCharacteristic(senseChar);
  BLE.addService(breakerService);

  // Set up BLE event handlers for proper notification handling
  BLE.setEventHandler(BLEConnected, [](BLEDevice central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    bluetoothConnected = true;
    update_button_styles();
    update_lock_icon();
    set_leds();
    update_top_bar_bt_status();
    
    // Send initial status when connected - wait for Flutter to be ready
    delay(800); // Longer delay to ensure Flutter connection is fully established
    
    // Ensure characteristics are updated with current state for immediate reading
    uint8_t currentStatus[3] = {
      breakerstate ? 1 : 0,
      switchToggled ? 1 : 0,
      locked ? 1 : 0
    };
    statusChar.writeValue(currentStatus, 3);
    
    uint8_t currentLock = locked ? 1 : 0;
    lockChar.writeValue(&currentLock, 1);
    
    // Send current sense value - app will write its saved value to sync Arduino
    uint8_t currentSense = senseA_selected ? 0 : 1;
    senseChar.writeValue(&currentSense, 1);
    
    // Send BS14 mode notification to app
    String modeNotification = "{\"type\":\"mode_changed\",\"mode\":\"BS14\"}";
    commandChar.writeValue(modeNotification.c_str(), modeNotification.length());
    
    Serial.print("Connection sync: breaker=");
    Serial.print(currentStatus[0]);
    Serial.print(", switch=");
    Serial.print(currentStatus[1]);
    Serial.print(", locked=");
    Serial.print(currentStatus[2]);
    Serial.print(", sense=");
    Serial.print(senseA_selected ? "A" : "B");
    Serial.print(" (senseChar value=");
    Serial.print(currentSense);
    Serial.println("), mode=BS14");
    
    // Single status notification - Flutter will handle multiple reads
    send_status_to_flutter();
    Serial.println("Initial status sent to new connection");
  });

  BLE.setEventHandler(BLEDisconnected, [](BLEDevice central) {
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    bluetoothConnected = false;
    update_button_styles();
    update_lock_icon();
    set_leds();
    update_top_bar_bt_status();
  });

  // Set up initial values for characteristics that can be read
  uint8_t initialStatus[3] = {
    breakerstate ? 1 : 0,
    switchToggled ? 1 : 0,
    locked ? 1 : 0
  };
  statusChar.writeValue(initialStatus, 3);
  
  uint8_t initialLock = locked ? 1 : 0;
  lockChar.writeValue(&initialLock, 1);
  
  uint8_t initialSense = senseA_selected ? 0 : 1;
  senseChar.writeValue(&initialSense, 1);

  BLE.setAdvertisingInterval(100);
  BLE.setConnectable(true);
  BLE.advertise();

  pinMode(redled, OUTPUT);
  pinMode(greenled, OUTPUT);
  pinMode(blueled, OUTPUT);
  pinMode(sense, OUTPUT);
  pinMode(senseB, OUTPUT);
  pinMode(pin39, OUTPUT);
  pinMode(pin41, OUTPUT);
  pinMode(openInput, INPUT);
  pinMode(closeInput, INPUT);

  digitalWrite(redled, HIGH);
  digitalWrite(greenled, HIGH);
  digitalWrite(blueled, HIGH);

  digitalWrite(pin39, LOW);
  digitalWrite(pin41, LOW);

  breakerstate = true;
  // Set pins according to loaded sense selection
  if (senseA_selected) {
    digitalWrite(sense, LOW);
    digitalWrite(senseB, HIGH);
  } else {
    digitalWrite(sense, HIGH);
    digitalWrite(senseB, LOW);
  }

  set_leds();

  currentRotation = 270;
  lv_disp_t *disp = lv_disp_get_default();
  lv_disp_set_rotation(disp, LV_DISP_ROT_270);

  create_ui();
  create_top_bar();
  update_lock_icon();
  
  // Set proper initial positions by calling the rotation function
  // This ensures consistency with the rotate_screen_cb function
  // Reposition controls based on orientation
 if (currentRotation == 90 || currentRotation == 270) {
    // Landscape orientations - position controls to the right of switch container L/R, UP/DOWN
    lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -50, 65); // Buttons down by 10px more (total 25px)
    lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 70, 30); // Switch up by 40px
    // Position lock button above open/close buttons in landscape (relative to tight_container)
    if (lock_icon_btn && tight_container) {
      lv_obj_align_to(lock_icon_btn, tight_container, LV_ALIGN_OUT_TOP_MID, 0, 00); // Lock button down by 10px more (total 35px)
    }
  } else {
    // Portrait orientations - position controls below switch container, lock button above switch
    lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15);
    lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 160); // Moved up by 10px more (total 20px)
    // Move lock button above switch container in portrait
    lv_obj_align(lock_icon_btn, LV_ALIGN_TOP_MID, 0, 60); // Moved up by 10px more (total 20px)
  }
}

void loop() {
  lv_tick_inc(1);
  lv_timer_handler();

  BLEDevice central = BLE.central();
  if (central) {
    if (central.connected()) {
      bluetoothConnected = true;
      BLE.poll();

      // Send periodic status updates for better synchronization - DISABLED to reduce spam
      // Flutter app now handles sync with multiple read attempts
      /*
      static unsigned long lastPeriodicUpdate = 0;
      if (millis() - lastPeriodicUpdate > 1000) { // Every 1000ms to ensure sync without overwhelming
        send_status_to_flutter();
        lastPeriodicUpdate = millis();
      }
      */

      // Handle commandChar - check for JSON mode change requests OR 3-byte commands
      if (commandChar.written()) {
        uint8_t cmdLength = commandChar.valueLength();
        uint8_t* data = (uint8_t*)commandChar.value();
        
        // Try to parse as JSON first (for mode change requests)
        bool isJSON = false;
        String jsonString = "";
        for (int i = 0; i < cmdLength; i++) {
          char c = (char)data[i];
          if (c == '{') {
            isJSON = true;
          }
          if (isJSON) {
            jsonString += c;
            if (c == '}') break;
          }
        }
        
        if (isJSON && jsonString.length() > 0) {
          // Parse JSON mode change request
          Serial.print("Received JSON command: ");
          Serial.println(jsonString);
          
          if (jsonString.indexOf("mode_change_request") >= 0) {
            String modeStr = "";
            int modeStart = jsonString.indexOf("\"mode\"");
            if (modeStart >= 0) {
              int colonPos = jsonString.indexOf(":", modeStart);
              int quoteStart = jsonString.indexOf("\"", colonPos);
              int quoteEnd = jsonString.indexOf("\"", quoteStart + 1);
              if (quoteStart >= 0 && quoteEnd > quoteStart) {
                modeStr = jsonString.substring(quoteStart + 1, quoteEnd);
                modeStr.toUpperCase();
                
                Serial.print("Mode change request received: ");
                Serial.println(modeStr);
                
                // BS14 device only accepts BS14 mode - disconnect if any other mode
                if (modeStr != "BS14") {
                  Serial.print("ERROR: BS14 device only supports BS14 mode. Received: ");
                  Serial.print(modeStr);
                  Serial.println(" - disconnecting");
                  central.disconnect();
                  // Skip processing rest of this iteration
                } else {
                  // Send mode change confirmation
                  String response = "{\"type\":\"mode_changed\",\"mode\":\"BS14\"}";
                  commandChar.writeValue(response.c_str(), response.length());
                  Serial.println("BS14 mode confirmed");
                }
              }
            }
          }
        } else if (cmdLength >= 3) {
          // Handle 3-byte command
          Serial.println("RECEIVED 3-BYTE COMMAND from Flutter");
          bool newBreakerState = data[0] == 1;
          bool newSwitchState  = data[1] == 1;
          bool newLockState    = data[2] == 1;

          Serial.print("Flutter says: breaker=");
          Serial.print(newBreakerState);
          Serial.print(", switch=");
          Serial.print(newSwitchState);
          Serial.print(", locked=");
          Serial.println(newLockState);

          bool stateChanged = false;

          if (locked != newLockState) {
            locked = newLockState;
            uint8_t val = locked ? 1 : 0;
            lockChar.writeValue(&val, 1);   // sync back
            update_lock_icon();
            update_button_styles();
            stateChanged = true;
          }

          // Always update switchToggled and UI, and send status, even if locked
          bool switchStateChanged = (switchToggled != newSwitchState);
          switchToggled = newSwitchState;

          // Always update knob position and color to match app input
          if (switch_69) {
            if (switchToggled) {
              lv_obj_set_pos(switch_69, 2, 3); // UP position
              lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0);
            } else {
              lv_obj_set_pos(switch_69, 2, 72); // DOWN position
              lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0);
            }
          }

          // If breaker is locked closed and switch is toggled down, open the breaker visually
          if (locked && !breakerstate && !switchToggled) {
            set_breaker_state(true); // Open breaker visually
          }

          // Always update button styles and send status
          update_button_styles();
          send_status_to_flutter();

          // Mark stateChanged only if switch state actually changed
          if (switchStateChanged) {
            stateChanged = true;
          }

          // Only allow breaker state change if not locked
          if (!locked && newBreakerState != breakerstate) {
            Serial.print("Processing breaker state change: ");
            Serial.print(breakerstate ? "OPEN" : "CLOSED");
            Serial.print(" -> ");
            Serial.print(newBreakerState ? "OPEN" : "CLOSED");
            Serial.print(", switch state: ");
            Serial.print(switchToggled ? "UP" : "DOWN");
            Serial.print(", locked: ");
            Serial.println(locked ? "YES" : "NO");
            
            // Actually change the breaker state
            set_breaker_state(newBreakerState);
            update_button_styles();
            stateChanged = true;
          } else if (locked && newBreakerState != breakerstate) {
            Serial.println("Breaker command ignored - device is locked");
            // Still send status so app stays in sync
            stateChanged = true;
          } else if (newBreakerState == breakerstate) {
            Serial.println("Breaker command ignored - no state change needed");
          }

          if (stateChanged) {
            send_status_to_flutter();
            update_button_styles();
            set_leds();
          }
        }
      }

      // 🔴 Handle direct lockChar writes (but ignore echo writes from Arduino itself)
      if (lockChar.written()) {
        uint8_t newVal = lockChar.value()[0];
        bool newLockState = (newVal == 1);
        
        Serial.print("lockChar written with value: ");
        Serial.print(newVal);
        Serial.print(" (current locked state: ");
        Serial.print(locked);
        Serial.println(")");
        
        // Only change state if it's actually different AND it's likely from external source
        // Add a small delay check to avoid processing writes that happen immediately after our own writes
        static unsigned long lastLockWrite = 0;
        unsigned long currentTime = millis();
        
        if (locked != newLockState && (currentTime - lastLockWrite > 300)) {
          Serial.println("Processing external lock state change");
          locked = newLockState;
          update_lock_icon();
          update_button_styles();
          send_status_to_flutter();
          lastLockWrite = currentTime;
        } else if (locked == newLockState) {
          Serial.println("Lock write ignored - same state (likely echo)");
        } else {
          Serial.println("Lock write ignored - too soon after last write (likely echo)");
        }
      }

      // 🔴 Handle sense selection writes from Flutter app
      if (senseChar.written()) {
        uint8_t newVal = senseChar.value()[0];
        bool newSenseA = (newVal == 0); // 0 = sense A, 1 = sense B
        
        Serial.print("senseChar written with value: ");
        Serial.print(newVal);
        Serial.print(" (");
        Serial.print(newSenseA ? "A" : "B");
        Serial.print("), current Arduino value: ");
        Serial.println(senseA_selected ? "A" : "B");
        
        // Accept app's sense selection and sync Arduino to app's saved value
        // Only change state if it's actually different
        if (senseA_selected != newSenseA) {
          Serial.println("Processing sense selection change from Flutter - syncing Arduino to app");
          senseA_selected = newSenseA;
          
          Serial.print("Updated sense selection to: ");
          Serial.println(senseA_selected ? "A" : "B");
          
          // Update UI if modal is open
          if (btn_sense_a && btn_sense_b) {
            lv_obj_set_style_bg_color(btn_sense_a, senseA_selected ? lv_color_hex(0x00AA00) : lv_color_hex(0xCCCCCC), 0);
            lv_obj_set_style_bg_color(btn_sense_b, senseA_selected ? lv_color_hex(0xCCCCCC) : lv_color_hex(0x00AA00), 0);
          }
          
          // Apply the current breaker state with the new sense pin
          set_breaker_state(breakerstate);
          
          Serial.println("Sense selection change complete");
        } else {
          Serial.println("Sense write ignored - same state");
        }
        Serial.println("========================================");
      }

    } else {
      if (bluetoothConnected) {
        bluetoothConnected = false;
        update_button_styles();
        update_lock_icon();
        set_leds();
        update_top_bar_bt_status();
        BLE.advertise();
      }
    }
  }

  // hardware inputs when not locked
  if (!bluetoothConnected) {
    bool openInputActive  = digitalRead(openInput);
    bool closeInputActive = digitalRead(closeInput);

    if (!locked) {
      if (openInputActive && !breakerstate) {
        set_breaker_state(true);
        // Send status update even when not connected via BLE for consistency
        send_status_to_flutter();
      } else if (closeInputActive && !openInputActive && switchToggled && breakerstate) {
        set_breaker_state(false);
        // Send status update even when not connected via BLE for consistency  
        send_status_to_flutter();
      }
    }
  } else {
    // Even when connected via BLE, check hardware inputs and send status updates
    bool openInputActive  = digitalRead(openInput);
    bool closeInputActive = digitalRead(closeInput);

    if (!locked) {
      if (openInputActive && !breakerstate) {
        set_breaker_state(true);
        send_status_to_flutter();
      } else if (closeInputActive && !openInputActive && switchToggled && breakerstate) {
        set_breaker_state(false);
        send_status_to_flutter();
      }
    }
  }

  delay(1);
}

// ---------------- helper functions ----------------

static void send_status_to_flutter() {
  // Update characteristic values for Flutter to read (passive communication)
  if (BLE.central() && BLE.central().connected()) {
    uint8_t status[3] = {
      breakerstate ? 1 : 0,
      switchToggled ? 1 : 0,
      locked ? 1 : 0
    };
    
    // Only show debug output when state actually changes
    static uint8_t lastStatus[3] = {255, 255, 255}; // Initialize to impossible values
    bool statusChanged = (status[0] != lastStatus[0] || status[1] != lastStatus[1] || status[2] != lastStatus[2]);
    
    if (statusChanged) {
      Serial.print("STATUS UPDATE: B=");
      Serial.print(breakerstate ? "OPEN" : "CLOSED");
      Serial.print(", S=");
      Serial.print(switchToggled ? "UP" : "DOWN");
      Serial.print(", L=");
      Serial.println(locked ? "LOCKED" : "UNLOCKED");
      
      // Remember last sent status
      lastStatus[0] = status[0];
      lastStatus[1] = status[1];
      lastStatus[2] = status[2];
    }
    
    // Write characteristic values for Flutter to read
    statusChar.writeValue(status, 3); 
    
    uint8_t lockVal = locked ? 1 : 0;
    lockChar.writeValue(&lockVal, 1);
    
    // Also update sense characteristic to ensure Flutter stays in sync
    uint8_t senseVal = senseA_selected ? 0 : 1;
    senseChar.writeValue(&senseVal, 1);
    
  }
}

static void set_leds() {
  if (breakerstate) {
    digitalWrite(greenled, LOW);
    digitalWrite(redled, HIGH);
    digitalWrite(blueled, HIGH);
  } else {
    digitalWrite(redled, LOW);
    digitalWrite(greenled, HIGH);
    digitalWrite(blueled, HIGH);
  }
}

static void set_breaker_state(bool open) {
  // Only prevent closing if locked and trying to close
  if (locked && !open) return;
  bool previousState = breakerstate;
  breakerstate = open;
  if (previousState != breakerstate) {
    // Determine which sense pin to use
    int activeSensePin = senseA_selected ? sense : senseB;
    
    if (breakerstate) {
      digitalWrite(pin39, LOW);
      delay(5);
      digitalWrite(pin41, LOW);
      digitalWrite(activeSensePin, LOW);
      // Turn off inactive sense pin
      digitalWrite(senseA_selected ? senseB : sense, HIGH);
    } else {
      digitalWrite(pin41, HIGH);
      digitalWrite(pin39, HIGH);
      digitalWrite(activeSensePin, HIGH);
      // Turn off inactive sense pin
      digitalWrite(senseA_selected ? senseB : sense, HIGH);
    }
  }
  update_button_styles();
  set_leds();
}

static void switch_toggled_cb(lv_event_t *e) {
  // Debouncing - ignore rapid consecutive presses
  unsigned long currentTime = millis();
  if (currentTime - last_switch_toggle < 300) { // 300ms debounce
    Serial.println("Switch toggle ignored - too soon after last toggle");
    return;
  }
  last_switch_toggle = currentTime;
  
  Serial.println("=== CUSTOM VERTICAL SWITCH TOGGLED ON ARDUINO ===");
  
  // Toggle the state
  switchToggled = !switchToggled;
  
  Serial.print("Switch is now: ");
  Serial.println(switchToggled ? "UP" : "DOWN");
  
  // Update the visual position and color of the switch knob
  if (switchToggled) {
    // UP position (switch ON)
    lv_obj_set_pos(switch_69, 2, 3); // Top of container
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for ON
  } else {
    // DOWN position (switch OFF)  
    lv_obj_set_pos(switch_69, 2, 72); // Adjusted for smaller knob and container: 150-70-8=72
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for OFF (position shows state)
  }
  
  // If breaker is locked closed and switch is toggled down, open the breaker
  if (locked && !breakerstate && !switchToggled) {
    Serial.println("Breaker locked closed, switch toggled down - opening breaker");
    set_breaker_state(true);
  } else if (!switchToggled && !breakerstate) {
    Serial.println("Switch down and breaker closed - opening breaker");
    set_breaker_state(true);
  } else {
    Serial.println("Updating button styles only");
    update_button_styles();
  }
  // ALWAYS send status update when switch is toggled, regardless of BLE connection
  send_status_to_flutter();
  Serial.println("=== SWITCH TOGGLE COMPLETE ===");
}

static void open_btn_cb(lv_event_t *e) {
  Serial.println("=== OPEN BUTTON PRESSED ON ARDUINO ===");
  if (locked) {
    Serial.println("Button press ignored - device is locked");
    return;
  }
  set_breaker_state(true);
  // ALWAYS send status update when button is pressed, regardless of BLE connection
  send_status_to_flutter();
  Serial.println("=== OPEN BUTTON COMPLETE ===");
}

static void close_btn_cb(lv_event_t *e) {
  Serial.println("=== CLOSE BUTTON PRESSED ON ARDUINO ===");
  if (locked) {
    Serial.println("Button press ignored - device is locked");
    return;
  }
  if (switchToggled && breakerstate) {
    Serial.println("Closing breaker (switch is up and breaker is open)");
    set_breaker_state(false);
    // ALWAYS send status update when button is pressed, regardless of BLE connection
    send_status_to_flutter();
  } else {
    Serial.print("Close button ignored - switchToggled=");
    Serial.print(switchToggled);
    Serial.print(", breakerstate=");
    Serial.println(breakerstate);
  }
  Serial.println("=== CLOSE BUTTON COMPLETE ===");
}

static void rotate_screen_cb(lv_event_t *e) {
  static unsigned long lastRotationTime = 0;
  unsigned long currentTime = millis();
  if (currentTime - lastRotationTime < 500) return;
  lastRotationTime = currentTime;

  currentRotation = (currentRotation + 90) % 360;
  lv_disp_t *disp = lv_disp_get_default();

  switch (currentRotation) {
    case 0:   lv_disp_set_rotation(disp, LV_DISP_ROT_NONE); break;
    case 90:  lv_disp_set_rotation(disp, LV_DISP_ROT_90);   break;
    case 180: lv_disp_set_rotation(disp, LV_DISP_ROT_180);  break;
    case 270: lv_disp_set_rotation(disp, LV_DISP_ROT_270);  break;
  }
  
  // Recreate top bar after rotation to ensure correct positioning
  create_top_bar();
  // Update Bluetooth status font size based on new orientation
  update_top_bar_bt_status();
  
  // Resize ui_container to match new screen dimensions after rotation
  if (ui_container) {
    lv_obj_t *scr = lv_scr_act();
    int32_t scr_width = lv_obj_get_width(scr);
    int32_t scr_height = lv_obj_get_height(scr);
    lv_obj_set_size(ui_container, scr_width, scr_height);
    lv_obj_align(ui_container, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_move_background(ui_container); // Ensure top bar stays on top
  }
  
  if (currentRotation == 90 || currentRotation == 270) {
    // Landscape orientations - position controls to the right of switch container L/R, UP/DOWN
    lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -50, 65); // Buttons down by 10px more (total 25px)
    lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 70, 30); // Switch up by 40px
    // Position lock button above open/close buttons in landscape (relative to tight_container)
    if (lock_icon_btn && tight_container) {
      lv_obj_align_to(lock_icon_btn, tight_container, LV_ALIGN_OUT_TOP_MID, 0, 00); // Lock button down by 10px more (total 35px)
    }
  } else {
    // Portrait orientations - position controls below switch container, lock button above switch
    // Ensure proper spacing - switch at top (below top bar), buttons at bottom
    lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 160); // Switch at top, below 60px top bar
    lv_obj_align(lock_icon_btn, LV_ALIGN_TOP_MID, 0, 60); // Lock button just below top bar
    lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15); // Buttons at bottom with margin
  }
  
  // Ensure background color is applied and covers full screen
  if (ui_container) {
    if (breakerstate) {
      lv_obj_set_style_bg_color(ui_container, lv_color_hex(0x00AA00), 0);
    } else {
      lv_obj_set_style_bg_color(ui_container, lv_color_hex(0xAA0000), 0);
    }
    lv_obj_set_style_bg_opa(ui_container, LV_OPA_COVER, 0);
  }
  
  lv_refr_now(lv_disp_get_default());
}

static void lock_icon_event_cb(lv_event_t *e) {
  uint32_t code = lv_event_get_code(e);
  
  if (code == LV_EVENT_PRESSED) {
    lock_press_start = millis();
    Serial.println("Lock button pressed - hold for 400ms");
    
    // Simple visual feedback - just change the text
    if (locked) {
      lv_label_set_text(lock_icon_label, "HOLD TO UNLOCK");
    } else {
      lv_label_set_text(lock_icon_label, "HOLD TO LOCK");
    }
    
  } else if (code == LV_EVENT_PRESSING) {
    if (lock_press_start && (millis() - lock_press_start > 400)) {
      lock_press_start = 0;
      locked = !locked;
      Serial.print("Lock toggled - locked: ");
      Serial.println(locked);

      // sync with BLE - track when we write to avoid echo processing
      uint8_t val = locked ? 1 : 0;
      lockChar.writeValue(&val, 1);
      
      // Update timing to prevent echo processing
      static unsigned long lastLockWrite = 0;
      lastLockWrite = millis();

      update_lock_icon();
      update_button_styles();
      // ALWAYS send status update when lock is toggled, regardless of BLE connection
      send_status_to_flutter();
    }
    
  } else if (code == LV_EVENT_RELEASED) {
    // Always restore to normal state when released
    update_lock_icon();
    lock_press_start = 0;
  }
}

static void update_lock_icon() {
  if (!lock_icon_label || !lock_icon_btn) return;
  if (locked) {
    lv_label_set_text(lock_icon_label, "HOLD TO UNLOCK");
    lv_obj_set_style_bg_color(lock_icon_btn, lv_color_hex(0xFF9800), 0);
  } else {
    lv_label_set_text(lock_icon_label, "HOLD TO LOCK");
    lv_obj_set_style_bg_color(lock_icon_btn, lv_color_hex(0x1976D2), 0);
  }
}

static void settings_btn_cb(lv_event_t *e) {
  Serial.println("=== SETTINGS BUTTON PRESSED ===");
  
  // Create modal dialog if it doesn't exist
  if (!settings_modal) {
    settings_modal = lv_obj_create(lv_scr_act());
    lv_obj_set_size(settings_modal, 500, 390); // Reduced height to fit content tightly
    lv_obj_align(settings_modal, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(settings_modal, lv_color_hex(0xDDDDDD), 0);
    lv_obj_set_style_border_width(settings_modal, 3, 0);
    lv_obj_set_style_border_color(settings_modal, lv_color_hex(0x000000), 0);
    lv_obj_set_style_pad_all(settings_modal, 20, 0);

    // Title
    lv_obj_t *title = lv_label_create(settings_modal);
    lv_label_set_text(title, "SENSE SELECTION");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_34, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    // Sense A button
    // Create a horizontal container for Sense A and Sense B buttons
    lv_obj_t *sense_row = lv_obj_create(settings_modal);
    lv_obj_set_size(sense_row, 440, 80);
    lv_obj_align(sense_row, LV_ALIGN_TOP_MID, 0, 60); // Move up 20px
    lv_obj_set_flex_flow(sense_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_bg_opa(sense_row, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(sense_row, 0, 0);
    lv_obj_set_style_pad_all(sense_row, 0, 0);
    lv_obj_set_style_pad_row(sense_row, 0, 0);
    lv_obj_set_style_pad_column(sense_row, 20, 0); // Space between buttons
    lv_obj_set_scroll_dir(sense_row, LV_DIR_NONE);

    // Sense A button
    btn_sense_a = lv_btn_create(sense_row);
    lv_obj_set_size(btn_sense_a, 200, 75);
    lv_obj_set_style_border_width(btn_sense_a, 4, 0);
    lv_obj_set_style_border_color(btn_sense_a, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(btn_sense_a, senseA_selected ? lv_color_hex(0x2196F3) : lv_color_hex(0xFFFFFF), 0);
    lv_obj_add_event_cb(btn_sense_a, sense_a_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_a = lv_label_create(btn_sense_a);
    lv_label_set_text(label_a, "SENSE A");
    lv_obj_set_style_text_font(label_a, &lv_font_montserrat_32, 0);
    lv_obj_center(label_a);

    // Sense B button
    btn_sense_b = lv_btn_create(sense_row);
    lv_obj_set_size(btn_sense_b, 200, 75);
    lv_obj_set_style_border_width(btn_sense_b, 4, 0);
    lv_obj_set_style_border_color(btn_sense_b, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(btn_sense_b, senseA_selected ? lv_color_hex(0xFFFFFF) : lv_color_hex(0x2196F3), 0);
    lv_obj_add_event_cb(btn_sense_b, sense_b_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_b = lv_label_create(btn_sense_b);
    lv_label_set_text(label_b, "SENSE B");
    lv_obj_set_style_text_font(label_b, &lv_font_montserrat_32, 0);
    lv_obj_center(label_b);

    // Help button
    lv_obj_t *btn_help = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_help, 380, 60);
    lv_obj_align(btn_help, LV_ALIGN_TOP_MID, 0, 150); // Just below sense row
    lv_obj_set_style_bg_color(btn_help, lv_color_hex(0x2196F3), 0);
    lv_obj_set_style_border_width(btn_help, 3, 0);
    lv_obj_set_style_border_color(btn_help, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(btn_help, [](lv_event_t *e) {
      // Create a modal dialog for help info
      lv_obj_t *help_modal = lv_obj_create(lv_scr_act());
      lv_obj_set_size(help_modal, 440, 340);
      lv_obj_align(help_modal, LV_ALIGN_CENTER, 0, 0);
      lv_obj_set_style_bg_color(help_modal, lv_color_hex(0xFFFFFF), 0);
      lv_obj_set_style_border_width(help_modal, 3, 0);
      lv_obj_set_style_border_color(help_modal, lv_color_hex(0x2196F3), 0);
      lv_obj_set_style_pad_all(help_modal, 24, 0);

      // Title
      lv_obj_t *help_title = lv_label_create(help_modal);
      lv_label_set_text(help_title, "HELP & CONTACT");
      lv_obj_set_style_text_font(help_title, &lv_font_montserrat_30, 0);
      lv_obj_align(help_title, LV_ALIGN_TOP_MID, 0, 8);

      // Email
      lv_obj_t *email_label = lv_label_create(help_modal);
      lv_label_set_text(email_label, "Email: andi@relport.com");
      lv_obj_set_style_text_font(email_label, &lv_font_montserrat_22, 0);
      lv_obj_align(email_label, LV_ALIGN_TOP_LEFT, 10, 60);

      // Phone
      lv_obj_t *phone_label = lv_label_create(help_modal);
      lv_label_set_text(phone_label, "Phone: 509-961-2744");
      lv_obj_set_style_text_font(phone_label, &lv_font_montserrat_22, 0);
      lv_obj_align(phone_label, LV_ALIGN_TOP_LEFT, 10, 100);

      // Website
      lv_obj_t *web_label = lv_label_create(help_modal);
      lv_label_set_text(web_label, "Website: relport.com");
      lv_obj_set_style_text_font(web_label, &lv_font_montserrat_22, 0);
      lv_obj_align(web_label, LV_ALIGN_TOP_LEFT, 10, 140);

      // Close button for help modal
      lv_obj_t *btn_close_help = lv_btn_create(help_modal);
      lv_obj_set_size(btn_close_help, 180, 60);
      lv_obj_align(btn_close_help, LV_ALIGN_BOTTOM_MID, 0, -10);
      lv_obj_set_style_bg_color(btn_close_help, lv_color_hex(0xFF9800), 0);
      lv_obj_set_style_border_width(btn_close_help, 2, 0);
      lv_obj_set_style_border_color(btn_close_help, lv_color_hex(0x000000), 0);
      lv_obj_set_style_radius(btn_close_help, 5, 0);
      lv_obj_t *close_help_label = lv_label_create(btn_close_help);
      lv_label_set_text(close_help_label, "CLOSE");
      lv_obj_set_style_text_font(close_help_label, &lv_font_montserrat_22, 0);
      lv_obj_center(close_help_label);
      lv_obj_add_event_cb(btn_close_help, [](lv_event_t *e) {
        // Delete the help modal (parent of the button)
        lv_obj_del(lv_obj_get_parent(lv_event_get_target(e)));
      }, LV_EVENT_CLICKED, NULL);
    }, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_help = lv_label_create(btn_help);
    lv_label_set_text(label_help, "HELP");
    lv_obj_set_style_text_font(label_help, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label_help, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(label_help);

    // Close button
    lv_obj_t *btn_close_settings = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_close_settings, 220, 70);
    lv_obj_align(btn_close_settings, LV_ALIGN_BOTTOM_MID, 0, 10); // Place at bottom with minimal margin
    lv_obj_set_style_border_width(btn_close_settings, 3, 0);
    lv_obj_set_style_border_color(btn_close_settings, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(btn_close_settings, close_settings_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_close = lv_label_create(btn_close_settings);
    lv_label_set_text(label_close, "CLOSE");
    lv_obj_set_style_text_font(label_close, &lv_font_montserrat_28, 0);
    lv_obj_center(label_close);

    // Disconnect Bluetooth button (only show if connected)
    lv_obj_t *btn_disconnect_bt = lv_btn_create(settings_modal);
    lv_obj_set_size(btn_disconnect_bt, 380, 60);
    // Move Disconnect button immediately above Close button with minimal spacing
    lv_obj_align(btn_disconnect_bt, LV_ALIGN_BOTTOM_MID, 0, 90); // Initial placement
    lv_obj_align_to(btn_disconnect_bt, btn_close_settings, LV_ALIGN_OUT_TOP_MID, 0, -10); // 10px gap above Close
    lv_obj_set_style_bg_color(btn_disconnect_bt, lv_color_hex(0xFF4444), 0);
    lv_obj_set_style_border_width(btn_disconnect_bt, 3, 0);
    lv_obj_set_style_border_color(btn_disconnect_bt, lv_color_hex(0x000000), 0);
    lv_obj_add_event_cb(btn_disconnect_bt, disconnect_bluetooth_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *label_disconnect = lv_label_create(btn_disconnect_bt);
    lv_label_set_text(label_disconnect, "DISCONNECT BLUETOOTH");
    lv_obj_set_style_text_font(label_disconnect, &lv_font_montserrat_24, 0);
    lv_obj_set_style_text_color(label_disconnect, lv_color_hex(0xFFFFFF), 0);
    lv_obj_center(label_disconnect);
  }
  
  // Update button colors based on current selection
  if (btn_sense_a && btn_sense_b) {
    if (senseA_selected) {
      lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0x00AA00), 0); // Green
      lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0xCCCCCC), 0); // Gray
    } else {
      lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0xCCCCCC), 0); // Gray
      lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0x00AA00), 0); // Green
    }
  }

  // Show modal by centering it
  if (settings_modal) {
    lv_obj_align(settings_modal, LV_ALIGN_CENTER, 0, 0);
  }
  Serial.println("Settings modal opened");
}

static void sense_a_btn_cb(lv_event_t *e) {
  Serial.println("Sense A selected");
  senseA_selected = true;
  
  // Update visuals
  lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0x00AA00), 0); // Green
  lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0xCCCCCC), 0); // Gray
  
  // Send update to app via BLE
  if (BLE.central() && BLE.central().connected()) {
    uint8_t value = 0; // Sense A
    senseChar.writeValue(&value, 1);
  }
  
  // Apply the current breaker state with the new sense pin
  set_breaker_state(breakerstate);
}

static void sense_b_btn_cb(lv_event_t *e) {
  Serial.println("Sense B selected");
  senseA_selected = false;
  
  // Update visuals
  lv_obj_set_style_bg_color(btn_sense_a, lv_color_hex(0xCCCCCC), 0); // Gray
  lv_obj_set_style_bg_color(btn_sense_b, lv_color_hex(0x00AA00), 0); // Green
  
  // Send update to app via BLE
  if (BLE.central() && BLE.central().connected()) {
    uint8_t value = 1; // Sense B
    senseChar.writeValue(&value, 1);
  }
  
  // Apply the current breaker state with the new sense pin
  set_breaker_state(breakerstate);
}

static void disconnect_bluetooth_cb(lv_event_t *e) {
  Serial.println("Disconnect Bluetooth button pressed");
  BLEDevice central = BLE.central();
  if (central && central.connected()) {
    Serial.println("Disconnecting from central device...");
    central.disconnect();
    bluetoothConnected = false;
    update_top_bar_bt_status();
    update_button_styles();
    update_lock_icon();
    set_leds();
    Serial.println("Disconnected successfully");
  } else {
    Serial.println("No connected device to disconnect");
  }
}

static void close_settings_cb(lv_event_t *e) {
  Serial.println("Closing settings");
  if (settings_modal) {
    // Hide settings modal by moving off-screen
    lv_obj_set_pos(settings_modal, -5000, -5000);
  }
}

static void update_button_styles() {
  if (!btn_open || !btn_close || !ui_container || !switch_container || !tight_container || !switch_69) return;
  
  Serial.print("UPDATE STYLES: locked=");
  Serial.print(locked);
  Serial.print(", switchToggled=");
  Serial.print(switchToggled);
  Serial.print(", breakerstate=");
  Serial.println(breakerstate);
  
  // Update background color based on breaker state
  if (breakerstate) {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0x00AA00), 0); // Green for open
  } else {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0xAA0000), 0); // Red for closed
  }
  
  // Always start by clearing everything
  lv_obj_clear_state(btn_open, LV_STATE_DISABLED);
  lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
  // Hide overlays by making them transparent and moving off-screen
  if (open_btn_overlay) {
    lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_pos(open_btn_overlay, -5000, -5000);
  }
  if (close_btn_overlay) {
    lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_TRANSP, 0);
    lv_obj_set_pos(close_btn_overlay, -5000, -5000);
  }

  if (locked) {
    Serial.println("LOCKED STATE - disabling all buttons");
    // When LOCKED: disable everything - buttons are disabled visually by their disabled state
    lv_obj_add_state(btn_open, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x333300), 0);
    lv_obj_add_state(btn_close, LV_STATE_DISABLED);
    lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x330000), 0);
    // Do not disable switch when locked
    
    // Hide all overlays when locked - buttons are disabled visually by their disabled state
    if (open_btn_overlay) {
      lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_TRANSP, 0);
      lv_obj_set_pos(open_btn_overlay, -5000, -5000);
    }
    if (close_btn_overlay) {
      lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_TRANSP, 0);
      lv_obj_set_pos(close_btn_overlay, -5000, -5000);
    }
    
  } else {
    Serial.println("UNLOCKED STATE - normal operation");
    // When UNLOCKED: normal operation
    lv_obj_clear_state(switch_69, LV_STATE_DISABLED);
    
    if (breakerstate) {
      // Breaker is OPEN
      lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x00AA00), 0);
      
      if (!switchToggled) {
        Serial.println("Switch is DOWN - disabling close button with prohibition overlay");
        // Switch DOWN - disable close button with prohibition overlay (safety rule)
        lv_obj_add_state(btn_close, LV_STATE_DISABLED);
        // Set disabled state background color specifically to override LVGL's automatic lightening
        lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x110000), LV_STATE_DISABLED);
        // Show prohibition overlay
        if (close_btn_overlay) {
          lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_40, 0);
          lv_obj_align(close_btn_overlay, LV_ALIGN_CENTER, 0, 0);
        }
        // Keep open button normal (no overlay)
        if (open_btn_overlay) {
          lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_TRANSP, 0);
          lv_obj_set_pos(open_btn_overlay, -5000, -5000);
        }
      } else {
        Serial.println("Switch is UP - all buttons enabled, no overlays");
        // Switch UP - both buttons enabled, no overlays
        lv_obj_set_style_bg_color(btn_close, lv_color_hex(0x880000), 0); // Medium-dark red, darker than background but still visible
        lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
        if (close_btn_overlay) {
          lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_TRANSP, 0);
          lv_obj_set_pos(close_btn_overlay, -5000, -5000);
        }
        if (open_btn_overlay) {
          lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_TRANSP, 0);
          lv_obj_set_pos(open_btn_overlay, -5000, -5000);
        }
      }
    } else {
      Serial.println("Breaker is CLOSED - all buttons enabled, no overlays");
      // Breaker is CLOSED - both buttons enabled, no overlays
      lv_obj_set_style_bg_color(btn_open, lv_color_hex(0x005500), 0);
      lv_obj_set_style_bg_color(btn_close, lv_color_hex(0xAA0000), 0);
      lv_obj_clear_state(btn_close, LV_STATE_DISABLED);
      if (close_btn_overlay) {
        lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_TRANSP, 0);
        lv_obj_set_pos(close_btn_overlay, -5000, -5000);
      }
      if (open_btn_overlay) {
        lv_obj_set_style_bg_opa(open_btn_overlay, LV_OPA_TRANSP, 0);
        lv_obj_set_pos(open_btn_overlay, -5000, -5000);
      }
    }
  }
}

// Create UI function - called once at startup
static void create_ui() {
  // Main container - match BS69 approach
  lv_obj_t *scr = lv_scr_act();
  ui_container = lv_obj_create(scr);
  int32_t scr_width = lv_obj_get_width(scr);
  int32_t scr_height = lv_obj_get_height(scr);
  lv_obj_set_size(ui_container, scr_width, scr_height);
  lv_obj_align(ui_container, LV_ALIGN_TOP_MID, 0, 0); // Align to top of screen
  lv_obj_move_background(ui_container); // Ensure top bar is visually above
  // Set initial background color based on breaker state
  if (breakerstate) {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0x00AA00), 0); // Green for open
  } else {
    lv_obj_set_style_bg_color(ui_container, lv_color_hex(0xAA0000), 0); // Red for closed
  }
  lv_obj_set_style_bg_opa(ui_container, LV_OPA_COVER, 0); // Ensure background is fully opaque
  lv_obj_set_scroll_dir(ui_container, LV_DIR_NONE);

  // Switch container - position will be set by rotation handler
  switch_container = lv_obj_create(ui_container);
  lv_obj_set_size(switch_container, 250, 320); // Made smaller: reduced from 280x380 to 250x320
  // Position will be set by rotate_screen_cb() or initial setup positioning
  lv_obj_set_style_bg_color(switch_container, lv_color_hex(0xFFFF00), 0); // Yellow background
  lv_obj_set_style_border_width(switch_container, 5, 0);
  lv_obj_set_style_border_color(switch_container, lv_color_hex(0x000000), 0);
  lv_obj_set_flex_flow(switch_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(switch_container, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER); // Space around for better spacing
  lv_obj_set_style_pad_all(switch_container, 15, 0); // More padding
  lv_obj_set_scroll_dir(switch_container, LV_DIR_NONE); // Make non-scrollable

  // UP label
  lv_obj_t *label_up = lv_label_create(switch_container);
  lv_label_set_text(label_up, "UP");
  lv_obj_set_style_text_color(label_up, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_up, &lv_font_montserrat_48, 0);

// Middle container for switch and 69 label (NO FLEX - manual positioning)
  lv_obj_t *middle_container = lv_obj_create(switch_container);
  lv_obj_set_size(middle_container, 220, 150); // Made smaller: reduced from 250x140 to 220x150
  lv_obj_align(middle_container, LV_ALIGN_LEFT_MID, 0, -50); // Center in switch_container
  // REMOVE FLEX - this was causing transform conflicts
  // lv_obj_set_flex_flow(middle_container, LV_FLEX_FLOW_ROW);
  // lv_obj_set_flex_align(middle_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(middle_container, 0, 0);
  lv_obj_set_style_border_width(middle_container, 0, 0);
  lv_obj_set_style_bg_opa(middle_container, LV_OPA_TRANSP, 0);
  lv_obj_set_scroll_dir(middle_container, LV_DIR_NONE); // Make non-scrollable

  // 69 label - manually positioned on the left
  lv_obj_t *label_69 = lv_label_create(middle_container);
  lv_label_set_text(label_69, "69");
  lv_obj_set_style_text_color(label_69, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_69, &lv_font_montserrat_48, 0);
  lv_obj_set_pos(label_69, 20, 45); // Adjusted for smaller container

  // Custom vertical switch using two buttons arranged vertically
  // Create a container for our custom vertical switch
  lv_obj_t *vertical_switch_container = lv_obj_create(middle_container);
  lv_obj_set_size(vertical_switch_container, 85, 150); // Made smaller: reduced from 100x180 to 85x150
  lv_obj_set_pos(vertical_switch_container, 110, 0); // Adjusted position for smaller container
  lv_obj_set_style_bg_color(vertical_switch_container, lv_color_hex(0xFFFFFF), 0); // White background for switch track
  lv_obj_set_style_border_width(vertical_switch_container, 2, 0);
  lv_obj_set_style_border_color(vertical_switch_container, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(vertical_switch_container, 42, 0); // Adjusted radius for smaller size
  lv_obj_set_style_pad_all(vertical_switch_container, 5, 0); // Slightly more padding
  lv_obj_set_scroll_dir(vertical_switch_container, LV_DIR_NONE); // Make non-scrollable
  
  // Create the switch "knob" - this will move up/down
  switch_69 = lv_btn_create(vertical_switch_container);
  lv_obj_set_size(switch_69, 70, 70); // Made smaller: reduced from 85x85 to 70x70
  lv_obj_set_style_radius(switch_69, 35, 0); // Make it circular (half of 70)
  lv_obj_set_style_border_width(switch_69, 1, 0);
  lv_obj_set_style_border_color(switch_69, lv_color_hex(0x000000), 0);
  lv_obj_set_scroll_dir(switch_69, LV_DIR_NONE); // Make non-scrollable
  
  // Position the knob based on switch state
  if (switchToggled) {
    // UP position (switch ON)
    lv_obj_set_pos(switch_69, 2, 3); // Top of container
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for ON
  } else {
    // DOWN position (switch OFF)  
    lv_obj_set_pos(switch_69, 2, 72); // Adjusted for smaller knob and container: 150-70-8=72
    lv_obj_set_style_bg_color(switch_69, lv_color_hex(0x2196F3), 0); // Blue for OFF (will distinguish by position)
  }
  
  // Add click event to toggle the switch
  lv_obj_add_event_cb(switch_69, switch_toggled_cb, LV_EVENT_CLICKED, NULL);
  
  // Disable switch if locked
  if (locked) lv_obj_add_state(switch_69, LV_STATE_DISABLED);

  // DOWN label
  lv_obj_t *label_down = lv_label_create(switch_container);
  lv_label_set_text(label_down, "DOWN");
  lv_obj_set_style_text_color(label_down, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_down, &lv_font_montserrat_48, 0);

  // Button container for open/close - position will be set by rotation handler
  tight_container = lv_obj_create(ui_container);
  lv_obj_set_size(tight_container, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_flex_flow(tight_container, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(tight_container, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_set_style_pad_all(tight_container, 10, 0);
  lv_obj_set_style_pad_row(tight_container, 15, 0);
  lv_obj_set_style_border_width(tight_container, 0, 0);
  lv_obj_set_style_bg_opa(tight_container, LV_OPA_TRANSP, 0);
  // Position will be set by rotate_screen_cb() or initial setup positioning

  // Open button
  btn_open = lv_btn_create(tight_container);
  lv_obj_set_size(btn_open, 350, 120); // Made taller: increased from 100 to 120
  lv_obj_set_style_border_width(btn_open, 3, 0);
  lv_obj_set_style_border_color(btn_open, lv_color_hex(0x000000), 0);
  lv_obj_add_event_cb(btn_open, open_btn_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *label_open = lv_label_create(btn_open);
  lv_label_set_text(label_open, "OPEN");
  lv_obj_set_style_text_color(label_open, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_open, &lv_font_montserrat_48, 0);
  lv_obj_center(label_open);

  // Close button
  btn_close = lv_btn_create(tight_container);
  lv_obj_set_size(btn_close, 350, 120); // Made taller: increased from 100 to 120
  lv_obj_set_style_border_width(btn_close, 3, 0);
  lv_obj_set_style_border_color(btn_close, lv_color_hex(0x000000), 0);
  lv_obj_add_event_cb(btn_close, close_btn_cb, LV_EVENT_CLICKED, NULL);

  lv_obj_t *label_close = lv_label_create(btn_close);
  lv_label_set_text(label_close, "CLOSE");
  lv_obj_set_style_text_color(label_close, lv_color_hex(0x000000), 0);
  lv_obj_set_style_text_font(label_close, &lv_font_montserrat_48, 0);
  lv_obj_center(label_close);

  // Create overlay for disabled state (circle slash symbol)
  close_btn_overlay = lv_obj_create(btn_close);
  lv_obj_set_size(close_btn_overlay, 350, 120); // Made taller to match button: increased from 100 to 120
  lv_obj_align(close_btn_overlay, LV_ALIGN_CENTER, 0, 0);
  lv_obj_set_style_bg_color(close_btn_overlay, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_40, 0);
  lv_obj_set_style_radius(close_btn_overlay, 0, 0);
  lv_obj_set_style_bg_opa(close_btn_overlay, LV_OPA_TRANSP, 0);  // Initially hidden
  lv_obj_set_pos(close_btn_overlay, -5000, -5000);  // Move off-screen initially
  
  // Create circle (O)
  lv_obj_t *circle_label = lv_label_create(close_btn_overlay);
  lv_label_set_text(circle_label, "O");
  lv_obj_set_style_text_color(circle_label, lv_color_hex(0xFF0000), 0);
  lv_obj_set_style_text_font(circle_label, &lv_font_montserrat_48, 0);
  lv_obj_center(circle_label);

  lv_obj_t *slash_label = lv_label_create(close_btn_overlay);
  lv_label_set_text(slash_label, "/");
  lv_obj_set_style_text_color(slash_label, lv_color_hex(0xFF0000), 0);
  lv_obj_set_style_text_font(slash_label, &lv_font_montserrat_48, 0);
  lv_obj_center(slash_label);

  // Note: open_btn_overlay is no longer needed since we removed the lock icon
  // Keeping the variable declaration for compatibility but not creating the overlay
  open_btn_overlay = NULL;

  // Lock icon button - create as direct child of ui_container for independent positioning
  lock_icon_btn = lv_btn_create(ui_container);
  lv_obj_set_size(lock_icon_btn, 260, 90); // Made larger: increased from 220x80 to 260x90 for easier pressing
  lv_obj_set_style_radius(lock_icon_btn, 8, 0); // Slightly more rounded for better feel
  lv_obj_set_style_border_width(lock_icon_btn, 4, 0); // Thicker border for better visual feedback
  lv_obj_set_style_border_color(lock_icon_btn, lv_color_hex(0x000000), 0);
  
  // Register for all touch events
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSED, NULL);
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_PRESSING, NULL);
  lv_obj_add_event_cb(lock_icon_btn, lock_icon_event_cb, LV_EVENT_RELEASED, NULL);
  
  // Create label
  lock_icon_label = lv_label_create(lock_icon_btn);
  lv_obj_set_style_text_font(lock_icon_label, &lv_font_montserrat_24, 0);
  lv_obj_set_style_text_color(lock_icon_label, lv_color_hex(0x000000), 0);
  lv_obj_center(lock_icon_label);

  // Note: Settings and Rotate buttons are now created in create_top_bar()
  
  // Position all elements based on current rotation (matching BS69 approach)
  if (currentRotation == 90 || currentRotation == 270) {
    // Landscape orientations - position controls to the right of switch container L/R, UP/DOWN
    lv_obj_align(tight_container, LV_ALIGN_RIGHT_MID, -50, 65);
    lv_obj_align(switch_container, LV_ALIGN_LEFT_MID, 70, 30);
    // Position lock button above open/close buttons in landscape (relative to tight_container)
    if (lock_icon_btn && tight_container) {
      lv_obj_align_to(lock_icon_btn, tight_container, LV_ALIGN_OUT_TOP_MID, 0, 0);
    }
  } else {
    // Portrait orientations - position controls below switch container, lock button above switch
    // Account for top bar (60px in portrait) and ensure proper spacing
    lv_obj_align(switch_container, LV_ALIGN_TOP_MID, 0, 160); // Switch at top, below top bar
    lv_obj_align(lock_icon_btn, LV_ALIGN_TOP_MID, 0, 60); // Lock button just below top bar
    lv_obj_align(tight_container, LV_ALIGN_BOTTOM_MID, 0, 15); // Buttons at bottom
  }

  // Initialize button styles
  update_button_styles();
}

// Create top bar with rotate, Bluetooth, and settings
static void create_top_bar() {
  // Delete existing top bar if it exists
  if (top_bar) {
    lv_obj_del(top_bar);
    top_bar = NULL;
  }
  
  // Get screen dimensions
  lv_obj_t *scr = lv_scr_act();
  int32_t screen_width = lv_obj_get_width(scr);
  int32_t screen_height = lv_obj_get_height(scr);
  
  // Check orientation for conditional sizing
  bool isPortrait = (currentRotation == 0 || currentRotation == 180);
  
  // Button sizes - smaller in portrait
  int btn_width_settings = isPortrait ? 100 : 140;
  int btn_width_rotate = isPortrait ? 100 : 140;
  int btn_height = isPortrait ? 40 : 50;
  int btn_spacing = isPortrait ? 10 : 20;
  int bt_spacing = isPortrait ? 10 : 20;
  const lv_font_t *btn_font = isPortrait ? &lv_font_montserrat_16 : &lv_font_montserrat_20;
  
  // Create top bar on screen (will be moved to foreground)
  top_bar = lv_obj_create(scr);
  lv_obj_set_size(top_bar, screen_width, isPortrait ? 60 : 70); // Smaller height in portrait
  lv_obj_align(top_bar, LV_ALIGN_TOP_MID, 0, 0);
  lv_obj_set_style_bg_color(top_bar, lv_color_hex(0x000000), 0); // Black background
  lv_obj_set_style_bg_opa(top_bar, LV_OPA_COVER, 0);
  lv_obj_set_style_border_width(top_bar, 0, 0);
  lv_obj_set_style_radius(top_bar, 0, 0); // Squared edges - no rounded corners
  lv_obj_set_scroll_dir(top_bar, LV_DIR_NONE);
  // Move to foreground to ensure it's always on top
  lv_obj_move_foreground(top_bar);
  
  // Rotate button on the left (swapped from right)
  btn_rotate = lv_btn_create(top_bar);
  lv_obj_set_size(btn_rotate, btn_width_rotate, btn_height);
  lv_obj_align(btn_rotate, LV_ALIGN_LEFT_MID, 5, 0);
  lv_obj_set_style_bg_color(btn_rotate, lv_color_hex(0x2196F3), 0);
  lv_obj_set_style_border_width(btn_rotate, 2, 0);
  lv_obj_set_style_border_color(btn_rotate, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(btn_rotate, 5, 0);
  lv_obj_add_event_cb(btn_rotate, rotate_screen_cb, LV_EVENT_CLICKED, NULL);
  lv_obj_t *rotate_label = lv_label_create(btn_rotate);
  lv_label_set_text(rotate_label, "ROTATE");
  lv_obj_set_style_text_font(rotate_label, btn_font, 0);
  lv_obj_set_style_text_color(rotate_label, lv_color_hex(0x000000), 0);
  lv_obj_center(rotate_label);
  
  // Bluetooth connection status label (between rotate and settings)
  top_bar_bt_status = lv_label_create(top_bar);
  const lv_font_t *bt_font = isPortrait ? &lv_font_montserrat_20 : &lv_font_montserrat_28;
  lv_obj_set_style_text_font(top_bar_bt_status, bt_font, 0);
  lv_obj_align_to(top_bar_bt_status, btn_rotate, LV_ALIGN_OUT_RIGHT_MID, bt_spacing, 0);
  update_top_bar_bt_status();
  
  // Settings button on the right (swapped from left)
  btn_settings = lv_btn_create(top_bar);
  lv_obj_set_size(btn_settings, btn_width_settings, btn_height);
  lv_obj_align(btn_settings, LV_ALIGN_RIGHT_MID, -5, 0);
  lv_obj_set_style_bg_color(btn_settings, lv_color_hex(0xFF9800), 0);
  lv_obj_set_style_border_width(btn_settings, 2, 0);
  lv_obj_set_style_border_color(btn_settings, lv_color_hex(0x000000), 0);
  lv_obj_set_style_radius(btn_settings, 5, 0);
  lv_obj_t *settings_label = lv_label_create(btn_settings);
  lv_label_set_text(settings_label, "SETTINGS");
  lv_obj_set_style_text_font(settings_label, btn_font, 0);
  lv_obj_set_style_text_color(settings_label, lv_color_hex(0x000000), 0);
  lv_obj_center(settings_label);
  lv_obj_add_event_cb(btn_settings, settings_btn_cb, LV_EVENT_CLICKED, NULL);
  
  // Ensure Bluetooth symbol is between rotate and settings
  lv_obj_align_to(top_bar_bt_status, btn_settings, LV_ALIGN_OUT_LEFT_MID, -bt_spacing, 0);
}

// Update Bluetooth status in top bar
static void update_top_bar_bt_status() {
  if (!top_bar_bt_status) {
    return; // UI element not created yet
  }
  
  #ifndef LV_SYMBOL_BLUETOOTH
  #define LV_SYMBOL_BLUETOOTH "\xef\x8a\x93"
  #endif
  
  // Use smaller font in portrait orientation (0 and 180 degrees)
  bool is_portrait = (currentRotation == 0 || currentRotation == 180);
  const lv_font_t *status_font = is_portrait ? &lv_font_montserrat_20 : &lv_font_montserrat_28;
  lv_obj_set_style_text_font(top_bar_bt_status, status_font, 0);
  
  // Only show Bluetooth symbol, no text
  lv_label_set_text(top_bar_bt_status, LV_SYMBOL_BLUETOOTH);
  
  // Set color based on connection status
  if (bluetoothConnected) {
    lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0x00FF00), 0); // Green when connected
  } else {
    lv_obj_set_style_text_color(top_bar_bt_status, lv_color_hex(0xFF0000), 0); // Red when disconnected
  }
}

// Test function to check red pixels - COMMENTED OUT due to API issues
// The Arduino_H7_Video class doesn't have drawPixel method as used here
// If you need to test display colors, you can create LVGL color objects instead
/*
void test_red_pixels() {
  Serial.println("Testing red pixels...");
  
  // This approach doesn't work with Arduino_H7_Video
  // Would need to use LVGL color objects and screen filling instead
  
  Serial.println("Color test function disabled - use LVGL UI for color testing");
}
*/
