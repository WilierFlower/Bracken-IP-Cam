#include "esp_camera.h"
#include <WiFi.h>

// ===========================
// Select camera model in board_config.h
// ===========================
#include "board_config.h"

// ===========================
// Enter your WiFi credentials
// ===========================
const char *ssid = "Dinosaurs";
const char *password = "JurassicJulia00!";

// ===========================
// User LED Configuration
// GPIO21, active LOW (0 = ON, 1 = OFF)
// ===========================
#define USER_LED_GPIO 21
#define LED_ON  LOW
#define LED_OFF HIGH

// LED state machine
enum LedMode {
  LED_MODE_CONNECTING,  // Slow blink (~500ms)
  LED_MODE_READY,       // Solid ON
  LED_MODE_ERROR        // Fast blink (~150-200ms)
};

LedMode led_mode = LED_MODE_CONNECTING;
bool led_state = false;
unsigned long last_led_toggle_ms = 0;

// System state tracking
bool camera_initialized = false;
bool wifi_connected = false;
bool server_started = false; // Set by startCameraServer() in app_httpd.cpp
unsigned long wifi_connect_start_ms = 0;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 30000; // 30 seconds

void startCameraServer();
void setLedMode(LedMode mode);
void updateLed();
bool initCamera();
bool connectWiFi();
void checkWiFiStatus();
void warmupCamera(); // Camera warm-up function (declared in app_httpd.cpp)

// ===========================
// LED Status Management
// ===========================
void setLedMode(LedMode mode) {
  led_mode = mode;
  last_led_toggle_ms = millis();
  
  // Immediately set LED state based on mode
  if (mode == LED_MODE_READY) {
    digitalWrite(USER_LED_GPIO, LED_ON);
    led_state = true;
  } else {
    digitalWrite(USER_LED_GPIO, LED_OFF);
    led_state = false;
  }
}

void updateLed() {
  unsigned long now = millis();
  unsigned long interval_ms = 0;
  
  switch (led_mode) {
    case LED_MODE_CONNECTING:
      interval_ms = 500; // Slow blink
      break;
    case LED_MODE_READY:
      // Solid ON - no toggling needed
      return;
    case LED_MODE_ERROR:
      interval_ms = 175; // Fast blink
      break;
  }
  
  if (now - last_led_toggle_ms >= interval_ms) {
    led_state = !led_state;
    digitalWrite(USER_LED_GPIO, led_state ? LED_ON : LED_OFF);
    last_led_toggle_ms = now;
  }
}

// ===========================
// Camera Initialization
// ===========================
bool initCamera() {
  Serial.println("Initializing camera...");
  
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  // Default to SVGA (800x600) for smooth streaming
  config.frame_size = FRAMESIZE_SVGA;
  
  const bool hasPsram = psramFound();
  config.fb_location = hasPsram ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  config.fb_count = hasPsram ? 2 : 1;
  config.jpeg_quality = hasPsram ? 12 : 16;
  config.grab_mode = hasPsram ? CAMERA_GRAB_LATEST : CAMERA_GRAB_WHEN_EMPTY;
  
  if (!hasPsram) {
    Serial.println("WARNING: No PSRAM detected, using DRAM only");
    config.frame_size = FRAMESIZE_SVGA;
  } else {
    Serial.println("PSRAM detected, using PSRAM for frame buffers");
  }
  
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("ERROR: Camera init failed with error 0x%x\n", err);
    return false;
  }
  
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    Serial.println("ERROR: Failed to get camera sensor");
    return false;
  }
  
  applySensorDefaults(s);
  
  // Ensure SVGA is set (may have been overridden by defaults)
  s->set_framesize(s, FRAMESIZE_SVGA);
  s->set_quality(s, config.jpeg_quality);
  
  Serial.println("Camera initialized successfully");
  
  // Warm up the camera: capture and discard a few frames
  // This ensures reliable captures from the first /snapshot request (no "prime the UI" needed)
  warmupCamera();
  
  return true;
}

static void applySensorDefaults(sensor_t *s) {
  if (s == nullptr) {
    return;
  }
  
  // OV3660 specific defaults
  if (s->id.PID == OV3660_PID) {
    // Default orientation: normal (can be changed via /control)
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);
    s->set_brightness(s, 0);
    s->set_saturation(s, 0);
  }
  
  // Enable auto features for indoor use
  s->set_lenc(s, 1);           // Lens correction
  s->set_whitebal(s, 1);       // Auto white balance
  s->set_awb_gain(s, 1);       // AWB gain
  s->set_exposure_ctrl(s, 1);  // Auto exposure
  s->set_gain_ctrl(s, 1);      // Auto gain
  s->set_special_effect(s, 0); // No special effects
  s->set_colorbar(s, 0);       // No test pattern
}

// ===========================
// WiFi Connection (Non-blocking)
// ===========================
bool connectWiFi() {
  Serial.println("Starting WiFi connection...");
  WiFi.setSleep(false); // Disable WiFi sleep for stable streaming
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  wifi_connect_start_ms = millis();
  wifi_connected = false;
  
  return true; // Connection attempt started
}

void checkWiFiStatus() {
  if (wifi_connected) {
    // Check if WiFi is still connected
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WARNING: WiFi connection lost!");
      wifi_connected = false;
      server_started = false;
      setLedMode(LED_MODE_ERROR);
    }
    return;
  }
  
  // Check connection status
  wl_status_t status = WiFi.status();
  
  if (status == WL_CONNECTED) {
    if (!wifi_connected) {
      wifi_connected = true;
      IPAddress ip = WiFi.localIP();
      Serial.println("");
      Serial.println("WiFi connected!");
      Serial.printf("IP address: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
      Serial.printf("RSSI: %d dBm\n", WiFi.RSSI());
    }
  } else if (status == WL_CONNECT_FAILED || status == WL_NO_SSID_AVAIL) {
    // Connection failed, check timeout
    if (millis() - wifi_connect_start_ms > WIFI_CONNECT_TIMEOUT_MS) {
      Serial.println("ERROR: WiFi connection timeout");
      setLedMode(LED_MODE_ERROR);
    }
  } else {
    // Still connecting
    unsigned long elapsed = millis() - wifi_connect_start_ms;
    if (elapsed % 2000 < 100) { // Print every 2 seconds
      Serial.print(".");
    }
  }
}

// ===========================
// Setup
// ===========================
void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false);
  delay(100); // Allow serial to stabilize
  
  Serial.println();
  Serial.println("========================================");
  Serial.println("XIAO ESP32-S3 Sense Camera Web Server");
  Serial.println("Board: Seeed Studio XIAO ESP32-S3 Sense");
  Serial.println("Camera: OV3660");
  Serial.println("Use case: Duet 2 WiFi 3D printer monitor");
  Serial.println("========================================");
  
  // Initialize user LED (GPIO21, active LOW)
  pinMode(USER_LED_GPIO, OUTPUT);
  setLedMode(LED_MODE_CONNECTING);
  Serial.println("User LED initialized (GPIO21, active LOW)");
  
  // Initialize camera
  camera_initialized = initCamera();
  if (!camera_initialized) {
    Serial.println("FATAL: Camera initialization failed, entering error state");
    setLedMode(LED_MODE_ERROR);
    return; // Continue anyway, LED will show error
  }
  
  // Start WiFi connection (non-blocking)
  connectWiFi();
  
  // Note: HTTP server will be started in loop() once WiFi is connected
}

// ===========================
// Main Loop (Non-blocking)
// ===========================
void loop() {
  // Update LED status
  updateLed();
  
  // Check WiFi connection status
  checkWiFiStatus();
  
  // Start HTTP server once WiFi is connected
  if (wifi_connected && camera_initialized && !server_started) {
    Serial.println("Starting HTTP server...");
    startCameraServer();
    
    if (server_started) {
      Serial.println("HTTP server started successfully");
      Serial.print("Camera Ready! Use 'http://");
      Serial.print(WiFi.localIP());
      Serial.println("/' to connect");
      Serial.println("");
      Serial.println("=== DUET WEB CONTROL (DWC) CONFIGURATION ===");
      Serial.print("DWC Webcam URL: http://");
      Serial.print(WiFi.localIP());
      Serial.println(":81/snapshot");
      Serial.println("Note: /snapshot is on port 81 (separate from UI on port 80)");
      Serial.println("=============================================");
      setLedMode(LED_MODE_READY);
    } else {
      Serial.println("ERROR: Failed to start HTTP server");
      setLedMode(LED_MODE_ERROR);
    }
  }
  
  // Small delay to prevent watchdog issues
  delay(10);
}
