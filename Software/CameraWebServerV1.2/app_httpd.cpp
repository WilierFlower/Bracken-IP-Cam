/*
 * ESP32 Camera Web Server - HTTP Handlers
 * 
 * API Endpoints:
 *   GET /              - Web UI (HTML page)
 *   GET /stream        - MJPEG stream (for browsers, VLC, etc.)
 *   GET /snapshot      - Single JPEG frame (for Duet Web Control)
 *   GET /capture       - Single JPEG frame (alias for /snapshot)
 *   GET /status        - JSON status (resolution, quality, etc.)
 *   GET /control       - Change settings (?var=<name>&val=<value>)
 *   GET /health        - Health check endpoint
 * 
 * DUET WEB CONTROL (DWC) CONFIGURATION:
 *   - DWC Webcam URL: http://<cam-ip>:81/snapshot
 *   - /snapshot is on port 81 (separate from port 80 UI/stream)
 *   - This prevents interference between DWC and web UI streams
 *   - Includes retry logic and proper cache headers for reliability
 * 
 * Features:
 *   - Separate servers: port 80 (UI/stream), port 81 (/snapshot for DWC)
 *   - Camera warm-up after init ensures reliable first captures
 *   - Spec-compliant MJPEG stream format
 *   - 30 FPS cap to prevent overheating
 *   - Minimal logging to reduce CPU usage
 */

#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "board_config.h"

#include <esp32-hal-psram.h>
#include <WiFi.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// External flag to track server status
extern bool server_started;

// HTTP server handles: port 80 for UI/stream, port 81 for /snapshot (DWC)
static httpd_handle_t camera_httpd = nullptr;
static httpd_handle_t snapshot_httpd = nullptr;

// Stream pacing: default 33ms = ~30 FPS (capped to prevent overheating)
static uint16_t stream_delay_ms = 33; // 30 FPS cap

// No mutex needed: /snapshot on port 81, /stream on port 80 (separate servers)

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

// MJPEG stream boundary (spec-compliant)
#define PART_BOUNDARY "frame"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace; boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

// Camera warm-up function: capture and discard a few frames after init
// This ensures the camera is ready for reliable captures
void warmupCamera() {
  for (int i = 0; i < 3; i++) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      esp_camera_fb_return(fb);
      vTaskDelay(100 / portTICK_PERIOD_MS); // Small delay between frames
    }
  }
}

static size_t jpg_encode_stream(void *arg, size_t index, const void *data, size_t len) {
  jpg_chunking_t *j = (jpg_chunking_t *)arg;
  if (!index) {
    j->len = 0;
  }
  if (httpd_resp_send_chunk(j->req, (const char *)data, len) != ESP_OK) {
    return 0;
  }
  j->len += len;
  return len;
}

// /capture and /snapshot: single JPEG frame
// 
// IMPORTANT FOR DUET WEB CONTROL (DWC):
// - DWC should use: http://<cam-ip>:81/snapshot (port 81, separate from UI)
// - This endpoint is on a separate server to prevent interference
// - Includes retry logic and proper cache headers for reliability
static esp_err_t capture_handler(httpd_req_t *req) {
  const int MAX_RETRIES = 3;
  const int RETRY_DELAY_MS = 50;
  
  camera_fb_t *fb = nullptr;
  esp_err_t res = ESP_FAIL;
  
  // Retry loop for reliability (handles cold-start and transient failures)
  for (int retry = 0; retry < MAX_RETRIES; retry++) {
    fb = esp_camera_fb_get();
    if (fb) {
      break; // Success, exit retry loop
    }
    if (retry < MAX_RETRIES - 1) {
      vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
  }
  
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  // Set response headers
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  
  // Critical: Prevent caching of stale/bad frames
  // This ensures DWC always gets fresh frames
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  httpd_resp_set_hdr(req, "Pragma", "no-cache");
  httpd_resp_set_hdr(req, "Expires", "0");

  // Encode and send frame
  if (fb->format == PIXFORMAT_JPEG) {
    res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  } else {
    jpg_chunking_t jchunk = {req, 0};
    res = frame2jpg_cb(fb, 80, jpg_encode_stream, &jchunk) ? ESP_OK : ESP_FAIL;
    httpd_resp_send_chunk(req, NULL, 0);
  }

  esp_camera_fb_return(fb);
  return res;
}

// /stream: MJPEG stream for browser viewing, VLC, etc. (port 80)
// 
// NOTE: Duet Web Control (DWC) should NOT use /stream (no MJPEG support).
// DWC should use http://<cam-ip>:81/snapshot instead.
// 
// This endpoint is spec-compliant MJPEG with proper multipart/x-mixed-replace format.
// Capped at 30 FPS (33ms delay) to prevent overheating.
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[128];
  static uint32_t error_count = 0;

  // Set spec-compliant MJPEG headers
  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate");

  while (true) {
    fb = esp_camera_fb_get();
    
    if (!fb) {
      error_count++;
      // If too many consecutive errors, break the stream
      if (error_count > 10) {
        res = ESP_FAIL;
        break;
      }
      vTaskDelay(50 / portTICK_PERIOD_MS);
      continue;
    }
    
    error_count = 0;
    
    _timestamp.tv_sec = fb->timestamp.tv_sec;
    _timestamp.tv_usec = fb->timestamp.tv_usec;
    
    if (fb->format != PIXFORMAT_JPEG) {
      // Convert non-JPEG format to JPEG
      bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
      esp_camera_fb_return(fb);
      fb = NULL;
      
      if (!jpeg_converted || !_jpg_buf || _jpg_buf_len == 0) {
        if (_jpg_buf) {
          free(_jpg_buf);
          _jpg_buf = NULL;
        }
        res = ESP_FAIL;
        break;
      }
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }

    // Send boundary (spec-compliant: --frame\r\n)
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    // Send frame headers with accurate Content-Length
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, 
                             _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      if (hlen < sizeof(part_buf)) {
        res = httpd_resp_send_chunk(req, part_buf, hlen);
      } else {
        res = ESP_FAIL;
      }
    }
    
    // Send image data
    if (res == ESP_OK && _jpg_buf && _jpg_buf_len > 0) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    // Free frame buffer
    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL; // Don't free, it's part of fb
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    // Check for send errors (client disconnect, network issue, etc.)
    if (res != ESP_OK) {
      break; // Exit gracefully
    }

    // Frame rate limiting: always enforce 30 FPS cap (33ms delay)
    vTaskDelay(stream_delay_ms / portTICK_PERIOD_MS);
  }

  // Cleanup: ensure all buffers are freed
  if (fb) {
    esp_camera_fb_return(fb);
    fb = NULL;
  }
  if (_jpg_buf && !fb) {
    free(_jpg_buf);
    _jpg_buf = NULL;
  }

  return res;
}

// /: Serve HTML UI
// Note: The UI may embed a /stream request that keeps the browser in a loading state.
// This is expected behavior for MJPEG streams. The page should load fully first,
// then JavaScript starts the stream without blocking the initial page load.
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  // Send the HTML page (this completes immediately, stream starts via JS)
  return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
}

// Helper: Parse framesize value
static framesize_t framesize_from_value(const char *value) {
  if (!value) {
    return FRAMESIZE_SVGA;
  }

  if (strcasecmp(value, "svga") == 0) {
    return FRAMESIZE_SVGA;
  }
  if (strcasecmp(value, "fhd") == 0 || strcasecmp(value, "1080p") == 0) {
    return FRAMESIZE_FHD;
  }

  int idx = atoi(value);
  if (idx >= 0 && idx < FRAMESIZE_INVALID) {
    return (framesize_t)idx;
  }
  return FRAMESIZE_SVGA;
}

// Helper: Get framesize name
static const char *framesize_name(framesize_t size) {
  switch (size) {
    case FRAMESIZE_FHD:
      return "FHD";
    case FRAMESIZE_SVGA:
      return "SVGA";
    default:
      return "CUSTOM";
  }
}

// /control: Camera controls (framesize, quality, stream_delay, vflip, hmirror)
// Used by the web UI to change settings. Format: /control?var=<name>&val=<value>
// Changes take effect immediately and are reflected in /status.
static esp_err_t cmd_handler(httpd_req_t *req) {
  char variable[32];
  char value[32];

  if (httpd_req_get_url_query_len(req) == 0) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  char *buf = (char *)malloc(httpd_req_get_url_query_len(req) + 1);
  if (!buf) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  esp_err_t res = ESP_OK;
  if (httpd_req_get_url_query_str(req, buf, httpd_req_get_url_query_len(req) + 1) != ESP_OK ||
      httpd_query_key_value(buf, "var", variable, sizeof(variable)) != ESP_OK ||
      httpd_query_key_value(buf, "val", value, sizeof(value)) != ESP_OK) {
    res = ESP_FAIL;
  }
  free(buf);

  if (res != ESP_OK) {
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }

  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  int val = atoi(value);
  
  if (!strcmp(variable, "framesize")) {
    framesize_t target = framesize_from_value(value);
    if (!psramFound() && target > FRAMESIZE_SVGA) {
      log_w("PSRAM not available, limiting to SVGA");
      target = FRAMESIZE_SVGA;
    }
    res = s->set_framesize(s, target);
    
  } else if (!strcmp(variable, "quality")) {
    if (val < 5) val = 5;
    if (val > 63) val = 63;
    res = s->set_quality(s, val);
    
  } else if (!strcmp(variable, "stream_delay")) {
    // Cap at 30 FPS minimum (33ms delay) to prevent overheating
    if (val < 33) val = 33;
    if (val > 500) val = 500;
    stream_delay_ms = (uint16_t)val;
    
  } else if (!strcmp(variable, "vflip")) {
    if (val < 0) val = 0;
    if (val > 1) val = 1;
    res = s->set_vflip(s, val);
    
  } else if (!strcmp(variable, "hmirror")) {
    if (val < 0) val = 0;
    if (val > 1) val = 1;
    res = s->set_hmirror(s, val);
    
  } else {
    res = ESP_FAIL;
  }

  if (res != ESP_OK) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// /status: JSON status with resolution, quality, fps, orientation
// Used by the web UI to sync settings. Returns current camera configuration.
static esp_err_t status_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    return httpd_resp_send_500(req);
  }

  char json_response[256];
  int len = snprintf(json_response, sizeof(json_response),
                     "{\"framesize\":%u,\"framesize_name\":\"%s\",\"quality\":%u,"
                     "\"stream_delay\":%u,\"vflip\":%u,\"hmirror\":%u,"
                     "\"wifi_rssi\":%d}",
                     s->status.framesize, 
                     framesize_name((framesize_t)s->status.framesize),
                     s->status.quality, 
                     stream_delay_ms,
                     s->status.vflip,
                     s->status.hmirror,
                     WiFi.RSSI());

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, len);
}

// /health: Simple health check endpoint
static esp_err_t health_handler(httpd_req_t *req) {
  bool camera_ok = (esp_camera_sensor_get() != nullptr);
  bool wifi_ok = (WiFi.status() == WL_CONNECTED);
  bool server_ok = (camera_httpd != nullptr);
  
  const char *status = (camera_ok && wifi_ok && server_ok) ? "OK" : "ERROR";
  
  char response[128];
  int len = snprintf(response, sizeof(response),
                     "{\"status\":\"%s\",\"camera\":%s,\"wifi\":%s,\"server\":%s}",
                     status,
                     camera_ok ? "true" : "false",
                     wifi_ok ? "true" : "false",
                     server_ok ? "true" : "false");
  
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, response, len);
}

// Start HTTP servers: port 80 for UI/stream, port 81 for /snapshot (DWC)
void startCameraServer() {
  // Prevent multiple starts
  if (camera_httpd != nullptr) {
    server_started = true;
    return;
  }

  // Start main server on port 80 (UI, /stream, /status, /control)
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;
  config.max_uri_handlers = 8;
  config.lru_purge_enable = true;

  esp_err_t err = httpd_start(&camera_httpd, &config);
  if (err != ESP_OK) {
    server_started = false;
    return;
  }

  // Register all URI handlers
  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL
  };

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL
  };

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL
  };

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL
  };

  httpd_uri_t health_uri = {
    .uri = "/health",
    .method = HTTP_GET,
    .handler = health_handler,
    .user_ctx = NULL
  };

  // Register handlers on port 80 (UI, stream, control)
  httpd_register_uri_handler(camera_httpd, &index_uri);
  httpd_register_uri_handler(camera_httpd, &status_uri);
  httpd_register_uri_handler(camera_httpd, &cmd_uri);
  httpd_register_uri_handler(camera_httpd, &capture_uri);
  httpd_register_uri_handler(camera_httpd, &stream_uri);
  httpd_register_uri_handler(camera_httpd, &health_uri);

  // Start separate server on port 81 for /snapshot (DWC)
  httpd_config_t snapshot_config = HTTPD_DEFAULT_CONFIG();
  snapshot_config.server_port = 81;
  snapshot_config.max_uri_handlers = 2;
  snapshot_config.lru_purge_enable = true;

  err = httpd_start(&snapshot_httpd, &snapshot_config);
  if (err != ESP_OK) {
    server_started = false;
    return;
  }

  httpd_uri_t snapshot_uri = {
    .uri = "/snapshot",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL
  };

  httpd_register_uri_handler(snapshot_httpd, &snapshot_uri);
  
  server_started = true;
}

// Legacy function (kept for compatibility, but LED is now handled in main sketch)
void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#endif
}
