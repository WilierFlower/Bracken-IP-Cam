#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "camera_index.h"
#include "board_config.h"

#include <esp32-hal-psram.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>

#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#endif

// HTTP handles
static httpd_handle_t stream_httpd = nullptr;
static httpd_handle_t camera_httpd = nullptr;

// User-tunable stream pacing (set via /control?var=stream_delay&val=###)
static uint16_t stream_delay_ms = 0;

typedef struct {
  httpd_req_t *req;
  size_t len;
} jpg_chunking_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *_STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %d.%06d\r\n\r\n";

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
static esp_err_t capture_handler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    log_e("Camera capture failed");
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }

  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  esp_err_t res;
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

// /stream: MJPEG stream (Duet Web Control webcam URL)
static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  struct timeval _timestamp;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t *_jpg_buf = NULL;
  char part_buf[128];

  static int64_t last_frame = 0;
  static uint32_t frame_counter = 0;

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if (res != ESP_OK) {
    return res;
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      log_e("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _timestamp.tv_sec = fb->timestamp.tv_sec;
      _timestamp.tv_usec = fb->timestamp.tv_usec;
      if (fb->format != PIXFORMAT_JPEG) {
        bool jpeg_converted = frame2jpg(fb, 80, &_jpg_buf, &_jpg_buf_len);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          res = ESP_FAIL;
        }
      } else {
        _jpg_buf_len = fb->len;
        _jpg_buf = fb->buf;
      }
    }

    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    if (res == ESP_OK) {
      size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART, _jpg_buf_len, _timestamp.tv_sec, _timestamp.tv_usec);
      res = httpd_resp_send_chunk(req, part_buf, hlen);
    }
    if (res == ESP_OK) {
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    } else if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }

    if (res != ESP_OK) {
      break;
    }

    int64_t now = esp_timer_get_time();
    if (last_frame) {
      int64_t frame_time = (now - last_frame) / 1000;
      if (++frame_counter % 30 == 0 && frame_time > 0) {
        log_i("Stream frame %ums (%.1ffps)", (uint32_t)frame_time, 1000.0 / frame_time);
      }
    }
    last_frame = now;

    if (stream_delay_ms > 0) {
      vTaskDelay(stream_delay_ms / portTICK_PERIOD_MS);
    }
  }

  return res;
}

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
  return httpd_resp_send(req, (const char *)index_ov3660_html_gz, index_ov3660_html_gz_len);
}

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

// /control: lightweight camera controls (framesize, quality, stream pacing)
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
      target = FRAMESIZE_SVGA;
    }
    res = s->set_framesize(s, target);
  } else if (!strcmp(variable, "quality")) {
    if (val < 5) val = 5;
    if (val > 63) val = 63;
    res = s->set_quality(s, val);
  } else if (!strcmp(variable, "stream_delay")) {
    if (val < 0) val = 0;
    if (val > 500) val = 500;
    stream_delay_ms = (uint16_t)val;
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

// /status: minimal JSON status for UI
static esp_err_t status_handler(httpd_req_t *req) {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) {
    return httpd_resp_send_500(req);
  }

  char json_response[128];
  int len = snprintf(json_response, sizeof(json_response),
                     "{\"framesize\":%u,\"framesize_name\":\"%s\",\"quality\":%u,\"stream_delay\":%u}",
                     s->status.framesize, framesize_name((framesize_t)s->status.framesize),
                     s->status.quality, stream_delay_ms);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, json_response, len);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 8;

  httpd_uri_t index_uri = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = index_handler,
    .user_ctx = NULL};

  httpd_uri_t status_uri = {
    .uri = "/status",
    .method = HTTP_GET,
    .handler = status_handler,
    .user_ctx = NULL};

  httpd_uri_t cmd_uri = {
    .uri = "/control",
    .method = HTTP_GET,
    .handler = cmd_handler,
    .user_ctx = NULL};

  httpd_uri_t capture_uri = {
    .uri = "/capture",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL};

  httpd_uri_t snapshot_uri = {
    .uri = "/snapshot",
    .method = HTTP_GET,
    .handler = capture_handler,
    .user_ctx = NULL};

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = stream_handler,
    .user_ctx = NULL};

  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &status_uri);
    httpd_register_uri_handler(camera_httpd, &capture_uri);
    httpd_register_uri_handler(camera_httpd, &snapshot_uri);
  }

  config.server_port += 1;
  config.ctrl_port += 1;
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setupLedFlash() {
#if defined(LED_GPIO_NUM)
  ledcAttach(LED_GPIO_NUM, 5000, 8);
#else
  log_i("LED flash is disabled -> LED_GPIO_NUM undefined");
#endif
}
