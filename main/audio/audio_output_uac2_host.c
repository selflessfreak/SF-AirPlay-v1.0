/*
 * AirPlay PCM -> USB UAC2 Host -> AUDIOCULAR D07
 * D07 experimental output: 44.1 kHz, stereo, signed 16-bit PCM.
 */
#include "audio_output.h"
#include "audio_receiver.h"
#include "audio_resample.h"
#include "rtsp_server.h"
#include "led.h"
#include "d07_software_eq.h"

#include <stdatomic.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "usb/usb_host.h"
#include "usb/uac2_host.h"

#define TAG "airplay_uac2"
#define RATE 44100
#define FRAME_SAMPLES 352
#define MAX_FRAMES 4096
#define SILENCE_FRAMES 96

#define HOST_PRIORITY 21
#define DRIVER_PRIORITY 20
#define OUTPUT_PRIORITY AUDIO_PLAYBACK_TASK_PRIORITY

typedef struct {
    uint8_t addr;
    uint8_t iface;
} connection_t;

static QueueHandle_t s_queue;
static _Atomic bool s_disconnected;
static _Atomic bool s_playing;
static _Atomic bool s_flush;
static _Atomic int s_source_rate = 44100;
static esp_err_t s_host_result = ESP_FAIL;

static void apply_volume(int16_t *pcm, size_t frames)
{
    static int32_t current = -1;
    int32_t target = airplay_get_volume_q15();

    if (current < 0) current = target;

    for (size_t i = 0; i < frames; i++) {
        if (current != target) {
            int32_t step = (target - current) / 256;
            if (step == 0)
                step = target > current ? 1 : -1;
            current += step;
        }

        for (int ch = 0; ch < 2; ch++) {
            size_t pos = i * 2 + ch;
            pcm[pos] = (int16_t)(
                ((int32_t)pcm[pos] * current) >> 15
            );
        }
    }
}

static void device_cb(uac2_host_device_handle_t dev,
                      uac2_host_device_event_t event,
                      void *arg)
{
    (void)dev;
    (void)arg;

    if (event == UAC2_HOST_DEVICE_EVENT_DISCONNECTED)
        atomic_store(&s_disconnected, true);
}

static void driver_cb(uint8_t addr, uint8_t iface,
                      uac2_host_driver_event_t event,
                      void *arg)
{
    (void)arg;

    if (event == UAC2_HOST_DRIVER_EVENT_TX_CONNECTED) {
        connection_t conn = {
            .addr = addr,
            .iface = iface
        };
        xQueueOverwrite(s_queue, &conn);
    }
}

static void usb_host_task(void *arg)
{
    TaskHandle_t caller = (TaskHandle_t)arg;

    usb_host_config_t cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };

    s_host_result = usb_host_install(&cfg);
    xTaskNotifyGive(caller);

    if (s_host_result != ESP_OK) {
        ESP_LOGE(TAG, "USB host install failed");
        vTaskDelete(NULL);
        return;
    }

    while (true) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);

        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS)
            usb_host_device_free_all();
    }
}

static void playback_worker(void *arg)
{
    (void)arg;

    int16_t *pcm = malloc((FRAME_SAMPLES + 1) * 4);
    int16_t *resampled = malloc(MAX_FRAMES * 4);
    uint8_t *usb_pcm = malloc(MAX_FRAMES * 4);

    if (!pcm || !resampled || !usb_pcm) {
        ESP_LOGE(TAG, "Audio buffer allocation failed");
        free(pcm);
        free(resampled);
        free(usb_pcm);
        vTaskDelete(NULL);
        return;
    }

    connection_t conn;

    while (true) {
        xQueueReceive(s_queue, &conn, portMAX_DELAY);

        atomic_store(&s_disconnected, false);

        uac2_host_device_handle_t dev = NULL;

        uac2_host_device_config_t cfg = {
            .addr = conn.addr,
            .iface_num = conn.iface,
            .buffer_size = 0,
            .buffer_threshold = 0,
            .callback = device_cb,
            .callback_arg = NULL,
        };

        esp_err_t err = uac2_host_device_open(&cfg, &dev);

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Open failed: %s",
                     esp_err_to_name(err));
            continue;
        }

        uac2_host_stream_config_t stream = {
            .channels = 2,
            .bit_resolution = 16,
            .sample_freq = RATE,
            .flags = 0,
        };

        err = uac2_host_device_start(dev, &stream);

        if (err == ESP_OK) {
            uac2_host_stream_info_t info = {0};
            err = uac2_host_device_get_stream_info(dev, &info);

            if (err == ESP_OK && info.sub_slot_size != 2)
                err = ESP_ERR_NOT_SUPPORTED;
        }

        if (err == ESP_OK) {
            ESP_LOGI(TAG,
                "D07 streaming: 44100 Hz / 16-bit / stereo");

            int configured_rate = 44100;
            audio_resample_init(44100, RATE, 2);

            while (!atomic_load(&s_disconnected)) {
                int requested_rate = atomic_load(&s_source_rate);

                if (requested_rate != configured_rate &&
                    requested_rate > 0) {
                    configured_rate = requested_rate;
                    audio_resample_init(configured_rate, RATE, 2);
                }

                if (atomic_exchange(&s_flush, false))
                    audio_resample_reset();

                size_t frames = 0;
                int16_t *out = pcm;

                if (atomic_load(&s_playing)) {
                    frames = audio_receiver_read(
                        pcm, FRAME_SAMPLES + 1);

                    if (frames > 0 && audio_resample_is_active()) {
                        frames = audio_resample_process(
                            pcm, frames, resampled, MAX_FRAMES);
                        out = resampled;
                    }
                }

                if (frames == 0) {
                    frames = SILENCE_FRAMES;
                    memset(pcm, 0, frames * 4);
                    out = pcm;
                } else {
                    apply_volume(out, frames);
                }

                d07_eq_process(out, frames);
                led_audio_feed(out, frames);

                memcpy(usb_pcm, out, frames * 4);

                err = uac2_host_device_write(
                    dev, usb_pcm, frames * 4, 1000);

                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "USB write: %s",
                             esp_err_to_name(err));
                    break;
                }

                /* USB output queue latency diagnostics. */
                static uint32_t last_latency_log_ms = 0;
                uint32_t now_ms = esp_log_timestamp();

                if (now_ms - last_latency_log_ms >= 5000) {
                    last_latency_log_ms = now_ms;

                    uac2_host_buffer_status_t status = {0};

                    esp_err_t status_err =
                        uac2_host_device_get_buffer_status(
                            dev, &status);

                    if (status_err == ESP_OK) {
                        uint64_t total_us =
                            (uint64_t)status.ring_us +
                            status.in_flight_us;

                        ESP_LOGI(TAG,
                            "USB queue: ring=%lu/%lu bytes, "
                            "ring=%lu us, flight=%lu us, "
                            "total=%llu us",
                            (unsigned long)status.ring_bytes,
                            (unsigned long)status.ring_capacity,
                            (unsigned long)status.ring_us,
                            (unsigned long)status.in_flight_us,
                            (unsigned long long)total_us);
                    }
                }

                taskYIELD();
            }
        } else {
            ESP_LOGE(TAG, "D07 stream failed: %s",
                     esp_err_to_name(err));
        }

        /* Retry closure until outstanding USB transfers drain. */
        while (dev) {
            err = uac2_host_device_close(dev);
            if (err == ESP_OK || err == ESP_ERR_INVALID_ARG)
                break;
            vTaskDelay(pdMS_TO_TICKS(500));
        }

        ESP_LOGI(TAG, "D07 disconnected or stream stopped");
    }
}

esp_err_t audio_output_init(void)
{
    d07_eq_init();
    s_queue = xQueueCreate(1, sizeof(connection_t));

    if (!s_queue)
        return ESP_ERR_NO_MEM;

    if (xTaskCreatePinnedToCore(
            usb_host_task, "usb_host", 4096,
            xTaskGetCurrentTaskHandle(),
            HOST_PRIORITY, NULL, 0) != pdPASS)
        return ESP_ERR_NO_MEM;

    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2000)) == 0)
        return ESP_ERR_TIMEOUT;

    if (s_host_result != ESP_OK)
        return s_host_result;

    if (xTaskCreatePinnedToCore(
            playback_worker, "uac2_pcm", 8192,
            NULL, OUTPUT_PRIORITY,
            NULL, 1) != pdPASS)
        return ESP_ERR_NO_MEM;

    uac2_host_driver_config_t cfg = {
        .create_background_task = true,
        .task_priority = DRIVER_PRIORITY,
        .stack_size = 6144,
        .core_id = 0,
        .callback = driver_cb,
        .callback_arg = NULL,
    };

    esp_err_t err = uac2_host_install(&cfg);

    if (err == ESP_OK)
        ESP_LOGI(TAG, "AirPlay UAC2 host initialized");

    return err;
}

void audio_output_start(void)
{
    atomic_store(&s_playing, true);
}

void audio_output_stop(void)
{
    atomic_store(&s_playing, false);
    atomic_store(&s_flush, true);
}

void audio_output_flush(void)
{
    atomic_store(&s_flush, true);
}

void audio_output_set_source_rate(int rate)
{
    if (rate > 0)
        atomic_store(&s_source_rate, rate);
}

void audio_output_set_sample_rate(uint32_t rate)
{
    /* D07 remains fixed at 44.1 kHz in this branch. */
    if (rate != RATE)
        ESP_LOGW(TAG, "USB output fixed at 44100 Hz");
}

esp_err_t audio_output_write(const void *data,
                             size_t bytes,
                             TickType_t wait)
{
    /* Direct A2DP writes are not supported in this first backend. */
    (void)data;
    (void)bytes;
    (void)wait;
    return ESP_ERR_NOT_SUPPORTED;
}

uint32_t audio_output_get_hardware_latency_us(void)
{
    /* Initial estimate; measure and tune during AirPlay testing. */
    /*
     * Measured USB queue: approximately 97-100 ms ring
     * plus 9 ms submitted transfers.
     *
     * Baseline estimate until synchronized live reporting
     * is implemented.
     */
    return 107000;
}
