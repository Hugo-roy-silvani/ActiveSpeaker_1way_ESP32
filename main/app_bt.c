#include "app_bt.h"
#include "app_dsp.h"

#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_a2dp_api.h"
#include "esp_avrc_api.h"
#include "esp_gap_bt_api.h"

static const char *TAG = "APP_BT_AUDIO";

static RingbufHandle_t s_bt_ringbuf = NULL;

static float s_bt_remote_volume = 1.0f;  // volume du téléphone normalisé [0..1]

float app_bt_get_remote_volume_level(void)
{
    return s_bt_remote_volume;
}


/* -------------------------------------------------------------------------- */
/*                Callback A2DP: données PCM décodées                          */
/* -------------------------------------------------------------------------- */

/* 
 * Cette callback est appelée par l'A2DP sink quand des données PCM sont prêtes.
 * data : buffer de PCM interleavé (souvent 16 bits stéréo : L, R, L, R, ...)
 * len  : taille en octets
 */

static void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len)
{
    if (!s_bt_ringbuf || !data || len == 0) {
        return;
    }

    const int16_t *in = (const int16_t *)data;
    size_t num_samples_total = len / sizeof(int16_t);   // L,R,L,R,...
    size_t num_frames        = num_samples_total / 2;   // paires L/R

    const size_t MAX_FRAMES = 256;
    int16_t mono_buf[MAX_FRAMES];

    static uint32_t drop_count = 0;

    size_t frames_remaining = num_frames;
    const int16_t *p = in;

    while (frames_remaining > 0) {
        size_t frames_now = (frames_remaining > MAX_FRAMES)
                                ? MAX_FRAMES
                                : frames_remaining;

        for (size_t i = 0; i < frames_now; i++) {
            int16_t left  = p[2 * i + 0];
            int16_t right = p[2 * i + 1];

            int32_t m = (int32_t)left + (int32_t)right;
            m /= 2;
            if (m > 32767)  m = 32767;
            if (m < -32768) m = -32768;
            mono_buf[i] = (int16_t)m;
        }

        size_t bytes_to_send = frames_now * sizeof(int16_t);

        BaseType_t ok = xRingbufferSend(
            s_bt_ringbuf,
            mono_buf,
            bytes_to_send,
            0
        );

        if (ok != pdTRUE) {
            drop_count++;
            if ((drop_count % 100) == 0) {
                ESP_LOGW(TAG, "BT ringbuffer full, dropped %u chunks", (unsigned)drop_count);
            }
        }

        p += frames_now * 2;
        frames_remaining -= frames_now;
    }
}


/* -------------------------------------------------------------------------- */
/*           Callback A2DP : évènements de contrôle (connexion, etc.)         */
/* -------------------------------------------------------------------------- */

static void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param)
{
    // Ici tu peux logguer les évènements (connecté, déconnecté, etc.)
    switch (event) {
    case ESP_A2D_CONNECTION_STATE_EVT:
        ESP_LOGI(TAG, "A2DP conn state: %d", param->conn_stat.state);
        break;

    case ESP_A2D_AUDIO_STATE_EVT:
        ESP_LOGI(TAG, "A2DP audio state: %d", param->audio_stat.state);
        break;

    case ESP_A2D_AUDIO_CFG_EVT: {
        uint32_t sample_rate = 0;

        if (param->audio_cfg.mcc.type == ESP_A2D_MCT_SBC) {
            // Dans ta version, 'sbc' est un tableau de uint8_t
            // On lit le premier octet, qui contient les flags de fréquence
            const uint8_t sf = param->audio_cfg.mcc.cie.sbc[0];

            if (sf & ESP_A2D_SBC_CIE_SF_16K) {
                sample_rate = 16000;
            } else if (sf & ESP_A2D_SBC_CIE_SF_32K) {
                sample_rate = 32000;
            } else if (sf & ESP_A2D_SBC_CIE_SF_44K) {
                sample_rate = 44100;
            } else if (sf & ESP_A2D_SBC_CIE_SF_48K) {
                sample_rate = 48000;
            }
        }

        ESP_LOGI(TAG, "A2DP audio cfg: sample_rate=%lu", (unsigned long)sample_rate);
        break;
    }

    default:
        break;
    }
}

static void bt_app_avrc_ct_cb(esp_avrc_ct_cb_event_t event, esp_avrc_ct_cb_param_t *param)
{
    switch (event) {
    case ESP_AVRC_CT_CHANGE_NOTIFY_EVT:
        if (param->change_ntf.event_id == ESP_AVRC_RN_VOLUME_CHANGE) {
            uint8_t vol = param->change_ntf.event_parameter.volume; 
            float v = (float)vol / 127.0f;
            if (v < 0.0f) v = 0.0f;
            if (v > 1.0f) v = 1.0f;

            s_bt_remote_volume = v;

            ESP_LOGI(TAG, "Remote BT volume changed: raw=%u -> norm=%.2f", vol, v);

            // Loudness dynamique basé sur le volume du téléphone
            app_dsp_set_loudness_level(v);

            // IMPORTANT : se réenregistrer pour la prochaine notif
            esp_avrc_ct_send_register_notification_cmd(1,
                ESP_AVRC_RN_VOLUME_CHANGE, 0);
        }
        break;

    case ESP_AVRC_CT_CONNECTION_STATE_EVT:
        if (param->conn_stat.connected) {
            ESP_LOGI(TAG, "AVRCP connected, registering for volume change");
            // On demande au téléphone de nous notifier les changements de volume
            esp_avrc_ct_send_register_notification_cmd(1,
                ESP_AVRC_RN_VOLUME_CHANGE, 0);
        }
        break;

    default:
        break;
    }
}


/* -------------------------------------------------------------------------- */
/*                         Initialisation Bluetooth                           */
/* -------------------------------------------------------------------------- */

void app_bt_audio_init(RingbufHandle_t ringbuf)
{
    s_bt_ringbuf = ringbuf;

    esp_err_t err;

    ESP_LOGI(TAG, "Init BT audio (A2DP sink)");

    // 1) Init controller BT
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    err = esp_bt_controller_init(&bt_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bt_controller_enable failed: %s", esp_err_to_name(err));
        return;
    }

    // 2) Init Bluedroid
    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_bluedroid_enable failed: %s", esp_err_to_name(err));
        return;
    }

    // 2bis) Init AVRCP Controller pour récupérer le volume du téléphone
    err = esp_avrc_ct_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_avrc_ct_init failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_avrc_ct_register_callback(bt_app_avrc_ct_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_avrc_ct_register_callback failed: %s", esp_err_to_name(err));
        return;
    }


    // 3) Configurer le nom du device
    esp_bt_dev_set_device_name("Oeuf");

    // 4) Init A2DP sink
    err = esp_a2d_register_callback(bt_app_a2d_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_register_callback failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_sink_register_data_callback failed: %s", esp_err_to_name(err));
        return;
    }

    err = esp_a2d_sink_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_a2d_sink_init failed: %s", esp_err_to_name(err));
        return;
    }

    // 5) Mettre en mode "découvrable" / "connectable"
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

    ESP_LOGI(TAG, "A2DP sink initialized, device ready for pairing");
}
