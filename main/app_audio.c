#include "app_audio.h"
#include "board_config.h"
#include "app_dsp.h"
#include "app_bt.h"

#include "esp_log.h"
#include "driver/i2s_std.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include <math.h>

/* === Log TAG === */
static const char *TAG = "APP_AUDIO";

/* === Paramètres audio === */
#define SINE_FREQ_HZ            500.0f
#define SINE_TABLE_SIZE         256

/* On définit la taille de nos blocs audio (frames) */
#define AUDIO_FRAME_SAMPLES     256                     // samples par bloc
#define AUDIO_FRAME_BYTES       (AUDIO_FRAME_SAMPLES * sizeof(int16_t))

/* Latence / profondeur du ring buffer :
 * 16 blocs de 256 samples 
 */
#define AUDIO_RINGBUF_FRAMES    32
#define AUDIO_RINGBUF_SIZE      (AUDIO_FRAME_BYTES * AUDIO_RINGBUF_FRAMES)

/* Volume de sortie global (0.0f à 1.0f) */
static float s_output_volume = 1.0f;  

static float s_pregain = 0.9f; // 80%, à ajuster

/* === Canal TX I2S === */
static i2s_chan_handle_t tx_chan = NULL;

/* === Ring buffer audio (producteur -> consommateur) === */
static RingbufHandle_t s_audio_ringbuf = NULL;

/* === Table de sinus (16 bits signé) === */
static int16_t sine_table[SINE_TABLE_SIZE];

void app_audio_set_volume(float vol)
{
    if (vol < 0.0f) vol = 0.0f;
    if (vol > 1.0f) vol = 1.0f;
    s_output_volume = vol;
}

/* -------------------------------------------------------------------------- */
/*                         Initialisation I2S TX (STD)                        */
/* -------------------------------------------------------------------------- */

static void i2s_init_tx(void)
{
    ESP_LOGI(TAG, "I2S TX Initialisation...");

    // 1) Création du canal TX
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT_TX,
                                                            I2S_ROLE_MASTER);
    chan_cfg.dma_frame_num = AUDIO_FRAME_SAMPLES;   // frames par buffer DMA
    chan_cfg.dma_desc_num  = 3;                     // nombre de descripteurs DMA

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_chan, NULL));   // TX only

    // 2) Configuration "STD" en sortie
    i2s_std_config_t cfg_tx = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = {
            .data_bit_width = I2S_DATA_BIT_WIDTH_16BIT,
            .slot_bit_width = I2S_SLOT_BIT_WIDTH_16BIT,
            .slot_mode      = I2S_SLOT_MODE_MONO,
            .slot_mask      = I2S_STD_SLOT_LEFT,          // on utilise le slot gauche
            .ws_width       = I2S_DATA_BIT_WIDTH_16BIT,
            .bit_shift      = true,                       // alignement I2S standard
            .msb_right      = false,
            .ws_pol         = false,
        },
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,                      // pas de MCLK pour l'instant
            .bclk = SPK_BCLK_GPIO,
            .ws   = SPK_LRCLK_GPIO,
            .dout = SPK_DOUT_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &cfg_tx));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}


/* -------------------------------------------------------------------------- */
/*           Tâche "output" : DSP + I2S (consommateur du ring buffer)         */
/* -------------------------------------------------------------------------- */

static void audio_output_task(void *arg)
{
    ESP_LOGI(TAG, "Audio output task started");

    while (1) {
        size_t item_size = 0;
        // On récupère un bloc depuis le ring buffer
        int16_t *data = (int16_t *) xRingbufferReceive(
            s_audio_ringbuf,
            &item_size,
            portMAX_DELAY    // on bloque tant qu'il n'y a pas de données
        );

        if (data == NULL) {
            ESP_LOGW(TAG, "audio_output_task: xRingbufferReceive returned NULL");
            continue;
        }

        size_t num_samples = item_size / sizeof(int16_t);

        float pg = s_pregain;
        if (pg < 0.999f || pg > 1.001f) { // évite boucle inutile si ~1
            for (size_t i = 0; i < num_samples; i++) {
                float s = (float)data[i] * pg;
                if (s > 32767.0f)  s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                data[i] = (int16_t)s;
            }
        }

        // 1) Passage par le DSP (in-place)
        app_dsp_process_block_int16(data, num_samples);

        // 2) Application du volume de sortie
        float vol = s_output_volume;   // copie locale (évite les races)
        if (vol < 0.0f) vol = 0.0f;
        if (vol > 1.0f) vol = 1.0f;

        if (vol < 0.999f) {  // si vol≈1, on peut skipper la boucle de gain
            for (size_t i = 0; i < num_samples; i++) {
                float s = (float)data[i] * vol;
                if (s > 32767.0f)  s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                data[i] = (int16_t)s;
            }
        }

        // 3) Envoi vers I2S
        size_t bytes_written = 0;
        esp_err_t err = i2s_channel_write(
            tx_chan,
            data,
            item_size,
            &bytes_written,
            portMAX_DELAY
        );

        if (err != ESP_OK) {
            ESP_LOGE(TAG, "i2s_channel_write error: %d", err);
        }

        // 4) On rend le bloc au ring buffer
        vRingbufferReturnItem(s_audio_ringbuf, (void *)data);
    }
}
/* -------------------------------------------------------------------------- */
/*                           API publique app_audio                           */
/* -------------------------------------------------------------------------- */

void app_audio_init(void)
{
    ESP_LOGI(TAG, "app_audio_init()");

    i2s_init_tx();

    // Création du ring buffer
    s_audio_ringbuf = xRingbufferCreate(
        AUDIO_RINGBUF_SIZE,
        RINGBUF_TYPE_BYTEBUF
    );

    if (s_audio_ringbuf == NULL) {
        ESP_LOGE(TAG, "Failed to create audio ring buffer");
        return;
    }

    app_bt_audio_init(s_audio_ringbuf);

    // Tâche output : lit le ring buffer, applique DSP et envoie à l'I2S
    xTaskCreatePinnedToCore(
        audio_output_task,
        "audio_output_task",
        4096,
        NULL,
        6,
        NULL,
        1 // CORE 1
    );
}
