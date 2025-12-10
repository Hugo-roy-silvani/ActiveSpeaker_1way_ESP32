#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

/* Initialise le Bluetooth A2DP et le lie à un ring buffer audio
 * Le ring buffer doit être déjà créé (dans app_audio).
 */
void app_bt_audio_init(RingbufHandle_t ringbuf);
float app_bt_get_remote_volume_level(void);