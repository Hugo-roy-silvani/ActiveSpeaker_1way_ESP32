#pragma once

#include <stdint.h>
#include <stddef.h>

#define DSP_MAX_BIQUADS   8

typedef enum {
    DSP_FILTER_BYPASS = 0,
    DSP_FILTER_LOWPASS,
    DSP_FILTER_HIGHPASS,
    DSP_FILTER_PEAK,
    DSP_FILTER_LOWSHELF,
    DSP_FILTER_HIGHSHELF,
} dsp_filter_type_t;

/* Configuration "logique" d'une bande, sans états internes */
typedef struct {
    dsp_filter_type_t type;
    float fc;        // fréquence en Hz
    float Q;
    float gain_db;   // pour peak / shelving
    int   enabled;
} dsp_band_config_t;

/* Description complète d'un preset (config seulement) */
typedef struct {
    const char *name;
    int         num_bands;
    dsp_band_config_t bands[DSP_MAX_BIQUADS];
} dsp_preset_t;

typedef enum {
    DSP_PRESET_NEUTRE = 0,
    DSP_PRESET_HIFI,
    DSP_PRESET_LOUDNESS,
    DSP_PRESET_COUNT
} dsp_preset_id_t;


/* Initialisation du moteur DSP (à appeler une seule fois au démarrage) */
void app_dsp_init(void);

/* Configuration d'une bande runtime (coeffs recalculés) */
void app_dsp_set_band(int index,
                      dsp_filter_type_t type,
                      float fc,
                      float Q,
                      float gain_db);

/* Activation / désactivation d'une bande */
void app_dsp_set_band_enabled(int index, int enabled);

/* Application d'un preset (intégré ou construit dynamiquement) */
void app_dsp_apply_preset(const dsp_preset_t *preset);

/* Récupérer un preset intégré */
const dsp_preset_t *app_dsp_get_builtin_preset(dsp_preset_id_t preset_id);

/* Traitement in-place d'un bloc mono 16 bits */
void app_dsp_process_block_int16(int16_t *buffer, size_t num_samples);

/* Limiteur mono (peak limiter simple) */
void app_dsp_limiter_set_enabled(int enabled);
void app_dsp_limiter_set_threshold_db(float threshold_db);

// dynamic loudness
void app_dsp_set_loudness_level(float vol_0_to_1);

/* --- Compresseur mono large bande --- */

/* Activer / désactiver le compresseur */
void app_dsp_comp_set_enabled(int enabled);

/* Régler les paramètres principaux du compresseur */
void app_dsp_comp_set_params(float threshold_db,
                             float ratio,
                             float attack_ms,
                             float release_ms,
                             float makeup_db);
