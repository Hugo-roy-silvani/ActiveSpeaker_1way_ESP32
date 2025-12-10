#include "app_dsp.h"
#include "board_config.h"   // pour SAMPLE_RATE

#include "esp_log.h"
#include <math.h>

static const char *TAG = "APP_DSP";

/* -------------------------------------------------------------------------- */
/*                       Structures internes runtime                          */
/* -------------------------------------------------------------------------- */

typedef struct {
    dsp_filter_type_t type;
    float fc;
    float Q;
    float gain_db;

    float b0, b1, b2;
    float a1, a2;

    float z1, z2;
    int   enabled;
} dsp_biquad_t;

static dsp_biquad_t g_bands[DSP_MAX_BIQUADS];
static float g_fs = (float)SAMPLE_RATE;

typedef struct {
    float threshold;       // seuil en linéaire (ex: -1 dB = ~0.89)
    float release_coeff;   // coefficient de release
    float gain;            // gain courant (<= 1)
    int   enabled;
} dsp_limiter_t;

static dsp_limiter_t g_limiter;

#define DSP_LOUDNESS_LOWSHELF_INDEX   6
#define DSP_LOUDNESS_HIGHSHELF_INDEX  7

typedef struct {
    float threshold_db;    // seuil en dBFS (ex: -12 dB)
    float ratio;           // ex: 2.0 pour 2:1
    float attack_ms;       // temps d'attack en ms
    float release_ms;      // temps de release en ms

    float attack_coeff;    // coeff IIR pour l'enveloppe (attack)
    float release_coeff;   // coeff IIR pour l'enveloppe (release)

    float makeup_db;       // gain de compensation en dB
    float makeup_lin;      // gain linéaire pré-calculé

    float env;             // enveloppe peak lissée [0..1]
    int   enabled;         // 0 = bypass, 1 = actif
} dsp_compressor_t;

static dsp_compressor_t g_comp;


/* -------------------------------------------------------------------------- */
/*                       Helpers internes : états & coeffs                    */
/* -------------------------------------------------------------------------- */

static void dsp_biquad_reset_state(dsp_biquad_t *bq)
{
    bq->z1 = 0.0f;
    bq->z2 = 0.0f;
}

static void dsp_biquad_set_bypass(dsp_biquad_t *bq)
{
    bq->b0 = 1.0f;
    bq->b1 = 0.0f;
    bq->b2 = 0.0f;
    bq->a1 = 0.0f;
    bq->a2 = 0.0f;
    dsp_biquad_reset_state(bq);
}

/* Recalcule les coeffs du biquad en fonction de type/fc/Q/gain */
static void dsp_biquad_recompute(dsp_biquad_t *bq)
{
    if (bq->type == DSP_FILTER_BYPASS || !bq->enabled) {
        dsp_biquad_set_bypass(bq);
        return;
    }

    float fs = g_fs;
    float fc = bq->fc;
    float Q  = (bq->Q <= 0.0f) ? 0.707f : bq->Q;
    float A  = powf(10.0f, bq->gain_db / 40.0f);  // amplitude linéaire

    float w0     = 2.0f * (float)M_PI * fc / fs;
    float cos_w0 = cosf(w0);
    float sin_w0 = sinf(w0);
    float alpha  = sin_w0 / (2.0f * Q);

    float b0, b1, b2, a0, a1, a2;

    switch (bq->type) {

    case DSP_FILTER_LOWPASS:
        // RBJ low-pass
        b0 =  (1.0f - cos_w0) / 2.0f;
        b1 =   1.0f - cos_w0;
        b2 =  (1.0f - cos_w0) / 2.0f;
        a0 =   1.0f + alpha;
        a1 =  -2.0f * cos_w0;
        a2 =   1.0f - alpha;
        break;

    case DSP_FILTER_HIGHPASS:
        // RBJ high-pass
        b0 =  (1.0f + cos_w0) / 2.0f;
        b1 = -(1.0f + cos_w0);
        b2 =  (1.0f + cos_w0) / 2.0f;
        a0 =   1.0f + alpha;
        a1 =  -2.0f * cos_w0;
        a2 =   1.0f - alpha;
        break;

    case DSP_FILTER_PEAK: {
        // RBJ peak/notch
        float alphaA    = alpha * A;
        float alphaDivA = alpha / A;
        b0 = 1.0f + alphaA;
        b1 = -2.0f * cos_w0;
        b2 = 1.0f - alphaA;
        a0 = 1.0f + alphaDivA;
        a1 = -2.0f * cos_w0;
        a2 = 1.0f - alphaDivA;
        } break;

    case DSP_FILTER_LOWSHELF: {
        // RBJ low-shelf
        float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;
        b0 =    A * ((A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrtA_alpha);
        b1 =  2.0f * A * ((A - 1.0f) - (A + 1.0f) * cos_w0);
        b2 =    A * ((A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrtA_alpha);
        a0 =        (A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrtA_alpha;
        a1 =   -2.0f * ((A - 1.0f) + (A + 1.0f) * cos_w0);
        a2 =        (A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrtA_alpha;
        } break;

    case DSP_FILTER_HIGHSHELF: {
        // RBJ high-shelf
        float two_sqrtA_alpha = 2.0f * sqrtf(A) * alpha;
        b0 =    A * ((A + 1.0f) + (A - 1.0f) * cos_w0 + two_sqrtA_alpha);
        b1 = -2.0f * A * ((A - 1.0f) + (A + 1.0f) * cos_w0);
        b2 =    A * ((A + 1.0f) + (A - 1.0f) * cos_w0 - two_sqrtA_alpha);
        a0 =        (A + 1.0f) - (A - 1.0f) * cos_w0 + two_sqrtA_alpha;
        a1 =    2.0f * ((A - 1.0f) - (A + 1.0f) * cos_w0);
        a2 =        (A + 1.0f) - (A - 1.0f) * cos_w0 - two_sqrtA_alpha;
        } break;

    case DSP_FILTER_BYPASS:
    default:
        dsp_biquad_set_bypass(bq);
        return;
    }

    // Normalisation a0 = 1
    bq->b0 = b0 / a0;
    bq->b1 = b1 / a0;
    bq->b2 = b2 / a0;
    bq->a1 = a1 / a0;
    bq->a2 = a2 / a0;

    dsp_biquad_reset_state(bq);
}

static inline float dsp_limiter_process_sample(dsp_limiter_t *lim, float x)
{
    float in = x;
    float abs_in = fabsf(in);

    // Gain souhaité pour ne pas dépasser le seuil
    float desired_gain = 1.0f;
    if (abs_in > lim->threshold) {
        // On calcule le gain qui ramène le signal au seuil
        desired_gain = lim->threshold / (abs_in + 1e-12f);
    }

    // Attack instantanée (on descend le gain tout de suite si nécessaire)
    if (desired_gain < lim->gain) {
        lim->gain = desired_gain;
    } else {
        // Release progressif vers 1.0
        // gain[n] = release_coeff * gain[n-1] + (1 - release_coeff) * 1.0
        lim->gain = lim->release_coeff * lim->gain
                  + (1.0f - lim->release_coeff) * 1.0f;
    }

    float out = in * lim->gain;
    return out;
}

/* -------------------------------------------------------------------------- */
/*                    Presets intégrés (configs uniquement)                   */
/* -------------------------------------------------------------------------- */

/* Preset 0 : Neutre / sécurité FRS5X */
static const dsp_preset_t preset_neutre = {
    .name      = "Neutre",
    .num_bands = 2,
    .bands = {
        { DSP_FILTER_HIGHPASS, 130.0f, 0.8f, 0.0f, 1 }, // HPF protection
        { DSP_FILTER_LOWSHELF, 220.0f, 0.7f, 2.0f, 1 }, // léger corps
    },
};

/* Preset 1 : HiFi doux / nearfield */
static const dsp_preset_t preset_hifi = {
    .name      = "HiFi doux",
    .num_bands = 4,
    .bands = {
        { DSP_FILTER_HIGHPASS,   110.0f, 0.8f,  0.0f, 1 },
        { DSP_FILTER_LOWSHELF,   220.0f, 0.7f,  3.0f, 1 },
        { DSP_FILTER_PEAK,      3000.0f, 1.0f, -1.0f, 1 },
        { DSP_FILTER_HIGHSHELF,  9500.0f, 0.7f, -4.0f, 1 },
    },
};

/* Preset 2 : Bas volume / loudness light */
static const dsp_preset_t preset_loudness = {
    .name      = "Bas volume",
    .num_bands = 4,
    .bands = {
        { DSP_FILTER_HIGHPASS,    80.0f, 0.8f,  0.0f, 1 },
        { DSP_FILTER_LOWSHELF,   160.0f, 0.7f,  3.0f, 1 },
        { DSP_FILTER_PEAK,       700.0f, 1.0f,  2.0f, 1 },
        { DSP_FILTER_HIGHSHELF, 10000.0f, 0.7f,  2.0f, 1 },
    },
};

/* -------------------------------------------------------------------------- */
/*                               API publique                                 */
/* -------------------------------------------------------------------------- */

void app_dsp_init(void)
{
    g_fs = (float)SAMPLE_RATE;
    ESP_LOGI(TAG, "DSP init (Fs = %.1f Hz)", g_fs);

    // Init interne des bandes (bypass)
    for (int i = 0; i < DSP_MAX_BIQUADS; i++) {
        g_bands[i].type    = DSP_FILTER_BYPASS;
        g_bands[i].fc      = 1000.0f;
        g_bands[i].Q       = 0.707f;
        g_bands[i].gain_db = 0.0f;
        g_bands[i].enabled = 0;
        g_bands[i].z1 = 0.0f;
        g_bands[i].z2 = 0.0f;
        dsp_biquad_set_bypass(&g_bands[i]);
    }

    int idx_hpf = 0;

    g_bands[idx_hpf].type    = DSP_FILTER_HIGHPASS;
    g_bands[idx_hpf].fc      = 120.0f;   // fréquence recommandée pour protéger un petit HP
    g_bands[idx_hpf].Q       = 0.707f;  // Butterworth
    g_bands[idx_hpf].gain_db = 0.0f;
    g_bands[idx_hpf].enabled = 1;

    dsp_biquad_recompute(&g_bands[idx_hpf]);

    //Loudness dinamic//
    // Low-shelf pour les basses (150–200 Hz)
    int idx_ls = DSP_LOUDNESS_LOWSHELF_INDEX;
    g_bands[idx_ls].type    = DSP_FILTER_LOWSHELF;
    g_bands[idx_ls].fc      = 180.0f;   // à ajuster si besoin
    g_bands[idx_ls].Q       = 0.7f;
    g_bands[idx_ls].gain_db = 0.0f;     // 0 dB par défaut
    g_bands[idx_ls].enabled = 1;
    dsp_biquad_recompute(&g_bands[idx_ls]);

    // High-shelf pour les aigus (8–10 kHz)
    int idx_hs = DSP_LOUDNESS_HIGHSHELF_INDEX;
    g_bands[idx_hs].type    = DSP_FILTER_HIGHSHELF;
    g_bands[idx_hs].fc      = 9000.0f;
    g_bands[idx_hs].Q       = 0.7f;
    g_bands[idx_hs].gain_db = 0.0f;     // 0 dB par défaut
    g_bands[idx_hs].enabled = 1;
    dsp_biquad_recompute(&g_bands[idx_hs]);


    // Limiter init //
    float threshold_db = -1.0f;   // seuil à -1 dBFS
    float threshold_lin = powf(10.0f, threshold_db / 20.0f);

    float release_ms = 80.0f;     // temps de release ≈ 80 ms
    float tau_release = release_ms / 1000.0f;
    float release_coeff = expf(-1.0f / (tau_release * g_fs));

    g_limiter.threshold      = threshold_lin;
    g_limiter.release_coeff  = release_coeff;
    g_limiter.gain           = 1.0f;
    g_limiter.enabled        = 1;   // activé par défaut

    // Compressor init //
    g_comp.threshold_db = -12.0f;   // seuil -12 dBFS
    g_comp.ratio        = 2.0f;     // 2:1
    g_comp.attack_ms    = 5.0f;     // attack 5 ms
    g_comp.release_ms   = 80.0f;    // release 80 ms
    g_comp.makeup_db    = 0.0f;     // pas de makeup au début

    float attack_sec  = g_comp.attack_ms  / 1000.0f;
    float release_sec = g_comp.release_ms / 1000.0f;

    g_comp.attack_coeff  = expf(-1.0f / (attack_sec  * g_fs));
    g_comp.release_coeff = expf(-1.0f / (release_sec * g_fs));

    g_comp.makeup_lin = powf(10.0f, g_comp.makeup_db / 20.0f);
    g_comp.env        = 0.0f;

    g_comp.enabled    = 0;  // on peut le laisser activé par défaut


    // Appliquer preset par défaut
    const dsp_preset_t *p = app_dsp_get_builtin_preset(DSP_PRESET_HIFI);
    app_dsp_apply_preset(p);

       
}

void app_dsp_set_band(int index,
                      dsp_filter_type_t type,
                      float fc,
                      float Q,
                      float gain_db)
{
    if (index < 0 || index >= DSP_MAX_BIQUADS) {
        ESP_LOGW(TAG, "Invalid band index %d", index);
        return;
    }

    dsp_biquad_t *bq = &g_bands[index];

    bq->type    = type;
    bq->fc      = fc;
    bq->Q       = Q;
    bq->gain_db = gain_db;

    dsp_biquad_recompute(bq);

    ESP_LOGI(TAG, "Band %d set: type=%d, fc=%.1f Hz, Q=%.3f, gain=%.1f dB",
             index, (int)type, fc, Q, gain_db);
}

void app_dsp_set_band_enabled(int index, int enabled)
{
    if (index < 0 || index >= DSP_MAX_BIQUADS) {
        return;
    }

    g_bands[index].enabled = (enabled != 0);

    if (!g_bands[index].enabled) {
        dsp_biquad_set_bypass(&g_bands[index]);
    } else {
        dsp_biquad_recompute(&g_bands[index]);
    }
}

/* Appliquer un preset générique */
/*
void app_dsp_apply_preset(const dsp_preset_t *preset)
{
    if (!preset) {
        return;
    }

    // Désactiver toutes les bandes
    for (int i = 0; i < DSP_MAX_BIQUADS; i++) {
        app_dsp_set_band_enabled(i, 0);
    }

    int n = (preset->num_bands > DSP_MAX_BIQUADS)
                ? DSP_MAX_BIQUADS
                : preset->num_bands;

    for (int i = 0; i < n; i++) {
        const dsp_band_config_t *cfg = &preset->bands[i];
        app_dsp_set_band(i, cfg->type, cfg->fc, cfg->Q, cfg->gain_db);
        app_dsp_set_band_enabled(i, cfg->enabled);
    }

    ESP_LOGI(TAG, "Preset applied: %s", preset->name ? preset->name : "(unnamed)");
}
    */
void app_dsp_apply_preset(const dsp_preset_t *preset)
{
    if (!preset) {
        return;
    }

    // Désactiver toutes les bandes SAUF celles réservées au loudness
    for (int i = 0; i < DSP_MAX_BIQUADS; i++) {
        if (i == DSP_LOUDNESS_LOWSHELF_INDEX ||
            i == DSP_LOUDNESS_HIGHSHELF_INDEX) {
            continue; // on garde ces bandes pour le loudness
        }
        app_dsp_set_band_enabled(i, 0);
    }

    int n = (preset->num_bands > DSP_MAX_BIQUADS)
                ? DSP_MAX_BIQUADS
                : preset->num_bands;

    for (int i = 0; i < n; i++) {
        // Si jamais un preset utilise trop de bandes, on s'assure
        // de ne pas écraser celles du loudness
        if (i == DSP_LOUDNESS_LOWSHELF_INDEX ||
            i == DSP_LOUDNESS_HIGHSHELF_INDEX) {
            continue;
        }

        const dsp_band_config_t *cfg = &preset->bands[i];
        app_dsp_set_band(i, cfg->type, cfg->fc, cfg->Q, cfg->gain_db);
        app_dsp_set_band_enabled(i, cfg->enabled);
    }

    ESP_LOGI(TAG, "Preset applied: %s", preset->name ? preset->name : "(unnamed)");
}


/* Renvoyer un preset intégré par ID */
const dsp_preset_t *app_dsp_get_builtin_preset(dsp_preset_id_t preset_id)
{
    switch (preset_id) {
    case DSP_PRESET_NEUTRE:   return &preset_neutre;
    case DSP_PRESET_HIFI:     return &preset_hifi;
    case DSP_PRESET_LOUDNESS: return &preset_loudness;
    default:                  return &preset_neutre;
    }
}

/* -------------------------------------------------------------------------- */
/*                            Compressor                                      */
/* -------------------------------------------------------------------------- */
static inline float dsp_compressor_process_sample(dsp_compressor_t *c, float x)
{
    const float eps = 1e-8f;
    float in = x;
    float abs_in = fabsf(in);

    // --- Mise à jour de l'enveloppe peak lissée ---
    float env = c->env;
    if (abs_in > env) {
        // Attack : quand le signal monte
        env = c->attack_coeff * env + (1.0f - c->attack_coeff) * abs_in;
    } else {
        // Release : quand le signal descend
        env = c->release_coeff * env + (1.0f - c->release_coeff) * abs_in;
    }
    c->env = env;

    if (env < eps) {
        // Silence -> seulement le makeup gain
        return in * c->makeup_lin;
    }

    // Niveau de l'enveloppe en dBFS
    float env_db = 20.0f * log10f(env + eps);

    float gain_db = 0.0f;

    if (env_db > c->threshold_db) {
        // Formule standard :
        // out_db = thr + (env_db - thr)/ratio
        // => gain_db = out_db - env_db = (1 - 1/ratio) * (thr - env_db)
        float one_minus_inv = 1.0f - (1.0f / c->ratio);
        gain_db = one_minus_inv * (c->threshold_db - env_db); // <= 0 dB
    }

    // Gain total = compression + makeup
    float total_gain_db  = gain_db + c->makeup_db;
    float total_gain_lin = powf(10.0f, total_gain_db / 20.0f);

    return in * total_gain_lin;
}

void app_dsp_comp_set_enabled(int enabled)
{
    g_comp.enabled = (enabled != 0);
    if (!g_comp.enabled) {
        g_comp.env = 0.0f;
    }
    ESP_LOGI(TAG, "Compressor %s", g_comp.enabled ? "ENABLED" : "DISABLED");
}

void app_dsp_comp_set_params(float threshold_db,
                             float ratio,
                             float attack_ms,
                             float release_ms,
                             float makeup_db)
{
    // Clamp basique pour éviter les valeurs absurdes
    if (ratio < 1.0f) ratio = 1.0f;
    if (attack_ms  < 0.1f) attack_ms  = 0.1f;
    if (release_ms < 1.0f) release_ms = 1.0f;

    g_comp.threshold_db = threshold_db;
    g_comp.ratio        = ratio;
    g_comp.attack_ms    = attack_ms;
    g_comp.release_ms   = release_ms;
    g_comp.makeup_db    = makeup_db;

    float attack_sec  = g_comp.attack_ms  / 1000.0f;
    float release_sec = g_comp.release_ms / 1000.0f;

    g_comp.attack_coeff  = expf(-1.0f / (attack_sec  * g_fs));
    g_comp.release_coeff = expf(-1.0f / (release_sec * g_fs));

    g_comp.makeup_lin = powf(10.0f, g_comp.makeup_db / 20.0f);

    ESP_LOGI(TAG,
             "Compressor params: thr=%.1f dB, ratio=%.1f:1, atk=%.1f ms, rel=%.1f ms, makeup=%.1f dB",
             g_comp.threshold_db,
             g_comp.ratio,
             g_comp.attack_ms,
             g_comp.release_ms,
             g_comp.makeup_db);
}



/* -------------------------------------------------------------------------- */
/*                            Limiter                                         */
/* -------------------------------------------------------------------------- */

void app_dsp_limiter_set_enabled(int enabled)
{
    g_limiter.enabled = (enabled != 0);
    if (!g_limiter.enabled) {
        g_limiter.gain = 1.0f;
    }
}

void app_dsp_limiter_set_threshold_db(float threshold_db)
{
    // clamp raisonnable
    if (threshold_db > -0.1f) threshold_db = -0.1f;  // évite d'être à 0 dB pile
    if (threshold_db < -12.0f) threshold_db = -12.0f;

    g_limiter.threshold = powf(10.0f, threshold_db / 20.0f);
    // on remet le gain à 1 pour repartir proprement
    g_limiter.gain = 1.0f;

    ESP_LOGI(TAG, "Limiter threshold set to %.1f dBFS (lin=%.3f)",
             threshold_db, g_limiter.threshold);
}

/* -------------------------------------------------------------------------- */
/*                            Dynamic Loudness                                */
/* -------------------------------------------------------------------------- */

void app_dsp_set_loudness_level(float vol_0_to_1)
{
    // Clamp volume 0..1
    if (vol_0_to_1 < 0.0f) vol_0_to_1 = 0.0f;
    if (vol_0_to_1 > 1.0f) vol_0_to_1 = 1.0f;

    // v = volume tel [0..1]
    float v = vol_0_to_1;

    // On veut plus de loudness quand le volume est BAS,
    // et 0 dB de loudness quand le volume est proche de 1.
    //
    // f = 1 - v  -> 1 à volume mini, 0 à volume max
    float f = 1.0f - v;

    // Courbe un peu plus douce : square (accentue l'effet aux très faibles volumes)
    float shape = f * f;

    // Gains max appliqués quand v = 0
    const float max_bass_boost_db   = 5.0f;  // +5 dB basses max
    const float max_treble_boost_db = 3.0f;  // +3 dB aigus max

    float bass_gain_db   = max_bass_boost_db   * shape;
    float treble_gain_db = max_treble_boost_db * shape;

    int idx_ls = DSP_LOUDNESS_LOWSHELF_INDEX;
    int idx_hs = DSP_LOUDNESS_HIGHSHELF_INDEX;

    // Sécurité : vérifier que les indices sont dans la plage
    if (idx_ls >= 0 && idx_ls < DSP_MAX_BIQUADS) {
        g_bands[idx_ls].type    = DSP_FILTER_LOWSHELF;
        // On garde fc / Q tels que tu les as configurés à l'init
        g_bands[idx_ls].gain_db = bass_gain_db;
        g_bands[idx_ls].enabled = 1;
        dsp_biquad_recompute(&g_bands[idx_ls]);
    }

    if (idx_hs >= 0 && idx_hs < DSP_MAX_BIQUADS) {
        g_bands[idx_hs].type    = DSP_FILTER_HIGHSHELF;
        g_bands[idx_hs].gain_db = treble_gain_db;
        g_bands[idx_hs].enabled = 1;
        dsp_biquad_recompute(&g_bands[idx_hs]);
    }

    ESP_LOGI(TAG, "Loudness: vol=%.2f -> bass=%.1f dB, treble=%.1f dB",
             v, bass_gain_db, treble_gain_db);
}


/* -------------------------------------------------------------------------- */
/*                       Traitement d'un bloc int16 mono                       */
/* -------------------------------------------------------------------------- */

static inline float dsp_biquad_process_sample(dsp_biquad_t *bq, float x)
{
    // Direct Form I transposé
    float y = bq->b0 * x + bq->z1;
    bq->z1  = bq->b1 * x - bq->a1 * y + bq->z2;
    bq->z2  = bq->b2 * x - bq->a2 * y;
    return y;
}

void app_dsp_process_block_int16(int16_t *buffer, size_t num_samples)
{
    for (size_t n = 0; n < num_samples; n++) {
        // int16 -> float [-1, 1]
        float x = (float)buffer[n] / 32768.0f;
        float y = x;

        // Cascade de biquads
        for (int i = 0; i < DSP_MAX_BIQUADS; i++) {
            if (g_bands[i].type == DSP_FILTER_BYPASS || !g_bands[i].enabled) {
                continue;
            }
            y = dsp_biquad_process_sample(&g_bands[i], y);
        }

        if (g_comp.enabled) {
            y = dsp_compressor_process_sample(&g_comp, y);
        }

        // Limiteur mono
        if (g_limiter.enabled) {
            y = dsp_limiter_process_sample(&g_limiter, y);
        }

        // Limite de sécurité
        if (y > 1.0f)  y = 1.0f;
        if (y < -1.0f) y = -1.0f;

        buffer[n] = (int16_t)(y * 32767.0f);
    }
}
