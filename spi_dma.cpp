// ================================================================
// Low-Shelving IIR (Q12) + Bias-Estimator + Button
// RMS AC (DC-frei) + Clip-Check  -> Gain bei hohen f ~ 0 dB messbar
// Mode 0 = -10 dB (Cut), Mode 1 = -35 dB (Cut)
// ================================================================
#include <cstdint>
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/spi.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

// ----------------- Pins / Parameter -----------------
#define ADC_PIN 26
#define SPI_PORT spi0
#define PIN_MOSI 19
#define PIN_SCK 18
#define PIN_CS 17
#define BUTTON_PIN 15

// Timer: 22 us => ~45.45 kHz
#define TIMER_US 22

// ----------------- Q12 Koeffizienten -----------------
// Mode 1: -35 dB (Cut)
#define B0_35 3188
#define B1_35 -6128
#define B2_35 2949
#define A1_35 -5870
#define A2_35 2300

// Mode 0: -10 dB (Cut)
#define B0_10 3974
#define B1_10 -7637
#define B2_10 3675
#define A1_10 -7624
#define A2_10 3565

static volatile int16_t B0 = B0_10, B1 = B1_10, B2 = B2_10, A1 = A1_10, A2 = A2_10;
static volatile int mode = 0; // 0=-10dB, 1=-35dB

// ----------------- Filterzustände (AC) -----------------
static int16_t x_1 = 0, x_2 = 0;
static int16_t y_1 = 0, y_2 = 0;

// ----------------- Bias-Estimator (dc_est) -----------------
static volatile int32_t dc_est = 2048;
#define DC_ALPHA_SHIFT 11
#define DC_MIN 1200
#define DC_MAX 2900

// ----------------- Debug -----------------
static volatile uint32_t sample_count = 0;
static volatile uint16_t last_adc = 0;
static volatile uint16_t last_dac = 0;
static volatile bool overflow_detected = false;

// Clip counter
static volatile uint32_t adc_clip = 0;
static volatile uint32_t dac_clip = 0;

// ----------------- RMS/Mean Akkus (AC, DC-frei per Varianz) -----------------
// Wir sammeln SUM und SUMSQ, damit RMS_AC = sqrt(E[x^2] - (E[x])^2)
static volatile int64_t sum_in = 0;
static volatile int64_t sum_out = 0;
static volatile uint64_t sumsq_in = 0;
static volatile uint64_t sumsq_out = 0;
static volatile uint32_t rms_N = 0;

static struct repeating_timer timer;

// ----------------- DAC -----------------

static inline void dac_write(uint16_t v12)
{
    uint16_t frame = 0x3000 | (v12 & 0x0FFF);
    gpio_put(PIN_CS, 0);
    spi_write16_blocking(SPI_PORT, &frame, 1);
    gpio_put(PIN_CS, 1);
}

// Erwartete Gains aus Koeffizienten (nur zur Anzeige)
static void print_expected_gains_q12(int16_t b0, int16_t b1, int16_t b2, int16_t a1, int16_t a2)
{
    // a0 = 4096 (Q12)
    int32_t num_dc = (int32_t)b0 + b1 + b2; // numerator at DC
    int32_t den_dc = 4096 + (int32_t)a1 + a2; // denominator at DC

    int32_t num_pi = (int32_t)b0 - b1 + b2; // numerator at pi
    int32_t den_pi = 4096 - (int32_t)a1 + a2; // denominator at pi

    float H0 = (den_dc != 0) ? ((float)num_dc / (float)den_dc) : 0.0f;
    float Hpi = (den_pi != 0) ? ((float)num_pi / (float)den_pi) : 0.0f;

    printf("Expected: DC=%.2f dB | High(pi)=%.2f dB\n",
           20.0f * log10f(fabsf(H0) + 1e-12f),
           20.0f * log10f(fabsf(Hpi) + 1e-12f));
}

// ----------------- Mode setzen (atomar) -----------------
static inline void set_mode_atomic(int m)
{
    uint32_t irq = save_and_disable_interrupts();

    if (m == 0)
    {
        B0 = B0_10;
        B1 = B1_10;
        B2 = B2_10;
        A1 = A1_10;
        A2 = A2_10;
    }
    else
    {
        B0 = B0_35;
        B1 = B1_35;
        B2 = B2_35;
        A1 = A1_35;
        A2 = A2_35;
    }

    // reset filter + measurement
    x_1 = x_2 = 0;
    y_1 = y_2 = 0;

    sum_in = sum_out = 0;
    sumsq_in = sumsq_out = 0;
    rms_N = 0;

    overflow_detected = false;
    adc_clip = dac_clip = 0;

    restore_interrupts(irq);

    // Print expected gains (non-ISR)
    printf("Mode %d set (%s). ", m, m ? "-35 dB" : "-10 dB");
    print_expected_gains_q12(B0, B1, B2, A1, A2);
}

// ----------------- ISR -----------------
static bool audio_isr(struct repeating_timer *t)
{
    uint16_t adc_raw = adc_read();
    last_adc = adc_raw;

    if (adc_raw == 0 || adc_raw == 4095)
        adc_clip++;

    // Bias estimate (langsam)
    dc_est += ((int32_t)adc_raw - dc_est) >> DC_ALPHA_SHIFT;
    if (dc_est < DC_MIN)
        dc_est = DC_MIN;
    if (dc_est > DC_MAX)
        dc_est = DC_MAX;

    // AC input (um dc_est zentriert)
    int16_t x0 = (int16_t)((int32_t)adc_raw - dc_est);

    // IIR Q12
    int64_t acc =
        (int64_t)B0 * x0 +
        (int64_t)B1 * x_1 +
        (int64_t)B2 * x_2 -
        (int64_t)A1 * y_1 -
        (int64_t)A2 * y_2;

    if (acc > INT32_MAX || acc < INT32_MIN)
        overflow_detected = true;

    int16_t y0 = (int16_t)((int32_t)acc >> 12);

    

    // clip AC
    if (y0 > 2047)
        y0 = 2047;
    if (y0 < -2048)
        y0 = -2048;

    // back to DAC domain
    int32_t dac_val = (int32_t)y0 + dc_est;
    if (dac_val < 0)
        dac_val = 0;
    if (dac_val > 4095)
        dac_val = 4095;

    if (dac_val == 0 || dac_val == 4095)
        dac_clip++;

    last_dac = (uint16_t)dac_val;
    dac_write(last_dac);

    // --------- DC-freie RMS Messung: SUM + SUMSQ ----------

    int32_t xi = x0;
    int32_t yo = y0;

    sum_in += (int64_t)xi;
    sum_out += (int64_t)yo;

    sumsq_in += (uint64_t)((int64_t)xi * (int64_t)xi);
    sumsq_out += (uint64_t)((int64_t)yo * (int64_t)yo);
    rms_N++;

    // update delays
    x_2 = x_1;
    x_1 = x0;
    y_2 = y_1;
    y_1 = y0;

    sample_count++;
    return true;
}

// ----------------- Main -----------------
int main()
{
    stdio_init_all();
    sleep_ms(2000);

    printf("\n=== Low-Shelf Q12 + Bias + RMS_AC (500ms) + ClipCheck ===\n");
    printf("Timer=%dus => Fs ~ %.2f Hz\n", TIMER_US, 1000000.0f / (float)TIMER_US);

    adc_init();
    adc_gpio_init(ADC_PIN);
    adc_select_input(0);

    spi_init(SPI_PORT, 5 * 1000 * 1000);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);

    spi_set_format(
    spi0,
    16,                 // << BIT-ANZAHL (4 bis 16)
    SPI_CPOL_0,         // Clock Polarity
    SPI_CPHA_0,         // Clock Phase
    SPI_MSB_FIRST       // Bit Order
);

    gpio_init(BUTTON_PIN);
    gpio_set_dir(BUTTON_PIN, GPIO_IN);
    gpio_pull_up(BUTTON_PIN);

    set_mode_atomic(mode);

    add_repeating_timer_us(-TIMER_US, audio_isr, NULL, &timer);

    bool last_button = true;
    uint32_t last_count = 0;

    while (1)
    {
        sleep_ms(500);

        // Button toggle
        bool button = gpio_get(BUTTON_PIN);
        if (!button && last_button)
        {
            mode = !mode;
            set_mode_atomic(mode);
        }
        last_button = button;

        // snapshot + reset stats
        uint32_t diff;
        uint16_t adc_now, dac_now;
        int32_t dc_now;
        int64_t s_in, s_out;
        uint64_t ss_in, ss_out;
        uint32_t N;
        bool of;
        uint32_t ac, dc;

        uint32_t irq = save_and_disable_interrupts();

        diff = sample_count - last_count;
        last_count = sample_count;

        adc_now = last_adc;
        dac_now = last_dac;
        dc_now = dc_est;

        s_in = sum_in;
        sum_in = 0;
        s_out = sum_out;
        sum_out = 0;
        ss_in = sumsq_in;
        sumsq_in = 0;
        ss_out = sumsq_out;
        sumsq_out = 0;
        N = rms_N;
        rms_N = 0;

        of = overflow_detected;
        overflow_detected = false;

        ac = adc_clip;
        adc_clip = 0;
        dc = dac_clip;
        dac_clip = 0;

        restore_interrupts(irq);

        float bias_v = (dc_now * 3.3f) / 4095.0f;
        float adc_v = (adc_now * 3.3f) / 4095.0f;
        float dac_v = (dac_now * 3.3f) / 4095.0f;

        // --------- RMS_AC per Varianz ----------
        float rms_in_counts = 0.0f, rms_out_counts = 0.0f;
        float mean_in_counts = 0.0f, mean_out_counts = 0.0f;
        float gain_db = -120.0f;

        if (N > 0)
        {
            double ex = (double)s_in / (double)N;
            double ex2 = (double)ss_in / (double)N;
            double varx = ex2 - ex * ex;

            if (varx < 0.0)
                varx = 0.0; // numerische Rundung

            double ey = (double)s_out / (double)N;
            double ey2 = (double)ss_out / (double)N;
            double vary = ey2 - ey * ey;
            if (vary < 0.0)
                vary = 0.0;

            mean_in_counts = (float)ex;
            mean_out_counts = (float)ey;

            rms_in_counts = (float)sqrt(varx);
            rms_out_counts = (float)sqrt(vary);

            if (rms_in_counts > 1e-3f)
            {
                gain_db = 20.0f * log10f(rms_out_counts / rms_in_counts);
            }
        }

        float rms_in_v = (rms_in_counts * 3.3f) / 4095.0f;
        float rms_out_v = (rms_out_counts * 3.3f) / 4095.0f;

        // Residual-DC in AC-Signal (sollte nahe 0 sein)
        float mean_in_v = (mean_in_counts * 3.3f) / 4095.0f;
        float mean_out_v = (mean_out_counts * 3.3f) / 4095.0f;

        printf("Mode=%d | Bias=%.2fV | ADC=%.2fV | DAC=%.2fV | RMSin=%.3fV | RMSout=%.3fV | Gain=%6.1f dB | Fs≈%5lu Hz | meanIn=%.3fV | meanOut=%.3fV | ADCclip=%lu | DACclip=%lu",
               mode, bias_v, adc_v, dac_v,
               rms_in_v, rms_out_v, gain_db,
               (unsigned long)(diff * 2),
               mean_in_v, mean_out_v,
               (unsigned long)ac, (unsigned long)dc);

        if (of)
            printf(" [OVERFLOW]");
        printf("\n");
    }
}
