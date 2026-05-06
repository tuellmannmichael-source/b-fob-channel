/**
 * Unit tests for the RSSI Kalman filter.
 *
 * Compile standalone:
 *   gcc -o test_rssi_filter test_rssi_filter.c \
 *       ../firmware/src/triangulation/rssi_filter.c \
 *       -I../firmware/src -lm
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

#include "triangulation/rssi_filter.h"

#define ASSERT_NEAR(a, b, tol) \
    do { \
        float _a = (a), _b = (b), _t = (tol); \
        if (fabsf(_a - _b) > _t) { \
            fprintf(stderr, "FAIL: %s:%d: %.4f != %.4f (tol %.4f)\n", \
                    __FILE__, __LINE__, _a, _b, _t); \
            assert(0); \
        } \
    } while (0)

static void test_init(void)
{
    printf("test_init... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 1.0f, 4.0f, -60.0f);

    ASSERT_NEAR(rssi_filter_get(&f), -60.0f, 0.001f);

    printf("OK\n");
}

static void test_converges_to_constant(void)
{
    printf("test_converges_to_constant... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 0.5f, 8.0f, -80.0f); /* Start far from target */

    /* Feed constant -60 dBm */
    for (int i = 0; i < 200; i++) {
        rssi_filter_update(&f, -60.0f);
    }

    ASSERT_NEAR(rssi_filter_get(&f), -60.0f, 0.05f);

    printf("OK\n");
}

static void test_smooths_noise(void)
{
    printf("test_smooths_noise... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 0.5f, 16.0f, -65.0f);

    /* Alternate between -60 and -70 (mean = -65) */
    for (int i = 0; i < 200; i++) {
        float z = (i % 2 == 0) ? -60.0f : -70.0f;
        rssi_filter_update(&f, z);
    }

    /* Should converge near the mean */
    ASSERT_NEAR(rssi_filter_get(&f), -65.0f, 1.0f);

    printf("OK\n");
}

static void test_tracks_step_change(void)
{
    printf("test_tracks_step_change... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 1.0f, 4.0f, -50.0f);

    /* Settle at -50 */
    for (int i = 0; i < 100; i++) {
        rssi_filter_update(&f, -50.0f);
    }
    ASSERT_NEAR(rssi_filter_get(&f), -50.0f, 0.1f);

    /* Step change to -70 */
    for (int i = 0; i < 100; i++) {
        rssi_filter_update(&f, -70.0f);
    }
    ASSERT_NEAR(rssi_filter_get(&f), -70.0f, 0.1f);

    printf("OK\n");
}

static void test_kalman_gain_decreases(void)
{
    printf("test_kalman_gain_decreases... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 0.1f, 16.0f, -60.0f);

    /* The covariance P should decrease over time as the filter gains
     * confidence, meaning new measurements have less impact. */
    float p_initial = f.p;
    for (int i = 0; i < 50; i++) {
        rssi_filter_update(&f, -60.0f);
    }
    assert(f.p < p_initial);

    printf("OK (P: %.4f -> %.4f)\n", p_initial, f.p);
}

int main(void)
{
    printf("=== RSSI Filter Unit Tests ===\n\n");

    test_init();
    test_converges_to_constant();
    test_smooths_noise();
    test_tracks_step_change();
    test_kalman_gain_decreases();

    printf("\nAll tests passed.\n");
    return 0;
}
