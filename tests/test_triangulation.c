/**
 * Unit tests for the trilateration math.
 *
 * Compile standalone (no Zephyr) with:
 *   gcc -o test_triangulation test_triangulation.c \
 *       ../firmware/src/triangulation/triangulation.c \
 *       ../firmware/src/triangulation/rssi_filter.c \
 *       -I../firmware/src -lm -DNUM_ANTENNAS=3 \
 *       -DCONFIG_TRIANGULATION_RSSI_REF=-59 \
 *       -DCONFIG_TRIANGULATION_PATH_LOSS_EXP_X10=30 \
 *       -DCONFIG_TRIANGULATION_KALMAN_Q_X10=5 \
 *       -DCONFIG_TRIANGULATION_KALMAN_R_X10=80
 */

#include <stdio.h>
#include <math.h>
#include <assert.h>

/* Provide Kconfig values for standalone compilation */
#ifndef CONFIG_TRIANGULATION_NUM_ANTENNAS
#define CONFIG_TRIANGULATION_NUM_ANTENNAS 3
#endif
#ifndef CONFIG_TRIANGULATION_RSSI_REF
#define CONFIG_TRIANGULATION_RSSI_REF -59
#endif
#ifndef CONFIG_TRIANGULATION_PATH_LOSS_EXP_X10
#define CONFIG_TRIANGULATION_PATH_LOSS_EXP_X10 30
#endif
#ifndef CONFIG_TRIANGULATION_KALMAN_Q_X10
#define CONFIG_TRIANGULATION_KALMAN_Q_X10 5
#endif
#ifndef CONFIG_TRIANGULATION_KALMAN_R_X10
#define CONFIG_TRIANGULATION_KALMAN_R_X10 80
#endif

#include "triangulation/triangulation.h"
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

/* ----------------------------------------------------------------------- */
/* Test: RSSI to distance conversion                                       */
/* ----------------------------------------------------------------------- */

static void test_rssi_to_distance(void)
{
    printf("test_rssi_to_distance... ");

    /* At reference distance (1 m), RSSI = RSSI_ref -> distance = 1.0 */
    float d = rssi_to_distance(-59.0f, -59.0f, 3.0f);
    ASSERT_NEAR(d, 1.0f, 0.01f);

    /* At 2 meters: RSSI = -59 - 10*3*log10(2) = -59 - 9.03 = -68.03 */
    d = rssi_to_distance(-68.03f, -59.0f, 3.0f);
    ASSERT_NEAR(d, 2.0f, 0.1f);

    /* At 5 meters: RSSI = -59 - 10*3*log10(5) = -59 - 20.97 = -79.97 */
    d = rssi_to_distance(-79.97f, -59.0f, 3.0f);
    ASSERT_NEAR(d, 5.0f, 0.1f);

    /* Very close: clamped to 0.1 m minimum */
    d = rssi_to_distance(-30.0f, -59.0f, 3.0f);
    ASSERT_NEAR(d, 0.1f, 0.01f);

    printf("OK\n");
}

/* ----------------------------------------------------------------------- */
/* Test: Kalman filter convergence                                         */
/* ----------------------------------------------------------------------- */

static void test_kalman_filter(void)
{
    printf("test_kalman_filter... ");

    rssi_filter_t f;
    rssi_filter_init(&f, 0.5f, 8.0f, -70.0f);

    /* Feed constant measurement of -65 dBm, filter should converge */
    for (int i = 0; i < 100; i++) {
        rssi_filter_update(&f, -65.0f);
    }
    ASSERT_NEAR(rssi_filter_get(&f), -65.0f, 0.1f);

    /* Feed a step change to -55, should track */
    for (int i = 0; i < 100; i++) {
        rssi_filter_update(&f, -55.0f);
    }
    ASSERT_NEAR(rssi_filter_get(&f), -55.0f, 0.1f);

    printf("OK\n");
}

/* ----------------------------------------------------------------------- */
/* Test: Trilateration with perfect distances                              */
/* ----------------------------------------------------------------------- */

static void test_trilateration_perfect(void)
{
    printf("test_trilateration_perfect... ");

    triangulation_ctx_t ctx;
    triangulation_init(&ctx);

    /*
     * Place keyfob at (0.5, 2.0).
     * Compute true distances to each antenna, convert to RSSI, and feed
     * many samples so the Kalman filter converges to the true value.
     */
    float true_x = 0.5f, true_y = 2.0f;

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        float dx = antenna_positions[i].x - true_x;
        float dy = antenna_positions[i].y - true_y;
        float true_dist = sqrtf(dx * dx + dy * dy);

        /* Convert distance to RSSI */
        float rssi = antenna_positions[i].rssi_ref -
                     10.0f * antenna_positions[i].path_loss_exp *
                     log10f(true_dist);

        /* Feed many samples to converge the Kalman filter */
        for (int s = 0; s < 200; s++) {
            triangulation_feed_rssi(&ctx, i, (int8_t)rssi);
        }
    }

    position_t pos;
    bool ok = triangulation_solve(&ctx, &pos);
    assert(ok);

    printf("Estimated: (%.2f, %.2f), True: (%.2f, %.2f)\n",
           pos.x, pos.y, true_x, true_y);

    ASSERT_NEAR(pos.x, true_x, 0.3f);
    ASSERT_NEAR(pos.y, true_y, 0.3f);

    printf("test_trilateration_perfect... OK\n");
}


/* ----------------------------------------------------------------------- */
/* Test: Weighted solver prioritizes low-sigma measurements                 */
/* ----------------------------------------------------------------------- */

static void test_weighted_channel_sounding_priority(void)
{
    printf("test_weighted_channel_sounding_priority... ");

    triangulation_ctx_t ctx;
    triangulation_init(&ctx);

    float true_x = 0.5f, true_y = 2.0f;

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        float dx = antenna_positions[i].x - true_x;
        float dy = antenna_positions[i].y - true_y;
        float true_dist = sqrtf(dx * dx + dy * dy);

        triangulation_feed_rssi(&ctx, i, -40);
        triangulation_feed_distance(&ctx, i, true_dist, 0.15f);
    }

    position_t pos;
    bool ok = triangulation_solve(&ctx, &pos);
    assert(ok);

    ASSERT_NEAR(pos.x, true_x, 0.2f);
    ASSERT_NEAR(pos.y, true_y, 0.2f);

    printf("OK\n");
}

/* ----------------------------------------------------------------------- */
/* Test: Innovation gate rejects implausible jump                           */
/* ----------------------------------------------------------------------- */

static void test_innovation_gate(void)
{
    printf("test_innovation_gate... ");

    triangulation_ctx_t ctx;
    triangulation_init(&ctx);

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        float dx = antenna_positions[i].x - 0.5f;
        float dy = antenna_positions[i].y - 2.0f;
        float d = sqrtf(dx * dx + dy * dy);
        triangulation_feed_distance(&ctx, i, d, 0.15f);
    }

    position_t pos;
    bool ok = triangulation_solve(&ctx, &pos);
    assert(ok);

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        float dx = antenna_positions[i].x - 20.0f;
        float dy = antenna_positions[i].y - 20.0f;
        float d = sqrtf(dx * dx + dy * dy);
        triangulation_feed_distance(&ctx, i, d, 0.10f);
    }

    ok = triangulation_solve(&ctx, &pos);
    assert(!ok);

    printf("OK\n");
}

/* ----------------------------------------------------------------------- */
/* Test: Weighted trilateration with direct distance feed (CS path)        */
/* ----------------------------------------------------------------------- */

static void test_trilateration_direct_distance(void)
{
    printf("test_trilateration_direct_distance... ");

    triangulation_ctx_t ctx;
    triangulation_init(&ctx);

    /*
     * Place keyfob at (0.0, 2.0).
     * Feed exact distances with a small sigma (simulating CS precision).
     */
    float true_x = 0.0f, true_y = 2.0f;

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        float dx = antenna_positions[i].x - true_x;
        float dy = antenna_positions[i].y - true_y;
        float true_dist = sqrtf(dx * dx + dy * dy);

        triangulation_feed_distance(&ctx,
                                    (uint8_t)i,
                                    true_dist,
                                    0.10f,
                                    TRI_MEAS_SOURCE_CHANNEL_SOUNDING);
    }

    position_t pos;
    bool ok = triangulation_solve(&ctx, &pos);
    assert(ok);

    printf("Estimated: (%.2f, %.2f), True: (%.2f, %.2f)\n",
           pos.x, pos.y, true_x, true_y);

    ASSERT_NEAR(pos.x, true_x, 0.1f);
    ASSERT_NEAR(pos.y, true_y, 0.1f);

    printf("test_trilateration_direct_distance... OK\n");
}

/* ----------------------------------------------------------------------- */
/* Main                                                                    */
/* ----------------------------------------------------------------------- */

int main(void)
{
    printf("=== Triangulation Unit Tests ===\n\n");

    test_rssi_to_distance();
    test_kalman_filter();
    test_trilateration_perfect();
    test_trilateration_direct_distance();
    test_weighted_channel_sounding_priority();
    test_innovation_gate();

    printf("\nAll tests passed.\n");
    return 0;
}
