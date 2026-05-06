#include "triangulation/triangulation.h"

#include <math.h>
#include <string.h>

#define DISTANCE_MIN_M  0.1f
#define DISTANCE_MAX_M  30.0f
#define TRI_MAX_INNOVATION_M          5.0f
#define TRI_OUTLIER_SIGMA_SCALE       3.0f
#define TRI_OUTLIER_ABS_MAX_M         3.0f

#define GN_MAX_ITER   15
#define GN_EPSILON    0.001f

/* --------------------------------------------------------------------------
 * Initialization
 * ----------------------------------------------------------------------- */

void triangulation_init(triangulation_ctx_t *ctx)
{
    memset(ctx, 0, sizeof(*ctx));

    for (int i = 0; i < NUM_ANTENNAS; i++) {
        rssi_filter_init(&ctx->filters[i], KALMAN_Q, KALMAN_R,
                         (float)RSSI_REF_DBM);
        ctx->distance_sigma_m[i] = RSSI_FALLBACK_SIGMA_M;
        ctx->distance_valid[i] = false;
    }

    ctx->position_valid = false;
}

/* --------------------------------------------------------------------------
 * RSSI -> Distance
 * ----------------------------------------------------------------------- */

float rssi_to_distance(float rssi_dbm, float rssi_ref_dbm,
                       float path_loss_exp)
{
    float exponent = (rssi_ref_dbm - rssi_dbm) / (10.0f * path_loss_exp);
    float distance = powf(10.0f, exponent);

    if (distance < DISTANCE_MIN_M) {
        distance = DISTANCE_MIN_M;
    }
    if (distance > DISTANCE_MAX_M) {
        distance = DISTANCE_MAX_M;
    }

    return distance;
}

/* --------------------------------------------------------------------------
 * Feed measurements
 * ----------------------------------------------------------------------- */

void triangulation_feed_distance(triangulation_ctx_t *ctx,
                                 uint8_t antenna_id,
                                 float distance_m,
                                 float sigma_m,
                                 enum triangulation_measurement_source source)
{
    (void)source;

    if (antenna_id >= NUM_ANTENNAS || !isfinite(distance_m) || !isfinite(sigma_m)) {
        return;
    }

    if (distance_m < DISTANCE_MIN_M || distance_m > DISTANCE_MAX_M) {
        return;
    }

    if (sigma_m < 0.01f) {
        sigma_m = 0.01f;
    }

    ctx->distances[antenna_id] = distance_m;
    ctx->distance_sigma_m[antenna_id] = sigma_m;
    ctx->distance_valid[antenna_id] = true;
}

void triangulation_feed_rssi(triangulation_ctx_t *ctx,
                             uint8_t antenna_id, int8_t rssi_dbm)
{
    if (antenna_id >= NUM_ANTENNAS) {
        return;
    }

    rssi_filter_update(&ctx->filters[antenna_id], (float)rssi_dbm);

    float filtered = rssi_filter_get(&ctx->filters[antenna_id]);
    float distance = rssi_to_distance(filtered,
                                      antenna_positions[antenna_id].rssi_ref,
                                      antenna_positions[antenna_id].path_loss_exp);

    triangulation_feed_distance(ctx,
                                antenna_id,
                                distance,
                                RSSI_FALLBACK_SIGMA_M,
                                TRI_MEAS_SOURCE_RSSI_FALLBACK);
}

/* --------------------------------------------------------------------------
 * Weighted 2D Trilateration: Nonlinear Least-Squares (Gauss-Newton)
 * ----------------------------------------------------------------------- */

static bool trilaterate_weighted_least_squares(const antenna_pos_t *antennas,
                                               const float *distances,
                                               const float *sigmas,
                                               const bool *valid,
                                               int n,
                                               const position_t *last_pos,
                                               bool has_last_pos,
                                               position_t *pos)
{
    float x = 0.0f, y = 0.0f;
    int used_for_init = 0;

    for (int i = 0; i < n; i++) {
        if (!valid[i]) {
            continue;
        }
        x += antennas[i].x;
        y += antennas[i].y;
        used_for_init++;
    }

    if (used_for_init < 3) {
        return false;
    }

    x /= used_for_init;
    y /= used_for_init;

    if (has_last_pos) {
        x = last_pos->x;
        y = last_pos->y;
    }

    for (int iter = 0; iter < GN_MAX_ITER; iter++) {
        float jtj00 = 0, jtj01 = 0, jtj11 = 0;
        float jtr0 = 0, jtr1 = 0;
        int used = 0;

        for (int i = 0; i < n; i++) {
            if (!valid[i]) {
                continue;
            }

            if (distances[i] < DISTANCE_MIN_M || distances[i] > DISTANCE_MAX_M) {
                continue;
            }

            float dx = x - antennas[i].x;
            float dy = y - antennas[i].y;
            float dist_est = sqrtf(dx * dx + dy * dy);
            if (dist_est < 1e-6f) {
                dist_est = 1e-6f;
            }

            float sigma = sigmas[i];
            if (sigma < 0.01f) {
                sigma = 0.01f;
            }
            float r_i = dist_est - distances[i];

            float outlier_gate = fmaxf(TRI_OUTLIER_SIGMA_SCALE * sigma, TRI_OUTLIER_ABS_MAX_M);
            if (fabsf(r_i) > outlier_gate) {
                continue;
            }

            float w = 1.0f / (sigma * sigma);
            float j0 = dx / dist_est;
            float j1 = dy / dist_est;

            jtj00 += w * j0 * j0;
            jtj01 += w * j0 * j1;
            jtj11 += w * j1 * j1;

            jtr0 += w * j0 * r_i;
            jtr1 += w * j1 * r_i;
            used++;
        }

        if (used < 3) {
            return false;
        }

        float det = jtj00 * jtj11 - jtj01 * jtj01;
        if (fabsf(det) < 1e-12f) {
            return false;
        }

        float dx_step = -(jtj11 * jtr0 - jtj01 * jtr1) / det;
        float dy_step = -(jtj00 * jtr1 - jtj01 * jtr0) / det;

        x += dx_step;
        y += dy_step;

        if (sqrtf(dx_step * dx_step + dy_step * dy_step) < GN_EPSILON) {
            break;
        }
    }

    if (has_last_pos) {
        float innov_x = x - last_pos->x;
        float innov_y = y - last_pos->y;
        float innovation = sqrtf(innov_x * innov_x + innov_y * innov_y);
        if (innovation > TRI_MAX_INNOVATION_M) {
            return false;
        }
    }

    pos->x = x;
    pos->y = y;
    return true;
}

/* --------------------------------------------------------------------------
 * Public solver
 * ----------------------------------------------------------------------- */

bool triangulation_solve(triangulation_ctx_t *ctx, position_t *pos)
{
    bool ok = trilaterate_weighted_least_squares(antenna_positions,
                                                 ctx->distances,
                                                 ctx->distance_sigma_m,
                                                 ctx->distance_valid,
                                                 NUM_ANTENNAS,
                                        &ctx->position,
                                        ctx->position_valid,
                                        pos);
    if (ok) {
        ctx->position = *pos;
        ctx->position_valid = true;
    }
    return ok;
}
