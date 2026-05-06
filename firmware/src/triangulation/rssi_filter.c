#include "triangulation/rssi_filter.h"

void rssi_filter_init(rssi_filter_t *f, float q, float r, float initial)
{
    f->x = initial;
    f->p = r;       /* Start with covariance equal to measurement noise */
    f->q = q;
    f->r = r;
}

float rssi_filter_update(rssi_filter_t *f, float rssi)
{
    /* Predict */
    float x_pred = f->x;
    float p_pred = f->p + f->q;

    /* Update */
    float k = p_pred / (p_pred + f->r);   /* Kalman gain */
    f->x = x_pred + k * (rssi - x_pred);
    f->p = (1.0f - k) * p_pred;

    return f->x;
}

float rssi_filter_get(const rssi_filter_t *f)
{
    return f->x;
}
