/*
 * handpose.h - dependency-free hand-pose CNN inference (C99).
 *
 * Runs the model trained in the cpp_hand_pose project: a 48x32 ToF depth map
 * (metres) -> hand joint angles (radians). Loads the same self-describing
 * weights file produced by python/export_weights.py ("NNH1") and the joint
 * ranges from meta.txt. No external dependencies (libm only).
 *
 * This module is sensor-agnostic and host-testable; the TMF8829 glue lives in
 * tmf8829_handpose.c.
 */
#ifndef HANDPOSE_INFER_H
#define HANDPOSE_INFER_H

#include <stdint.h>

#define HANDPOSE_W           48
#define HANDPOSE_H           32
#define HANDPOSE_PIXELS      (HANDPOSE_W * HANDPOSE_H)
#define HANDPOSE_MAX_JOINTS  32
#define HANDPOSE_INVALID_FILL 1.0f   /* must match python/data.py */

typedef struct handpose
{
    /* Model */
    int   n_joints;
    char  names[HANDPOSE_MAX_JOINTS][32];
    float lo[HANDPOSE_MAX_JOINTS];
    float hi[HANDPOSE_MAX_JOINTS];
    /* 6 layers: conv0..conv3, dense0, dense1; each a weight + bias array */
    float *w[6];   int wdim[6][4]; int wnd[6];
    float *b[6];   int bn[6];
    int    loaded;

    /* Runtime preprocessing config (set by the caller / CLI) */
    int   flip_x;        /* mirror columns  */
    int   flip_y;        /* mirror rows     */
    int   transpose;     /* swap rows/cols (only if dims match) */
    int   snr_min;       /* reject peaks below this SNR */
    float invalid_fill;  /* metres for invalid/no-return pixels */
    long  frame_count;   /* processed frames (for record naming) */
    char  record_dir[256]; /* "" = disabled; dump depth frames here */
} handpose_t;

/* Load weights + meta. Returns 0 on success, <0 on error. */
int  handpose_init(handpose_t *hp, const char *weights_path, const char *meta_path);
void handpose_free(handpose_t *hp);

/* Run inference. depth_m: HANDPOSE_PIXELS metres, row-major in MODEL layout
 * (row 0 first). NaN/<=0 are treated as invalid by the caller before this.
 * out_rad: at least n_joints floats (radians). */
void handpose_run(const handpose_t *hp, const float *depth_m, float *out_rad);

#endif /* HANDPOSE_INFER_H */
