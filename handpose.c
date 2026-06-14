/*
 * handpose.c - dependency-free hand-pose CNN inference (C99 port of
 * cpp_hand_pose/src/nn_infer.hpp). Architecture (python/train.py:build_model):
 *
 *   Input(32,48,1)
 *   Conv2D(16,3,same,relu) -> Conv2D(32,3,same,relu) -> MaxPool2
 *   Conv2D(64,3,same,relu) -> MaxPool2 -> Conv2D(64,3,same,relu)
 *   Flatten -> Dense(256,relu) -> Dense(n_joints,sigmoid)
 *
 * Tensors are NHWC (row-major h,w,c) to match Keras Flatten ordering.
 */
#include "handpose.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NNH1_MAGIC 0x4E4E4831

/* ---- weights file -------------------------------------------------------- */

int handpose_init(handpose_t *hp, const char *weights_path, const char *meta_path)
{
    memset(hp, 0, sizeof(*hp));
    hp->invalid_fill = HANDPOSE_INVALID_FILL;
    hp->snr_min = 0;

    /* meta.txt: "<name> <lo> <hi>" lines (skip n_joints/depth_w headers). */
    FILE *mf = fopen(meta_path, "r");
    if (!mf) { fprintf(stderr, "handpose: cannot open meta %s\n", meta_path); return -1; }
    char line[256];
    hp->n_joints = 0;
    while (fgets(line, sizeof(line), mf)) {
        char nm[32]; float lo, hi;
        if (sscanf(line, "%31s %f %f", nm, &lo, &hi) == 3) {
            if (strcmp(nm, "n_joints") == 0 || strcmp(nm, "depth_w") == 0) continue;
            if (hp->n_joints >= HANDPOSE_MAX_JOINTS) break;
            snprintf(hp->names[hp->n_joints], sizeof(hp->names[0]), "%s", nm);
            hp->lo[hp->n_joints] = lo;
            hp->hi[hp->n_joints] = hi;
            hp->n_joints++;
        }
    }
    fclose(mf);
    if (hp->n_joints == 0) { fprintf(stderr, "handpose: no joints in meta\n"); return -1; }

    /* weights: int32 magic, int32 n_arrays, then per array: int32 ndim,
     * ndim int32 dims, prod floats. 12 arrays = 6 (weight,bias) pairs. */
    FILE *wf = fopen(weights_path, "rb");
    if (!wf) { fprintf(stderr, "handpose: cannot open weights %s\n", weights_path); return -1; }
    int32_t magic = 0, narr = 0;
    if (fread(&magic, 4, 1, wf) != 1 || fread(&narr, 4, 1, wf) != 1 ||
        magic != NNH1_MAGIC || narr != 12) {
        fprintf(stderr, "handpose: bad weights header\n"); fclose(wf); return -1;
    }
    for (int a = 0; a < 12; a++) {
        int32_t nd = 0;
        if (fread(&nd, 4, 1, wf) != 1 || nd < 1 || nd > 4) { fclose(wf); return -1; }
        int dims[4] = {1, 1, 1, 1};
        size_t cnt = 1;
        for (int k = 0; k < nd; k++) {
            int32_t dv = 0;
            if (fread(&dv, 4, 1, wf) != 1) { fclose(wf); return -1; }
            dims[k] = dv; cnt *= (size_t)dv;
        }
        float *buf = (float *)malloc(cnt * sizeof(float));
        if (!buf || fread(buf, sizeof(float), cnt, wf) != cnt) {
            free(buf); fclose(wf); fprintf(stderr, "handpose: weight read error\n"); return -1;
        }
        int layer = a / 2;
        if (a % 2 == 0) {  /* weight/kernel array */
            hp->w[layer] = buf; hp->wnd[layer] = nd;
            for (int k = 0; k < 4; k++) hp->wdim[layer][k] = dims[k];
        } else {           /* bias array */
            hp->b[layer] = buf; hp->bn[layer] = dims[0];
        }
    }
    fclose(wf);
    hp->loaded = 1;
    return 0;
}

void handpose_free(handpose_t *hp)
{
    for (int i = 0; i < 6; i++) { free(hp->w[i]); free(hp->b[i]); hp->w[i] = hp->b[i] = NULL; }
    hp->loaded = 0;
}

/* ---- layer ops ----------------------------------------------------------- */

/* Conv2D 3x3 stride1 SAME + ReLU. in: NHWC[h*w*ci]; K dims (kh,kw,ci,co). */
static void conv_relu(const float *in, int h, int w, int ci,
                      const float *K, const int kd[4], const float *bias,
                      float *out /* h*w*co */)
{
    int kh = kd[0], kw = kd[1], co = kd[3];
    int ph = kh / 2, pw = kw / 2;
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            for (int oc = 0; oc < co; oc++) {
                float acc = bias[oc];
                for (int ky = 0; ky < kh; ky++) {
                    int iy = y + ky - ph;
                    if (iy < 0 || iy >= h) continue;
                    for (int kx = 0; kx < kw; kx++) {
                        int ix = x + kx - pw;
                        if (ix < 0 || ix >= w) continue;
                        const float *ip = in + ((size_t)iy * w + ix) * ci;
                        for (int c = 0; c < ci; c++)
                            acc += ip[c] * K[(((size_t)ky * kw + kx) * ci + c) * co + oc];
                    }
                }
                out[((size_t)y * w + x) * co + oc] = acc > 0.0f ? acc : 0.0f;
            }
}

/* MaxPool 2x2 stride2 (floor), NHWC. */
static void maxpool2(const float *in, int h, int w, int c, float *out)
{
    int oh = h / 2, ow = w / 2;
    for (int y = 0; y < oh; y++)
        for (int x = 0; x < ow; x++)
            for (int ch = 0; ch < c; ch++) {
                float m = -1e30f;
                for (int dy = 0; dy < 2; dy++)
                    for (int dx = 0; dx < 2; dx++) {
                        float v = in[(((size_t)(2*y+dy)) * w + (2*x+dx)) * c + ch];
                        if (v > m) m = v;
                    }
                out[((size_t)y * ow + x) * c + ch] = m;
            }
}

/* Dense: y = relu?(x.W + b). W dims (ni,no). */
static void dense(const float *x, int ni, const float *W, const float *b,
                  int no, int relu, float *y)
{
    for (int o = 0; o < no; o++) {
        float acc = b[o];
        for (int i = 0; i < ni; i++) acc += x[i] * W[(size_t)i * no + o];
        y[o] = (relu && acc < 0.0f) ? 0.0f : acc;
    }
}

void handpose_run(const handpose_t *hp, const float *depth_m, float *out_rad)
{
    /* Conv stack. Buffers sized to the largest intermediate (32*48*64). */
    static float ba[HANDPOSE_H * HANDPOSE_W * 64];
    static float bb[HANDPOSE_H * HANDPOSE_W * 64];

    int h = HANDPOSE_H, w = HANDPOSE_W;
    /* conv0: 1->16 */
    conv_relu(depth_m, h, w, 1, hp->w[0], hp->wdim[0], hp->b[0], ba);
    /* conv1: 16->32 */
    conv_relu(ba, h, w, 16, hp->w[1], hp->wdim[1], hp->b[1], bb);
    /* pool -> 16x24 */
    maxpool2(bb, h, w, 32, ba); h /= 2; w /= 2;
    /* conv2: 32->64 */
    conv_relu(ba, h, w, 32, hp->w[2], hp->wdim[2], hp->b[2], bb);
    /* pool -> 8x12 */
    maxpool2(bb, h, w, 64, ba); h /= 2; w /= 2;
    /* conv3: 64->64 */
    conv_relu(ba, h, w, 64, hp->w[3], hp->wdim[3], hp->b[3], bb);

    /* Flatten (h*w*64) -> Dense(256) relu -> Dense(n) sigmoid. */
    static float d1[256];
    int flat = h * w * 64;                 /* 8*12*64 = 6144 */
    dense(bb, flat, hp->w[4], hp->b[4], 256, 1, d1);

    float o[HANDPOSE_MAX_JOINTS];
    dense(d1, 256, hp->w[5], hp->b[5], hp->n_joints, 0, o);

    for (int k = 0; k < hp->n_joints; k++) {
        float s = 1.0f / (1.0f + expf(-o[k]));            /* sigmoid */
        out_rad[k] = s * (hp->hi[k] - hp->lo[k]) + hp->lo[k]; /* denormalise */
    }
}
