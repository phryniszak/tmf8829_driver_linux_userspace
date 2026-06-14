/*
 * tmf8829_handpose.c - TMF8829 -> hand-pose CNN glue.
 *
 * Extracts a 48x32 depth map (metres) from the parsed pixel results using each
 * zone's Cartesian z (peaks[0].z, mm, already computed by the parser via
 * tmf8829CalcPixelXYZ), maps it into the model's pixel layout (with optional
 * orientation flips for sensor<->sim calibration), runs the CNN, and emits a
 * tagged JSON line so it never collides with the existing depth --stream output.
 */
#ifdef ENABLE_HANDPOSE

#include "tmf8829_handpose.h"
#include "tmf8829_frameparser.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* Build the model-layout depth map (metres) from parsed pixel results. */
static void build_depth(tmf8829_chip *chip, float depth_m[HANDPOSE_PIXELS])
{
    handpose_t *hp = &chip->handpose;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    int rows = parser->frame.numRows;   /* expect 32 for 48x32 mode */
    int cols = parser->frame.numCols;   /* expect 48 */

    for (int i = 0; i < HANDPOSE_PIXELS; i++)
        depth_m[i] = hp->invalid_fill;

    if (rows <= 0 || cols <= 0) return;

    for (int r = 0; r < HANDPOSE_H; r++) {
        for (int c = 0; c < HANDPOSE_W; c++) {
            /* Map model (r,c) -> sensor (sr,sc) with optional flips/transpose. */
            int mr = r, mc = c;
            if (hp->transpose) { int t = mr; mr = mc; mc = t; }
            int sr = hp->flip_y ? (rows - 1 - mr) : mr;
            int sc = hp->flip_x ? (cols - 1 - mc) : mc;
            if (sr < 0 || sr >= rows || sc < 0 || sc >= cols) continue;

            const tmf8829PixelResult_t *px = &parser->pixelResults[sr * cols + sc];
            if (px->numPeaks == 0) continue;
            const tmf8829Peak_t *pk = &px->peaks[0];
            if (pk->snr < hp->snr_min) continue;
            float z_mm = pk->z;                 /* Cartesian depth, mm */
            if (!(z_mm > 0.0f)) continue;       /* invalid / behind sensor */
            depth_m[r * HANDPOSE_W + c] = z_mm * 0.001f;   /* -> metres */
        }
    }
}

static void record_frame(handpose_t *hp, const float depth_m[HANDPOSE_PIXELS])
{
    if (hp->record_dir[0] == '\0') return;
    char path[320];
    snprintf(path, sizeof(path), "%s/%06ld.bin", hp->record_dir, hp->frame_count);
    FILE *f = fopen(path, "wb");
    if (f) { fwrite(depth_m, sizeof(float), HANDPOSE_PIXELS, f); fclose(f); }
}

void tmf8829_handpose_on_frame(tmf8829_chip *chip)
{
    handpose_t *hp = &chip->handpose;
    if (!hp->loaded) return;

    float depth_m[HANDPOSE_PIXELS];
    build_depth(chip, depth_m);

    float joints[HANDPOSE_MAX_JOINTS];
    handpose_run(hp, depth_m, joints);

    /* Tagged JSON line on stdout: "HP {...}" — distinct from depth --stream. */
    printf("HP {\"frame\":%lu,\"joints_deg\":[", (unsigned long)chip->frameParser.frame.frameNumber);
    for (int k = 0; k < hp->n_joints; k++)
        printf("%s%.2f", k ? "," : "", joints[k] * 57.2957795f);
    printf("],\"names\":[");
    for (int k = 0; k < hp->n_joints; k++)
        printf("%s\"%s\"", k ? "," : "", hp->names[k]);
    printf("]}\n");
    fflush(stdout);

    record_frame(hp, depth_m);
    hp->frame_count++;
}

#endif /* ENABLE_HANDPOSE */
