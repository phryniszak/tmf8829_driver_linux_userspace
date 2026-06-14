/*
 * handpose_test.c - host parity test for the C inferencer.
 *
 * Reads a depth .bin (float32[1536], metres, model layout) produced by the
 * cpp_hand_pose dataset and prints predicted joint angles. Used to confirm the
 * C port matches the reference desktop nn_test numerically.
 *
 *   handpose_test <weights> <meta.txt> <depth.bin>
 */
#include "../handpose.h"

#include <stdio.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "usage: %s weights meta.txt depth.bin\n", argv[0]);
        return 1;
    }
    handpose_t hp;
    if (handpose_init(&hp, argv[1], argv[2]) != 0) return 1;

    float depth[HANDPOSE_PIXELS];
    FILE *f = fopen(argv[3], "rb");
    if (!f) { fprintf(stderr, "cannot open %s\n", argv[3]); return 1; }
    if (fread(depth, sizeof(float), HANDPOSE_PIXELS, f) != HANDPOSE_PIXELS) {
        fprintf(stderr, "short read\n"); fclose(f); return 1;
    }
    fclose(f);

    /* Match nn_infer: NaN/<=0 -> invalid fill (dataset bins are already clean). */
    for (int i = 0; i < HANDPOSE_PIXELS; i++)
        if (!(depth[i] == depth[i])) depth[i] = hp.invalid_fill;

    float out[HANDPOSE_MAX_JOINTS];
    handpose_run(&hp, depth, out);
    for (int k = 0; k < hp.n_joints; k++)
        printf("%-18s %8.4f rad  %7.2f deg\n",
               hp.names[k], out[k], out[k] * 57.2957795f);

    handpose_free(&hp);
    return 0;
}
