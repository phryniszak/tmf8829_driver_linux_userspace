/*
 * tmf8829_handpose.h - glue between the TMF8829 frame parser and the
 * dependency-free hand-pose CNN (handpose.c).
 *
 * All hand-pose code is additive and compiled only when ENABLE_HANDPOSE is
 * defined; with the feature off the driver is unchanged.
 */
#ifndef TMF8829_HANDPOSE_H
#define TMF8829_HANDPOSE_H

#ifdef ENABLE_HANDPOSE

#include "tmf8829_driver.h"

/* Called once per complete result frame (from handleReceivedResultDataEnd) when
 * chip->handpose_enabled is set. Builds a 48x32 depth map from the parsed pixel
 * results, runs the CNN, emits a tagged JSON line ("HP {...}") to stdout, and
 * optionally records the depth frame. Never touches existing outputs. */
void tmf8829_handpose_on_frame(tmf8829_chip *chip);

#endif /* ENABLE_HANDPOSE */
#endif /* TMF8829_HANDPOSE_H */
