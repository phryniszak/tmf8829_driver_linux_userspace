/*
 ************************************************************************************
 * Copyright (c) [2025] ams-OSRAM AG                                                *
 *                                                                                  *
 * SPDX-License-Identifier: GPL-2.0 OR MIT                                          *
 *                                                                                  *
 * For the full license texts, see LICENSES-GPL-2.0.txt or LICENSES-MIT.TXT.        *
 ************************************************************************************
*/

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <time.h>
#include "tmf8829.h"
#include "tmf8829_driver.h"
#include "tmf8829_frameparser.h"

#ifdef ENABLE_JSON_LOGGING
#include "tmf8829_json.h"
#endif
#ifdef ENABLE_KEYSTONE
#include "tmf8829_keystone.h"
#endif
#ifdef ENABLE_HANDPOSE
#include "tmf8829_handpose.h"
#endif

/* Parser states */
#define PARSER_STATE_IDLE           0
#define PARSER_STATE_HEADER         1
#define PARSER_STATE_DATA           2
#define PARSER_STATE_COMPLETE       3

/* ============================================================================
 * Frame Parser Core Functions
 * ============================================================================ */

void tmf8829FrameParserInit(tmf8829FrameParser_t *parser)
{
    memset(parser, 0, sizeof(tmf8829FrameParser_t));
    parser->state = PARSER_STATE_IDLE;
    parser->frame.pixelResults = parser->pixelResults;
    parser->lastFrameNumber = 0;
    parser->dualMode = 0;  /* Default: dual mode disabled */
    parser->dualCurrentPhase = 0; /* Start with HA phase */
    parser->dualHistoCount = 0;

    /* Initialize FPS statistics */
    parser->fpsFirstSystick = 0;
    parser->fpsLastSystick = 0;
    parser->fpsFrameCount = 0;

    /* Pre-allocate result buffer for maximum possible size */
    int maxResultSize = TMF8829_MAX_ROWS * TMF8829_MAX_COLS * 15 + TMF8829_FRAME_FOOTER_SIZE;
    parser->resultBuffer = (uint8_t *)malloc(maxResultSize);
    if (parser->resultBuffer != NULL)
    {
        parser->resultBufferAllocated = maxResultSize;
    }
    
#if ENABLE_HISTOGRAM
    parser->histograms = NULL;
    parser->histogramsHA = NULL;
    parser->histogramsAllocated = 0;
    parser->histogramsHAAllocated = 0;
    parser->histogramBuffer = NULL;
    parser->histogramBufferAllocated = 0;
    parser->histogramDataLen = 0;
#endif
}

void tmf8829FrameParserCleanup(tmf8829FrameParser_t *parser)
{
    /* Free result buffer */
    if (parser->resultBuffer != NULL)
    {
        free(parser->resultBuffer);
        parser->resultBuffer = NULL;
        parser->resultBufferAllocated = 0;
    }
    
#if ENABLE_HISTOGRAM
    if (parser->histograms != NULL)
    {
        free(parser->histograms);
        parser->histograms = NULL;
        parser->histogramsAllocated = 0;
    }
    if (parser->histogramsHA != NULL)
    {
        free(parser->histogramsHA);
        parser->histogramsHA = NULL;
        parser->histogramsHAAllocated = 0;
    }
    if (parser->histogramBuffer != NULL)
    {
        free(parser->histogramBuffer);
        parser->histogramBuffer = NULL;
        parser->histogramBufferAllocated = 0;
    }
#endif
}

#if ENABLE_HISTOGRAM
static int tmf8829AllocateHistograms(tmf8829FrameParser_t *parser, int count)
{
    if (parser->histograms != NULL && parser->histogramsAllocated >= count)
    {
        return 0;  /* Already allocated enough */
    }
    
    /* Free old allocation if any */
    if (parser->histograms != NULL)
    {
        free(parser->histograms);
    }
    
    parser->histograms = (tmf8829Histogram_t *)malloc(
        sizeof(tmf8829Histogram_t) * count);
    if (parser->histograms == NULL)
    {
        PRINT_INFO("Error: Failed to allocate histogram storage\n");
        return -1;
    }
    
    memset(parser->histograms, 0, sizeof(tmf8829Histogram_t) * count);
    parser->histogramsAllocated = count;
    return 0;
}

static int tmf8829AllocateHistogramsHA(tmf8829FrameParser_t *parser, int count)
{
    if (parser->histogramsHA != NULL && parser->histogramsHAAllocated >= count)
    {
        return 0;  /* Already allocated enough */
    }
    
    /* Free old allocation if any */
    if (parser->histogramsHA != NULL)
    {
        free(parser->histogramsHA);
    }
    
    parser->histogramsHA = (tmf8829Histogram_t *)malloc(
        sizeof(tmf8829Histogram_t) * count);
    if (parser->histogramsHA == NULL)
    {
        PRINT_INFO("Failed to allocate HA histogram storage\n");
        return -1;
    }
    
    memset(parser->histogramsHA, 0, sizeof(tmf8829Histogram_t) * count);
    parser->histogramsHAAllocated = count;
    return 0;
}
#endif /* ENABLE_HISTOGRAM */

void tmf8829GetResolution(int fpMode, int *rows, int *cols)
{
    switch (fpMode)
    {
        case TMF8829_CFG_FP_MODE_48x32:
            *rows = 32;  /* Y direction */
            *cols = 48;  /* X direction */
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            *rows = 32;
            *cols = 32;
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            *rows = 16;
            *cols = 16;
            break;
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            *rows = 8;
            *cols = 8;
            break;
    }
}

/**
 * @brief Get the number of histogram sub-frames for a given mode
 * @param fpMode The fpga mode
 * @return Number of histogram sub-frames
 */
/**
 * Get the number of histogram frames between two result frames
 * Returns the total number of histogram frames before a result frame
 *
 * For high-resolution modes (32x32, 48x32):
 *   - Non-dual: result sub-frames alternate, each preceded by histograms
 *   - Dual: each result sub-frame is preceded by histograms (HA + Default)
 *
 * For low-resolution modes (8x8, 16x16):
 *   - Single result frame preceded by histograms
 *
 * Return value:
 *   - High-res dual: histograms per result sub-frame cycle (e.g., 32x32 dual = 8)
 *   - High-res non-dual: histograms per result sub-frame (e.g., 32x32 = 4)
 *   - Low-res dual: histograms per result (e.g., 8x8 dual = 4)
 *   - Low-res non-dual: histograms per result (e.g., 8x8 = 2)
 */
int tmf8829GetHistogramsPerResult(int fpMode, int dualMode)
{
    int baseHistograms;

    /* Base number of histograms based on focal plane mode */
    switch (fpMode)
    {
        case TMF8829_CFG_FP_MODE_48x32:
            baseHistograms = 6;   /* 6 histograms per result sub-frame */
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            baseHistograms = 4;   /* 4 histograms per result sub-frame */
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            baseHistograms = 2;   /* 2 histograms per result frame */
            break;
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            baseHistograms = 2;   /* 2 histograms per result frame */
            break;
    }

    /* Dual mode doubles the histogram count for low-resolution modes only
     * High-resolution dual mode already accounted for in baseHistograms
     */
    if (dualMode == 1 || dualMode == 2)
    {
        if (fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            /* High-resolution dual mode: each sub-frame has HA + Default
             * 32x32: 4 HA + 4 Default = 8 per result sub-frame cycle
             * 48x32: 6 HA + 6 Default = 12 per result sub-frame cycle
             */
            return baseHistograms * 2;
        }
        else
        {
            /* Low-resolution dual mode: all histograms in one sequence
             * 8x8/16x16: 2 HA + 2 Default = 4 per result
             */
            return baseHistograms * 2;
        }
    }

    return baseHistograms;
}

int tmf8829GetPixelDataSize(uint8_t resultFormat)
{
    int numPeak = resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeak == 0) numPeak = 1;
    
    int useSignal = (resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    int useNoise = (resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    int useXtalk = (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;
    
    /* Each peak: distance(2) + snr(1) + signal(2 if enabled) */
    /* Per pixel: noise(2 if enabled) + xtalk(2 if enabled) + peaks */
    int size = (numPeak * (3 + 2 * useSignal)) + (2 * useNoise) + (2 * useXtalk);
    return size;
}

/**
 * @brief Calculate 3D point cloud coordinates (X, Y, Z) from distance
 *
 * Converts the distance measurement to 3D point cloud coordinates.
 *   x_norm = (col - cols/2 + 0.5) / spanX - fov_correction_x
 *   y_norm = (row - rows/2 + 0.5) / spanY - fov_correction_y
 *   depth = distance / sqrt(1 + x_norm² + y_norm²)
 *   X = depth * x_norm
 *   Y = depth * y_norm
 *   Z = depth
 *
 * FOV correction (from TMF8829_CFG_INFO_FOV_CORR):
 *   - bits 0-1: X correction (coding in 16x16 sub-macropixel)
 *   - bits 2-3: Y correction
 *
 * Focal length correction:
 *   - The angle per unit x_norm is proportional to (SPAD_pitch / focal_length)
 *   - Scale x_norm and y_norm by the ratio of nominal to actual focal length
 *   - FOCAL_LENGTH_NOM = 560.0 (micrometers)
 *
 * @param col Column index (X direction, starting from 0)
 * @param row Row index (Y direction, starting from 0)
 * @param fp_mode Focal plane mode
 * @param distance Distance in mm
 * @param fovCorrection FOV correction value (bits 0-1: X, bits 2-3: Y)
 * @param focal_length Focal length in micrometers (default: 560.0)
 * @param outX Pointer to store X coordinate in mm
 * @param outY Pointer to store Y coordinate in mm
 * @param outZ Pointer to store Z coordinate (depth) in mm
 */
static void tmf8829CalcPixelXYZ(int col, int row, int fp_mode, float distance,
                                uint8_t fovCorrection, float focal_length,
                                float *outX, float *outY, float *outZ)
{
    float X, Y;
    float spanX, spanY;
    float xNorm, yNorm, depth, depthFactor;
    float fovCorrX, fovCorrY;
    float FOCAL_LENGTH_NOM = 560.0f;

    /* Extract FOV correction values from configuration
     * fovCorrection bits 0-1: X correction, bits 2-3: Y correction
     */
    uint8_t fovCorrectionX = fovCorrection & 0x03;
    uint8_t fovCorrectionY = (fovCorrection >> 2) & 0x03;

    /* Convert to normalized offset
     * Formula: (fovCorrection - 1.5) / 2.5 / 16
     * This maps 0-3 correction values to small offsets in normalized coordinates
     */
    fovCorrX = ((float)fovCorrectionX - 1.5f) / 2.5f / 16.0f;
    fovCorrY = ((float)fovCorrectionY - 1.5f) / 2.5f / 16.0f;

    /* Get resolution based on fp_mode */
    switch (fp_mode)
    {
        case TMF8829_CFG_FP_MODE_8x8A:
        case TMF8829_CFG_FP_MODE_8x8B:
        default:
            X = 8;
            Y = 8;
            break;
        case TMF8829_CFG_FP_MODE_16x16:
            X = 16;
            Y = 16;
            break;
        case TMF8829_CFG_FP_MODE_32x32:
        case TMF8829_CFG_FP_MODE_32x32s:
            X = 32;
            Y = 32;
            break;
        case TMF8829_CFG_FP_MODE_48x32:
            X = 48;
            Y = 32;
            break;
    }

    spanX = X * 3.0f / 4.0f;
    spanY = Y;

    /* Calculate normalized x/y coordinates (from center) with FOV correction */
    xNorm = ((float)col - X / 2.0f + 0.5f) / spanX - fovCorrX;
    yNorm = ((float)row - Y / 2.0f + 0.5f) / spanY - fovCorrY;

    /* Apply focal length correction
     * Scale xNorm and yNorm by the ratio of nominal to actual focal length
     * This correctly adjusts the FOV: shorter focal_length -> wider FOV
     */
    if (focal_length != FOCAL_LENGTH_NOM) {
        float scale = FOCAL_LENGTH_NOM / focal_length;
        xNorm *= scale;
        yNorm *= scale;
    }

    /* Calculate 3D coordinates */
    depthFactor = sqrtf(1.0f + xNorm * xNorm + yNorm * yNorm);
    depth = distance / depthFactor;

    *outX = depth * xNorm;
    *outY = depth * yNorm;
    *outZ = depth;
}

int tmf8829ParseFrameHeader(tmf8829FrameParser_t *parser, uint8_t *data)
{
    tmf8829FrameHeader_t *header;
    
    /* Parse pre-header */
    /* byte 0: FIFOSTATUS, bytes 1-4: SYSTICK (little endian) */
    parser->frame.systick = data[1] | (data[2] << 8) | (data[3] << 16) | ((uint32_t)data[4] << 24);
    
    /* Parse frame header */
    header = (tmf8829FrameHeader_t *)(data + TMF8829_PRE_HEADER_SIZE);
    
    parser->frame.frameType = header->id & TMF8829_FID_MASK;
    parser->frame.fpMode = header->id & TMF8829_FPM_MASK;
    parser->frame.layout = header->layout;  /* Store layout for later use */
    parser->frame.frameNumber = header->fNumber;
    parser->frame.temperature = header->temperature[2]; /* Use sensor 2 */
    
    /* Get resolution */
    tmf8829GetResolution(parser->frame.fpMode, &parser->frame.numRows, &parser->frame.numCols);
    
#if ENABLE_HISTOGRAM
    /* Reset histogram buffer for new frame */
    parser->histogramDataLen = 0;
#endif
    parser->dataLen = 0;
    
    parser->state = PARSER_STATE_HEADER;
    
    return 0;
}

int tmf8829ParseFrameFooter(tmf8829FrameParser_t *parser, uint8_t *data, int totalLen)
{
    /* Footer is at the end of the frame */
    int footerOffset = totalLen - TMF8829_FRAME_FOOTER_SIZE;
    
    if (footerOffset < TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE)
    {
        return -1;
    }
    
    tmf8829FrameFooter_t *footer = (tmf8829FrameFooter_t *)(data + footerOffset);
    
    parser->frame.t0Integration = footer->t0Integration;
    parser->frame.t1Integration = footer->t1Integration;
    parser->frame.frameStatus = footer->frameStatus;
    
    /* Extract warnings (all bits except valid bit) */
    parser->frame.warnings = footer->frameStatus & ~TMF8829_FRAME_VALID;
    
    parser->state = PARSER_STATE_COMPLETE;
    
    /* Print frame info */
    tmf8829PrintFrameInfo(parser);
    
    return 0;
}

int tmf8829ParseResultData(tmf8829FrameParser_t *parser, uint8_t *data, uint8_t resultFormat)
{
    int pixelDataSize;
    int numPeaks;
    int useSignal, useNoise, useXtalk;
    int row, col, i;
    int offset;
    uint8_t *pixelPtr;
    tmf8829PixelResult_t *pixel;
    float distance;
    int subFrameIdx = 0;
    int actualRows;
    int targetRow;
    uint16_t maxXtalk = 0;
    
    numPeaks = resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    if (numPeaks == 0) numPeaks = 1;
    
    useSignal = (resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    useNoise = (resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK) ? 1 : 0;
    useXtalk = (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK) ? 1 : 0;
    
    pixelDataSize = tmf8829GetPixelDataSize(resultFormat);
    
    /* For high-resolution result frames, determine sub-frame and actual rows */
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
        actualRows = parser->frame.numRows / 2;  /* Each sub-frame has half the rows */
    }
    else
    {
        actualRows = parser->frame.numRows;
        subFrameIdx = 0;
    }
    
    /* Start parsing after header */
    offset = TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE;
    
    /* Parse actual number of rows in this sub-frame */
    for (row = 0; row < actualRows; row++)
    {
        /* For high-res modes, interleave rows: sub-frame 0 -> even rows, sub-frame 1 -> odd rows */
        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            targetRow = row * 2 + subFrameIdx;  /* 0,2,4,... or 1,3,5,... */
        }
        else
        {
            targetRow = row;
        }
        
        for (col = 0; col < parser->frame.numCols; col++)
        {
            pixelPtr = data + offset;
            pixel = &parser->pixelResults[targetRow * parser->frame.numCols + col];
            
            /* Reset pixel result */
            pixel->noise = 0;
            pixel->xtalk = 0;
            pixel->numPeaks = numPeaks;
            
            int byteOffset = 0;
            
            /* Parse noise if present */
            if (useNoise)
            {
                pixel->noise = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                byteOffset += 2;
            }
            
            /* Parse crosstalk if present */
            if (useXtalk)
            {
                pixel->xtalk = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                byteOffset += 2;
            }
            
            /* Parse peaks */
            for (i = 0; i < numPeaks; i++)
            {
                /* Distance is in quarter-mm, convert to mm */
                uint16_t distQuarter = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                distance = distQuarter / 4.0f;
                pixel->peaks[i].distance = (int)distance;
                pixel->peaks[i].snr = pixelPtr[byteOffset + 2];
                byteOffset += 3;
                
                /* Signal strength if present */
                if (useSignal)
                {
                    pixel->peaks[i].signal = pixelPtr[byteOffset] | (pixelPtr[byteOffset + 1] << 8);
                    byteOffset += 2;
                }
                else
                {
                    pixel->peaks[i].signal = 0;
                }
                
                /* Calculate 3D point cloud coordinates (X, Y, Z in mm) */
                tmf8829CalcPixelXYZ(col, targetRow, parser->frame.fpMode, distance,
                                    parser->fovCorrection, 560,
                                    &pixel->peaks[i].x, &pixel->peaks[i].y, &pixel->peaks[i].z);
            }
            
            offset += pixelDataSize;

            /* Track max crosstalk for this frame */
            if (useXtalk && pixel->xtalk > maxXtalk)
            {
                maxXtalk = pixel->xtalk;
            }
        }
    }

    /* Store maxXtalk in parser for later percentage calculation */
    parser->maxXtalk = maxXtalk;

    return 0;
}

/* ============================================================================
 * Frame Handler Callbacks
 * ============================================================================ */

void handleReceivedFrameHeaderData(void *dptr, uint8_t *data)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    
    /* Parse header directly from tof_core.dataBuffer */
    tmf8829ParseFrameHeader(parser, data);
    
    /* Reset result data accumulation */
    parser->dataLen = 0;
    
    /* Copy header to result buffer */
    if (parser->resultBuffer != NULL)
    {
        memcpy(parser->resultBuffer, data, TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE);
        parser->dataLen = TMF8829_PRE_HEADER_SIZE + TMF8829_FRAME_HEADER_SIZE;
    }
}

void handleReceivedResultData(void *dptr, uint8_t *data, uint16_t size)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    
    /* Accumulate data in result buffer */
    if (parser->resultBuffer != NULL && 
        parser->dataLen + size <= parser->resultBufferAllocated)
    {
        memcpy(parser->resultBuffer + parser->dataLen, data, size);
        parser->dataLen += size;
    }
    else
    {
        PRINT_INFO("Warning: Result buffer overflow or not allocated, truncating\n");
    }
    
    parser->state = PARSER_STATE_DATA;
}

#if ENABLE_HISTOGRAM
int tmf8829ParseHistogramData(tmf8829FrameParser_t *parser, uint8_t *data, int subFrame)
{
    int row, col, bin;
    int offset;
    int numRows, numCols;
    int binSize;
    int numHistograms;
    tmf8829Histogram_t *histogram;
    int fovRows, fovCols;        /* FOV dimensions in histogram frame */
    int pixelRowsPerMp, pixelColsPerMp;  /* Scaling factors */
    int leftFovOffset, pixelRowOffset, pixelColumnOffset;
    int targetRow, targetCol;
    
    /* Get resolution */
    tmf8829GetResolution(parser->frame.fpMode, &numRows, &numCols);
    
    /* Note: isHAHistogram is now set in handleReceivedHistogramDataEnd before this function is called */
    
    /* Allocate histogram storage if needed */
    numHistograms = numRows * numCols;
    if (parser->isHAHistogram)
    {
        if (tmf8829AllocateHistogramsHA(parser, numHistograms) != 0)
        {
            return -1;
        }
    }
    else
    {
        if (tmf8829AllocateHistograms(parser, numHistograms) != 0)
        {
            return -1;
        }
    }
    
    /* Determine bin size based on mode */
    /* 8x8 mode uses 256 bins, other modes use 64 bins */
    if (numRows == 8 && numCols == 8)
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_8x8;
    }
    else
    {
        binSize = TMF8829_HISTOGRAM_BIN_SIZE_OTHER;
    }
    
    /* Data buffer contains FIFO data which starts after frame header */
    /* First 4 histograms are reference pixels with 64 bins each */
    offset = 0;
    
    /* Skip 4 reference pixel histograms (each 64 bins * 3 bytes = 192 bytes) */
    /* Reference pixels always have 64 bins */
    offset += 4 * 192;
    
    /* Calculate FOV dimensions and scaling factors based on mode */
    /* For 8x8: fovRows=8, fovCols=8, but frame has fovRows x (fovCols/2) = 8x4 histograms */
    /* For 16x16: fovRows=16, fovCols=16, frame has 16x8 histograms */
    /* For 32x32/48x32: fovRows=16, fovCols=16, frame has 16x8 histograms */
    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_8x8B)
    {
        fovRows = 8;
        fovCols = 8;  /* Full FOV width, but frame has only fovCols/2 columns */
    }
    else
    {
        fovRows = 16;
        fovCols = 16;
    }
    
    pixelRowsPerMp = numRows / fovRows;
    pixelColsPerMp = numCols / fovCols;
    
    /* Frame actually contains fovRows x (fovCols/2) histograms */
    int frameCols = fovCols / 2;  /* Number of columns in the frame data */
    
    /* Calculate position offsets based on layout (subFrame number) */
    leftFovOffset = 0;
    pixelRowOffset = 0;
    pixelColumnOffset = 0;
    
    /* Odd layout (1,3,5,7,...): right half of FOV */
    if (subFrame % 2 != 0)
    {
        leftFovOffset = numCols / 2;
    }
    
    if (parser->frame.fpMode == TMF8829_CFG_FP_MODE_48x32)
    {
        /* 48x32 mode: layout in [2,3,8,9] -> colOffset=1, [4,5,10,11] -> colOffset=2 */
        if (subFrame == 2 || subFrame == 3 || subFrame == 8 || subFrame == 9)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame == 4 || subFrame == 5 || subFrame == 10 || subFrame == 11)
        {
            pixelColumnOffset = 2;
        }
        if (subFrame > 5)
        {
            pixelRowOffset = 1;
        }
    }
    else if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* 32x32 mode: layout in [2,3,6,7] -> colOffset=1, layout > 3 -> rowOffset=1 */
        if (subFrame == 2 || subFrame == 3 || subFrame == 6 || subFrame == 7)
        {
            pixelColumnOffset = 1;
        }
        if (subFrame > 3)
        {
            pixelRowOffset = 1;
        }
    }
    
    // printf("Parse hist: layout=%d, fovRows=%d, frameCols=%d, leftFov=%d, rowOff=%d, colOff=%d\n",
    //        subFrame, fovRows, frameCols, leftFovOffset, pixelRowOffset, pixelColumnOffset);
    
    /* Parse histograms from frame data */
    /* Each frame contains fovRows x frameCols histograms */
    for (row = 0; row < fovRows; row++)
    {
        for (col = 0; col < frameCols; col++)
        {
            /* Calculate target position in full resolution grid */
            targetRow = row * pixelRowsPerMp + pixelRowOffset;
            targetCol = col * pixelColsPerMp + leftFovOffset + pixelColumnOffset;
            
            /* Bounds check */
            if (targetRow >= numRows || targetCol >= numCols)
            {
                PRINT_INFO("Warning: histogram index out of bounds (%d,%d)\n", targetRow, targetCol);
                /* Still need to skip the data */
                offset += binSize * 3;
                continue;
            }
            
            /* Select correct histogram buffer based on dual mode phase */
            if (parser->isHAHistogram && parser->dualMode != 0)
            {
                histogram = &parser->histogramsHA[targetRow * numCols + targetCol];
            }
            else
            {
                histogram = &parser->histograms[targetRow * numCols + targetCol];
            }
            
            /* Parse bins, each bin is 3 bytes (little endian) */
            for (bin = 0; bin < binSize; bin++)
            {
                histogram->bin[bin] = data[offset] | 
                                      (data[offset + 1] << 8) | 
                                      ((uint32_t)data[offset + 2] << 16);
                offset += 3;
            }
        }
    }
    
    //PRINT_DEBUG("Parse hist: layout=%d done, offset=%d, expectedPixels=%d, isHA=%d\n",
    //       subFrame, offset, fovRows * frameCols, parser->isHAHistogram);
    
    /* Mark histogram as available */
    parser->frame.hasHistogram = 1;
    parser->frame.histogramSubFrame = subFrame;
    /* Store correct histogram pointer based on mode */
    if (parser->isHAHistogram && parser->dualMode != 0)
    {
        parser->frame.histograms = parser->histogramsHA;
    }
    else
    {
        parser->frame.histograms = parser->histograms;
    }
    
    /* Reset dual histogram counter when we reach a result frame */
    /* This is handled in handleReceivedResultDataEnd */
    
    return 0;
}
#endif /* ENABLE_HISTOGRAM */

#if ENABLE_HISTOGRAM
void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    
    /* Allocate histogram buffer if needed */
        if (parser->histogramBuffer == NULL)
        {
            parser->histogramBuffer = (uint8_t *)malloc(TMF8829_HISTOGRAM_BUFFER_SIZE);
            if (parser->histogramBuffer == NULL)
            {
                PRINT_INFO("Failed to allocate histogram buffer\n");
                return;
            }
            parser->histogramBufferAllocated = TMF8829_HISTOGRAM_BUFFER_SIZE;
        }
    
    /* Accumulate histogram data into parser buffer */
    if (parser->histogramDataLen + size <= parser->histogramBufferAllocated)
    {
        memcpy(parser->histogramBuffer + parser->histogramDataLen, data, size);
        parser->histogramDataLen += size;
    }
    else
    {
        PRINT_INFO("Warning: Histogram buffer overflow, truncating\n");
    }
    
    parser->dataLen += size;
    parser->state = PARSER_STATE_DATA;
}
#else
void handleReceivedHistogramData(void *dptr, uint8_t *data, uint16_t size)
{
    /* Histogram disabled - do nothing */
    (void)dptr;
    (void)data;
    (void)size;
}
#endif /* ENABLE_HISTOGRAM */

static void tmf8829StreamFrameJson(tmf8829_chip *chip, uint8_t resultFormat);

void handleReceivedResultDataEnd(void *dptr)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    tmf8829Driver *driver = &chip->tof_core;
    uint8_t *data = parser->resultBuffer; /* Use accumulated result data */
    uint8_t resultFormat = driver->config[TMF8829_CFG_RESULT_FORMAT - TMF8829_CFG_PERIOD_MS_LSB];
    uint8_t dumpHistograms = driver->config[TMF8829_CFG_DUMP_HISTOGRAMS - TMF8829_CFG_PERIOD_MS_LSB];
    int subFrameIdx = 0;
    int expectedFrameDiff = 0;

    /* Parse footer and result data */
    tmf8829ParseFrameFooter(parser, data, parser->dataLen);
    
    /* Determine sub-frame index for high-resolution modes */
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
        
        /* Clear pixel results buffer at start of each result cycle (sub-frame 0 only) */
        /* This ensures clean data when sub-frame 1 arrives for printing */
        if (subFrameIdx == 0)
        {
            memset(parser->pixelResults, 0, sizeof(parser->pixelResults));
        }
    }
    
    tmf8829ParseResultData(parser, data, resultFormat);

    /* Calculate crosstalk percentage if xtalk data is available */
    parser->xtalkPercentage = 0.0f;
    if (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK)
    {
        int iteration = driver->config[TMF8829_CFG_KILO_ITERATIONS_LSB - TMF8829_CFG_PERIOD_MS_LSB] |
                        driver->config[TMF8829_CFG_KILO_ITERATIONS_MSB - TMF8829_CFG_PERIOD_MS_LSB] << 8;
        int iterationsDivider = 2; /* for 8x8 and 16x16 */

        if (parser->frame.fpMode >= TMF8829_CFG_FP_MODE_32x32)
        {
            iterationsDivider = 8;
        }

        if (parser->frame.fpMode >= TMF8829_CFG_FP_MODE_48x32)
        {
            iterationsDivider = 12;
        }

        if (iterationsDivider > 0)
        {
            int xtalkIteration = (iteration / iterationsDivider) * 1024;
            if (xtalkIteration > 0)
            {
                parser->xtalkPercentage = 100.0f * parser->maxXtalk / xtalkIteration;
            }
        }
    }
    
    /* Only high-resolution modes need sub-frame tracking */
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
        
        /* Determine expected frame number difference based on mode and histogram config */
        if (dumpHistograms)
        {
            /* With histograms:
             * Non-dual mode:
             *   - 32x32: sub-frame 0 -> 4 histograms -> sub-frame 1 -> 4 histograms -> sub-frame 0
             *            frameDiff = 4 histograms + 1 sub-frame = 5
             *   - 48x32: sub-frame 0 -> 6 histograms -> sub-frame 1 -> 6 histograms -> sub-frame 0
             *            frameDiff = 6 histograms + 1 sub-frame = 7
             * Dual mode (dualMode=1 or 2):
             *   - 32x32: sub-frame 0 -> 8 histograms (4 HA + 4 Default) -> sub-frame 1
             *            frameDiff = 8 histograms + 1 sub-frame = 9
             *   - 48x32: sub-frame 0 -> 12 histograms (6 HA + 6 Default) -> sub-frame 1
             *            frameDiff = 12 histograms + 1 sub-frame = 13
             */
            int histogramsPerSubframe = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            expectedFrameDiff = histogramsPerSubframe + 1;
        }
        else
        {
            /* Without histograms: result frames are consecutive, frameDiff = 1 */
            expectedFrameDiff = 1;
        }

        int frameDiff = parser->frame.frameNumber - parser->lastFrameNumber;
        
        /* Skip frame loss check for first frame (lastFrameNumber == 0) */
        int isFirstFrame = (parser->lastFrameNumber == 0);
        
        if (subFrameIdx == 0)
        {
            /* Start fresh with this sub-frame 0 as the first sub-frame of new cycle */
            parser->resultSubFrameReceived = (1 << subFrameIdx);

            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {
                /* Frame number discontinuity */
                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (fpMode=%d, expected frameDiff=%d, actual=%d)\n",
                       lostFrames, parser->frame.fpMode, expectedFrameDiff, frameDiff);
            }
        }
        else
        {
            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {
                /* Frame number discontinuity */
                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (expected frameDiff=%d, actual=%d)\n",
                       lostFrames, expectedFrameDiff, frameDiff);
            }
            else
            {
                parser->resultSubFrameReceived |= (1 << subFrameIdx);

                if (parser->resultSubFrameReceived == 0x3)
                {
                    /* Both sub-frames received correctly, mark result ready */
                    parser->resultReady = 1;

                    /* Update FPS statistics */
                    if (parser->fpsFirstSystick == 0)
                    {
                        /* First complete frame */
                        parser->fpsFirstSystick = parser->frame.systick;
                    }
                    parser->fpsLastSystick = parser->frame.systick;
                    parser->fpsFrameCount++;
                }
            }
        }

        parser->lastFrameNumber = parser->frame.frameNumber;
    }
    else
    {
        /* Low-resolution modes: single sub-frame, always ready */
        parser->resultReady = 1;

        /* Update FPS statistics for low-resolution modes */
        if (parser->fpsFirstSystick == 0)
        {
            /* First complete frame */
            parser->fpsFirstSystick = parser->frame.systick;
        }
        parser->fpsLastSystick = parser->frame.systick;
        parser->fpsFrameCount++;

        /* Frame loss detection for low-resolution modes */
        if (dumpHistograms)
        {
            /* With histograms:
             * Non-dual mode (8x8/16x16): T0 -> T1 -> Result
             *                         frameDiff = 2 histograms + 1 = 3
             * Dual mode (8x8/16x16): T0 -> T1 -> T0 -> T1 -> Result
             *                       frameDiff = 4 histograms + 1 = 5
             */
            int frameDiff = parser->frame.frameNumber - parser->lastFrameNumber;
            int isFirstFrame = (parser->lastFrameNumber == 0);
            int histogramsPerResult = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            int expectedFrameDiff = histogramsPerResult + 1;

            if (!isFirstFrame && frameDiff != expectedFrameDiff)
            {
                /* Frame number discontinuity */
                int lostFrames = frameDiff - expectedFrameDiff;
                PRINT_INFO("Warning: Frame loss detected! missed %d frames (expected frameDiff=%d, actual=%d)\n",
                       lostFrames, expectedFrameDiff, frameDiff);
            }
        }
    }

#ifdef ENABLE_JSON_LOGGING
    /* Write to JSON file only when result is complete */

#if ENABLE_HISTOGRAM
    /* For high-resolution modes, need both result and histograms ready ONLY if histogram dumping is enabled */
    int shouldWrite = 0;
    if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
    {
        /* High resolution: need resultReady, and histogram_ready only if histogram dumping is enabled */
        /* Read histogram_dump setting from driver config */
        int histogram_dump_enabled = chip->tof_core.config[TMF8829_CFG_DUMP_HISTOGRAMS - TMF8829_CFG_PERIOD_MS_LSB];

        if (histogram_dump_enabled)
        {
            /* Histogram dumping enabled: need both resultReady and histogram_ready */
            shouldWrite = (chip->jsonLogger.is_open && parser->resultReady && chip->jsonLogger.histogram_ready);
        }
        else
        {
            /* Histogram dumping disabled: only need resultReady */
            shouldWrite = (chip->jsonLogger.is_open && parser->resultReady);
        }
    }
    else
    {
        /* Low resolution: only need resultReady */
        shouldWrite = (chip->jsonLogger.is_open && parser->resultReady);
    }

    if (shouldWrite)
#else
    if (chip->jsonLogger.is_open && parser->resultReady)
#endif
    {
        tmf8829_json_write_frame(&chip->jsonLogger, parser, resultFormat);

#if ENABLE_HISTOGRAM
        /* Reset histogram tracking for next measurement cycle */
        chip->jsonLogger.histogram_subframes_received = 0;
        chip->jsonLogger.histogram_subframes_received_ha = 0;
        chip->jsonLogger.histogram_ready = 0;
        chip->jsonLogger.histogram_ha_ready = 0;
        chip->jsonLogger.histogram_subframe0_ready = 0;
        chip->jsonLogger.histogram_subframe1_ready = 0;
        chip->jsonLogger.histogram_subframe0_ready_ha = 0;
        chip->jsonLogger.histogram_subframe1_ready_ha = 0;
#endif
    }
#endif

    if (chip->stream_enabled && parser->resultReady)
        tmf8829StreamFrameJson(chip, resultFormat);

#ifdef ENABLE_KEYSTONE
    /* Perform keystone angle calculation if enabled */
    /* resultReady is only set to 1 when complete result frame is received:
     * - Low resolution (8x8, 16x16): single frame, resultReady=1 after each frame
     * - High resolution (32x32, 48x32): two sub-frames, resultReady=1 only after both received
     */
    if (parser->keystoneEnabled && parser->resultReady) {
        /* Get parent chip structure from parser offset */
        tmf8829_chip *chip = (tmf8829_chip *)((char*)parser - offsetof(tmf8829_chip, frameParser));
        
        /* Extract XYZ data from pixel results */
        float xData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        float yData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        float zData[TMF8829_MAX_ROWS * TMF8829_MAX_COLS];
        int row, col;
        
        for (row = 0; row < parser->frame.numRows; row++) {
            for (col = 0; col < parser->frame.numCols; col++) {
                int index = row * parser->frame.numCols + col;
                tmf8829Peak_t *peak = &parser->pixelResults[index].peaks[0];
                xData[index] = peak->x;
                yData[index] = peak->y;
                zData[index] = peak->z;
            }
        }
        
        /* Call keystone module to calculate angles */
        keystoneProcessFrame(&chip->keystoneCtx,
                             parser->frame.numRows,
                             parser->frame.numCols,
                             xData, yData, zData, NULL);
        
        /* Get calculated angles */
        keystoneGetAngles(&chip->keystoneCtx,
                         &parser->keystoneAngleX,
                         &parser->keystoneAngleY,
                         &parser->keystoneAngleZ);
    }
#endif

#ifdef ENABLE_HANDPOSE
    /* Hand-pose inference on the completed result frame (opt-in, additive). */
    if (chip->handpose_enabled && parser->resultReady) {
        tmf8829_handpose_on_frame(chip);
    }
#endif

    tmf8829PrintFrameResults(parser);

    /* Reset for next result cycle (always reset, regardless of JSON logging) */
    /* Save resultReady state before resetting */
    int wasResultReady = parser->resultReady;
    parser->resultReady = 0;
    /* For high-resolution modes, only reset sub-frame tracking when resultReady was set */
    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16 || wasResultReady)
    {
        parser->resultSubFrameReceived = 0;
    }

    /* Reset dual mode histogram counter for low resolution modes */
    if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16)
    {
        parser->dualHistoCount = 0;
    }
    /* High resolution modes: counter is reset within histogram handler */
}

#if ENABLE_HISTOGRAM
void handleReceivedHistogramDataEnd(void *dptr)
{
    tmf8829_chip *chip = (tmf8829_chip *)dptr;
    tmf8829FrameParser_t *parser = &chip->frameParser;
    uint8_t *data = parser->histogramBuffer;  /* Use accumulated histogram data */
    int subFrame;
    
    /* Sub-frame number is in the layout field (stored during header parsing) */
    subFrame = parser->frame.layout & 0x0F;
    
    /* Determine HA/Default status before parsing and printing */
    if (parser->dualMode != 0)
    {
        /* Dual mode logic based on actual hardware behavior:
         * - Low resolution (8x8, 16x16): All HA histograms first, then all Default histograms
         * - High resolution (32x32, 48x32):
         *      For each result sub-frame cycle:
         *         First: all HA histograms (same layout range)
         *         Then: all Default histograms (same layout range)
         *      For 48x32: HA layout 0-5 -> Default layout 0-5 -> Result sub0 -> HA layout 6-11 -> Default layout 6-11 -> Result sub1
         *      For 32x32: HA layout 0-3 -> Default layout 0-3 -> Result sub0 -> HA layout 4-7 -> Default layout 4-7 -> Result sub1
         */
        
        /* For low resolution modes */
        if (parser->frame.fpMode <= TMF8829_CFG_FP_MODE_16x16)
        {
            /* Simple counting: first N histograms are HA, next N are Default */
            int totalHistosPerResult = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);
            int histosPerMode = totalHistosPerResult / 2;
            
            if (parser->dualHistoCount < histosPerMode)
            {
                parser->isHAHistogram = 1;  /* First half are HA histograms */
            }
            else
            {
                parser->isHAHistogram = 0;  /* Second half are Default histograms */
            }
            
            parser->dualHistoCount++;
        }
        else
        {
            int histosPerCycle = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);

            /* First half of each cycle are HA, second half are Default */
            parser->isHAHistogram = (parser->dualHistoCount < histosPerCycle / 2);

            parser->dualHistoCount++;

            /* Reset counter at end of each cycle (before result frame) */
            if (parser->dualHistoCount >= histosPerCycle)
            {
                parser->dualHistoCount = 0;
            }
        }
    }
    else
    {
        parser->isHAHistogram = 0;
    }
    
    /* Parse footer from the accumulated data */
    tmf8829ParseFrameFooter(parser, data, parser->histogramDataLen);
    
    /* Parse histogram data */
    tmf8829ParseHistogramData(parser, data, subFrame);
    
#ifdef ENABLE_JSON_LOGGING
    /* Copy histogram to JSON logger buffer for later write */
    //PRINT_DEBUG("Frameparser: JSON logger open=%d, caching histogram\n", chip->jsonLogger.is_open);
    if (chip->jsonLogger.is_open)
    {
        tmf8829_json_cache_histogram(&chip->jsonLogger, parser, subFrame);
    }
#endif
    
    /* Reset histogram buffer for next frame */
    parser->histogramDataLen = 0;
}
#else
void handleReceivedHistogramDataEnd(void *dptr)
{
    /* Histogram disabled - do nothing */
    (void)dptr;
}
#endif /* ENABLE_HISTOGRAM */

/* ============================================================================
 * JSON Streaming (stdout, newline-delimited, for websocketd)
 * ============================================================================ */

static void tmf8829StreamFrameJson(tmf8829_chip *chip, uint8_t resultFormat)
{
    tmf8829FrameParser_t *parser = &chip->frameParser;
    tmf8829FrameData_t   *frame  = &parser->frame;
    int row, col, i;
    int numPeaks  = resultFormat & TMF8829_CFG_RESULT_FORMAT_NR_PEAKS_MASK;
    int useSignal = (resultFormat & TMF8829_CFG_RESULT_FORMAT_SIGNAL_STRENGTH_MASK) ? 1 : 0;
    int useNoise  = (resultFormat & TMF8829_CFG_RESULT_FORMAT_NOISE_STRENGTH_MASK)  ? 1 : 0;
    int useXtalk  = (resultFormat & TMF8829_CFG_RESULT_FORMAT_XTALK_MASK)           ? 1 : 0;

    if (numPeaks == 0) numPeaks = 1;

    fprintf(stdout,
            "{\"info\":{"
            "\"frame_number\":%u,"
            "\"read_time\":%u,"
            "\"systick_t0\":%u,"
            "\"systick_t1\":%u,"
            "\"temperature\":%d,"
            "\"warnings\":%d"
            "},\"results\":[",
            frame->frameNumber, frame->systick,
            frame->t0Integration, frame->t1Integration,
            frame->temperature, frame->warnings);

    for (row = 0; row < frame->numRows; row++)
    {
        fprintf(stdout, "[");
        for (col = 0; col < frame->numCols; col++)
        {
            tmf8829PixelResult_t *pixel =
                &parser->pixelResults[row * frame->numCols + col];

            fprintf(stdout, "{\"noise\":%d,\"peaks\":[",
                    useNoise ? pixel->noise : 0);

            for (i = 0; i < numPeaks; i++)
            {
                fprintf(stdout,
                        "{\"distance\":%d,\"signal\":%d,\"snr\":%d,"
                        "\"x\":\"%.2f\",\"y\":\"%.2f\",\"z\":\"%.2f\"}%s",
                        pixel->peaks[i].distance,
                        useSignal ? pixel->peaks[i].signal : 0,
                        pixel->peaks[i].snr,
                        pixel->peaks[i].x, pixel->peaks[i].y, pixel->peaks[i].z,
                        i < numPeaks - 1 ? "," : "");
            }

            fprintf(stdout, "],\"xtalk\":%d}%s",
                    useXtalk ? pixel->xtalk : 0,
                    col < frame->numCols - 1 ? "," : "");
        }
        fprintf(stdout, "]%s", row < frame->numRows - 1 ? "," : "");
    }

    fprintf(stdout, "]}\n");
    fflush(stdout);
}

/* ============================================================================
 * Print Functions
 * ============================================================================ */

void tmf8829PrintFrameInfo(tmf8829FrameParser_t *parser)
{
    if (parser->frame.warnings)
    {
        PRINT_INFO("Frame #%d has warnings: 0x%02x\n", parser->frame.frameNumber, parser->frame.warnings);
    }

    if (parser->frame.frameType == TMF8829_FID_RESULTS)
    {
        int fullRows, fullCols;
        tmf8829GetResolution(parser->frame.fpMode, &fullRows, &fullCols);

        /* Show sub-frame info for result frames in high-res modes */
        if (parser->frame.fpMode > TMF8829_CFG_FP_MODE_16x16)
        {
            int subFrameIdx = (parser->frame.layout & TMF8829_RESULT_FRAME_SUBIDX_MASK) ? 1 : 0;
            PRINT_INFO("Result Frame #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d)\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrameIdx, fullRows / parser->frame.numRows);
        }
        else
        {
             PRINT_INFO("Result Frame #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                    parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick);
        }
    }

    /* Show sub-frame info for histogram frames */
    if (parser->frame.frameType == TMF8829_FID_HISTOGRAMS)
    {
        int subFrame = parser->frame.layout & 0x0F;
        int totalSubFrames = tmf8829GetHistogramsPerResult(parser->frame.fpMode, parser->dualMode);

        /* Add HA indicator for dual mode histograms */
        if (parser->dualMode != 0)
        {
            const char *haType = parser->isHAHistogram ? "HA" : "Default";
            PRINT_INFO("Histogram #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d), %s\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrame, totalSubFrames, haType);
        }
        else
        {
            PRINT_INFO("Histogram #%d: fpMode=%d, dualMode=%d, res=%dx%d, systick=%u, sub-frame(%d of %d)\n",
                   parser->frame.frameNumber, parser->frame.fpMode, parser->dualMode,
                   parser->frame.numRows, parser->frame.numCols,
                   parser->frame.systick,
                   subFrame, totalSubFrames);
        }
    }
}

void tmf8829PrintFrameResults(tmf8829FrameParser_t *parser)
{
    int row, col;
    tmf8829PixelResult_t *pixel;

    PRINT_DEBUG("Frame #%d (res=%dx%d):\n",
           parser->frame.frameNumber,
           parser->frame.numRows, parser->frame.numCols);

    /* Print distance matrix */
    for (row = 0; row < parser->frame.numRows; row++)
    {
        for (col = 0; col < parser->frame.numCols; col++)
        {
            pixel = &parser->pixelResults[row * parser->frame.numCols + col];
            PRINT_DEBUG("%4d ", pixel->peaks[0].distance);
        }
        PRINT_DEBUG("\n");
    }

    /* Print crosstalk percentage */
    if (parser->xtalkPercentage > 0)
    {
        PRINT_DEBUG("maxXtalk:%d, %.3f%%\n", parser->maxXtalk, parser->xtalkPercentage);
    }

    /* Print keystone angles if enabled and result is ready */
#ifdef ENABLE_KEYSTONE
    if (parser->keystoneEnabled && parser->resultReady)
    {
        /* Get current timestamp */
        time_t now;
        struct tm *tm_info;
        char timestamp[32];
        
        time(&now);
        tm_info = localtime(&now);
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
        
        PRINT_INFO("[%s][F=#%d][sys=%u] Keystone angles: X=%.2f deg, Y=%.2f deg, Z=%.2f deg\n",
               timestamp,
               parser->frame.frameNumber,
               parser->frame.systick,
               parser->keystoneAngleX,
               parser->keystoneAngleY,
               parser->keystoneAngleZ);
    }
#endif
}

void tmf8829PrintFpsStats(tmf8829FrameParser_t *parser)
{
    if (parser->fpsFrameCount == 0)
    {
        PRINT_INFO("FPS Statistics: No complete frames received\n");
        return;
    }

    /* Calculate time difference in systick units (125kHz = 8us per tick) */
    uint32_t timeDiff = parser->fpsLastSystick - parser->fpsFirstSystick;

    if (timeDiff == 0)
    {
        PRINT_INFO("FPS Statistics: %d frames in <8us (too fast to measure)\n",
               parser->fpsFrameCount);
        return;
    }

    /* Convert time from 8us ticks to seconds: time / 125000.0 */
    double timeInSeconds = (double)timeDiff / 125000.0;

    /* Calculate FPS */
    double fps = (double)parser->fpsFrameCount / timeInSeconds;

    PRINT_INFO("FPS Statistics: %d frames in %.3f seconds = %.2f FPS\n",
           parser->fpsFrameCount, timeInSeconds, fps);
}
