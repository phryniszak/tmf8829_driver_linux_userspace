# Hand-Pose Estimation (optional add-on)

Turns live TMF8829 **48×32** depth frames into **22 human-hand joint angles** using
a small CNN trained in simulation (MuJoCo MS-Human-700 → ToF render → CNN). The
inference is a **dependency-free C port** (`handpose.c`) of the trained network;
no TensorFlow/TFLite is needed on the Pi.

Everything here is **additive and opt-in** — without `--handpose` the driver
behaves exactly as before.

## Files added
| File | Purpose |
|------|---------|
| `handpose.c/.h` | Pure-C CNN inference (loads `model/hand_pose.weights`). |
| `tmf8829_handpose.c/.h` | Glue: builds the depth map from `pixelResults`, runs the net, emits JSON. |
| `model/hand_pose.weights`, `model/meta.txt` | Trained weights + joint names/ranges. |
| `tools/handpose_test.c` | Host parity test (no sensor needed). |
| `tools_stream/hand_viewer.html` | Live web viewer (websocketd). |

Integration points (all guarded by `ENABLE_HANDPOSE`): a frame-ready hook in
`tmf8829_frameparser.c` (next to the JSON/keystone path), a context field in
`tmf8829_chip` (`tmf8829_driver.h`), and CLI flags in `main.c`.

## Build
`ENABLE_HANDPOSE` is ON by default:
```bash
cmake -S . -B build && cmake --build build
```
Requires libgpiod **v2** (same as the base driver). The host parity test builds
without any sensor/GPIO deps:
```bash
cmake --build build --target handpose_test
```

## Run (on the Pi)
Use the **48×32** mode (`-d 11`):
```bash
./build/tmf8829 -m -b 1 -d 11 --handpose -t 0
```
Per complete frame it prints a tagged line to stdout (kept separate from the
existing depth `--stream`, so current viewers are unaffected):
```
HP {"frame":123,"joints_deg":[...22 values...],"names":["cmc_flexion",...]}
```

### Live web viewer
```bash
websocketd --port=8080 --staticdir=tools_stream \
    ./build/tmf8829 -m -b 1 -d 11 --handpose -t 0
# open http://<pi-ip>:8080/hand_viewer.html
```

## Options
```
-P, --handpose              enable inference
    --handpose-model <f>    weights file (default model/hand_pose.weights)
    --handpose-meta  <f>    joint meta   (default model/meta.txt)
    --handpose-record <dir> dump depth frames (float32[1536]) for offline use
    --handpose-flipx/flipy/transpose   pixel-layout calibration
    --handpose-snrmin <n>   reject peaks with SNR below n
```

## Calibration & expectations (important)
The model was trained in simulation, so a first run needs a quick calibration:

1. **Pixel orientation.** The sensor's zone order may be mirrored/rotated vs the
   training layout. Capture frames with `--handpose-record <dir>` while holding a
   clearly **asymmetric** pose (only the index finger extended), then check the
   saved `*.bin` against the desktop tools (`probe_tof`/`nn_test` in the
   `cpp_hand_pose` project). Set `--handpose-flipx` / `--handpose-flipy` /
   `--handpose-transpose` until predictions match reality.
2. **Distance.** Hold the hand ~**0.30–0.45 m** from the sensor (the trained range).
3. **FoV.** Training assumed ~40.6°×27° for 48×32. If the device FoV differs the
   apparent hand scale will differ and accuracy drops — regenerate training data
   with the true FoV if needed.
4. **Accuracy.** Expect tight tracking on finger **spread/abduction** (~6–8°) and
   coarser **curl/flexion** (~20°) — an inherent limit of 48×32 depth. Use
   `--handpose-record` to collect real frames for a future fine-tuning pass.

## Verified
The C inference is numerically identical to the reference desktop implementation
(`cpp_hand_pose/src/nn_test.cpp`) and reproduces ~14° mean joint MAE on the
simulation validation set.
