# Tire temperature segmentation design

## Goal

From each MLX90640 32×24 thermal frame, identify the tire region, split it along its width axis into three segments (outside, center, inside), and publish one JSON payload per second with the average temperature of each segment.

## Context

- Sensor is fixed on the car and looks at one tire from approximately 10 o'clock above the rubber.
- Wheel steering moves the tire blob left/right or up/down in the frame.
- Inside/outside may be inverted; the segment closest to the left or top of the image is defined as outside.
- Target refresh rate: 1 Hz. Latency of ~1 s is acceptable.
- Output: single JSON payload published via MQTT per frame.

## Components

A new module `tire_segment.c/h` sits between `tire_temp.c` and `mlx90640.c`:

- `tire_segment_process(frame, ta, result)` — takes the raw 32×24 matrix and returns three averages plus detection metadata.
- `tire_segment_json(result, buf, len)` — formats the result as the JSON MQTT payload.
- `tire_temp.c` keeps the 1 Hz loop, calls the segmenter, and publishes via `mqttcomm_publish()`.

## Segmentation algorithm

For each 1 Hz frame:

1. **Temperature threshold**
   - Compute the 25th percentile (background) of the 768 pixel temperatures.
   - Tire pixels are those at least `TIRE_THRESHOLD_OFFSET` °C above that percentile (default 5 °C). This adapts to ambient changes.
   - If fewer than `TIRE_MIN_PIXELS` (e.g., 30) pixels qualify, mark detection as failed.

2. **Connected component**
   - Run 4-connectivity flood-fill on the threshold mask.
   - Keep the largest component as the tire; discard smaller noise blobs.

3. **Principal axis**
   - Compute centroid and second moments of the tire pixels.
   - The principal axis gives the tire's orientation angle.

4. **Split into three segments**
   - Project each tire pixel onto the principal axis.
   - Sort by projection value.
   - Divide into three equal-count bands along the axis: outside, center, inside.
   - Average the original temperatures of pixels in each band.

5. **Orientation rule**
   - The band with the most-negative projection (left/top in image coordinates) is labeled `outside`.
   - The opposite end is `inside`.
   - `center` is the middle band.

## Output format

One JSON payload per second, published to:

```
fiesta/tire-temp/<tire-id>
```

For now hardcode `tire-id = "tire-temp"` (matches the existing client ID). Payload:

```json
{
  "ts": 123456789,
  "ta": 26.3,
  "outside": 85.4,
  "center": 92.1,
  "inside": 88.7,
  "detected": true,
  "pixels": 142
}
```

- `ts`: frame timestamp (millis since boot).
- `ta`: sensor die temperature.
- `outside/center/inside`: segment averages in °C.
- `detected`: false if the tire could not be found.
- `pixels`: number of tire pixels used.

If `detected` is false, the three temperature fields are omitted.

## Error handling

- **No tire detected**: publish `detected: false` with `ta` and `ts`; do not crash or reboot.
- **Too few pixels**: same as above.
- **Degenerate blob** (all pixels in a line): still split by projection; averages may be noisy but valid.
- **Steering moves tire out of FOV**: `detected: false` until it returns.

## Testing

- Unit test the segmenter offline with captured 32×24 frames dumped to UART.
- Verify against known targets (coolbag / warm object) placed in the expected tire region.
- On the car: check that turning the wheel changes which segment is hottest and that `detected` stays true.

## Tunables

| Symbol | Default | Description |
|--------|---------|-------------|
| `TIRE_THRESHOLD_OFFSET` | 5.0 °C | Pixels warmer than background + offset are considered tire |
| `TIRE_MIN_PIXELS` | 30 | Minimum pixels for a valid detection |
| `TIRE_CONNECTIVITY` | 4 | Flood-fill connectivity |
