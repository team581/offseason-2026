| Run | Setting | Startup Δv | Command ramp | Measured ramp fit | Peak command braking | Raw measured peak | Windowed measured peak |
|---|---:|---:|---:|---:|---:|---:|---:|
| a0.25 | 0.250 | 0.500 | 0.250 | 0.254 | 11.284 | 25.401 | 5.926 |
| a0.25-repeat | 0.250 | 0.500 | 0.250 | 0.173 | 9.217 | 31.985 | 6.819 |
| a0.75 | 0.750 | 0.500 | 0.750 | 0.750 | 17.543 | 17.078 | 6.034 |
| a0.75-repeat | 0.750 | 0.500 | 0.750 | 0.751 | 11.844 | 16.733 | 5.684 |
| a1.5 | 1.500 | 0.500 | 1.500 | 1.500 | 16.430 | 14.308 | 6.352 |
| a3 | 3.000 | 0.500 | 3.000 | 3.009 | 19.099 | 73.354 | 14.644 |
| a3-repeat | 3.000 | 0.500 | 3.000 | 2.984 | 12.394 | 56.031 | 13.630 |
| a10 | 10.000 | 0.500 | 10.000 | — | 26.407 | 53.189 | 14.929 |
| a50 | 50.000 | 0.500 | 50.041 | — | 16.777 | 18.531 | 15.882 |
| a50-repeat | 50.000 | 0.500 | 50.080 | — | 20.511 | 42.873 | 16.248 |

Units: acceleration in m/s²; startup Δv in m/s.
Measured ramp fits exclude the first 250 ms and last 100 ms of the rising command, and require at least 250 ms of data. A dash means insufficient ramp duration.
Fits retain scheduling stalls: inspect max_sample_interval_seconds in acceleration-metrics.json before interpreting a slow ramp.
Windowed values use a 120 ms local linear fit of measured speed. These do not establish vector-acceleration compliance on curved paths.
