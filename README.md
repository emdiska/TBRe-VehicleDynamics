# TBRe Vehicle Dynamics — Yaw vs Lateral Acceleration Analysis

MATLAB tool for analysing yaw acceleration vs lateral acceleration 
from Formula Student vehicle telemetry data.

## What it does
- Loads any CSV telemetry on computer logs from the TBRe 2025 car
- Computes yaw acceleration from yaw rate (with correct unit conversion 
  and IMU bias removal)
- Maps the correct lateral acceleration channel (IMU mounting correction: The IMU is mounted wrong in the car so Lateral Acceleration is actually InlineAcc in our CSV files)
- Plots the dynamic envelope using a 99th percentile boundary in 0.25g bins
- Outputs linear regression: yaw response sensitivity [rad/s² per g]
- Prints left/right asymmetry analysis to console

## Key findings — Silverstone Endurance
- Lateral g: -1.86g to +1.45g
- Yaw acceleration: ±8 rad/s²
- Envelope coverage: 99%
- Yaw response sensitivity: 0.52 rad/s² per g
- Car behaviour: neutral-to-understeering at limit — correct for endurance

## Files
- `plot_yaw_vs_lateral.m` — script
- `enduranceIvanAxel_2025Car_SilverstNat_GenericTesting_a_3382_YMD` — Yaw acceleration VS Lateral Acceleration Graph

## Author
Mahdi Kadiri — MEng Mechanical Engineering (Automotive), University of Bath  
Vehicle Dynamics, TBRe Formula Student Team
