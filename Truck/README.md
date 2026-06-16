# Truck

This is the truck Simulink model. The purpose of each file is described in the table below. All files except the plot functions are supposed to be located in the IPG TruckMaker project directory under `src_tm4sl`.

| File | Description |
|------|-------------|
| `AccelNoise_modified.m` | Adds measurement noise to the simulated accelerometer, given a true signal and the sampling frequency Fs. |
| `GyroNoise_modified.m` | Adds measurement noise to the simulated gyroscope, given a true signal and the sampling frequency Fs. |
| `MeasurementModelAnalysis.m` | Plot function to analyse the predictive measurements (UKF). |
| `generic_truck.mdl` | Simulink model for the truck-only system. |
| `plot_funtion_5_state_wheelSpeed.m` | Plot function to show the estimated states as functions of time, NEES, and other results included in the report. |
