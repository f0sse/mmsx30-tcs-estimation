# Trailer

This is the trailer Simulink model. The purpose of each file is described in the table below. All files except the plot functions are supposed to be located in the IPG TruckMaker project directory under `src_tm4sl`.

| File | Description |
|------|-------------|
| `AccelNoise_modified.m` | Adds measurement noise to the simulated accelerometer, given a true signal and the sampling frequency Fs. |
| `GyroNoise_modified.m` | Adds measurement noise to the simulated gyroscope, given a true signal and the sampling frequency Fs. |
| `MeasurementModelAnalysis.m` | Plot function to analyse the predictive measurements (UKF). |
| `generic_Truck_And_Trailer.mdl` | Simulink model for the coupled truck and trailer system. |
| `plot_TruckAndTrailer.m` | Plot function to show the estimated states as functions of time, NEES, and other results included in the report. |
