%
% increasingAnalysis.m
%
% Use CSV files of data exported from IPGControl in TruckMaker
% to find side-slip angle and normal force from driving the
% truck in a steady-state circle of radius 100m with 10 kmh
% increments every 100 seconds, lasting 50 seconds, starting
% at t = 100, spanning 20 kmh to 60 kmh.
%

clc
clear variables

load("../regressions/increasing_velocity_circle_100m_trailer.mat");

Labels = ["FL", "FR", "RL", "RR"];

SlipAngles = [
    Car_SlipAngleFL(:), ...
    Car_SlipAngleFR(:), ...
    Car_SlipAngleRL(:), ...
    Car_SlipAngleRR(:)
];
LateralForces = [
    Car_FyFL(:), ...
    Car_FyFR(:), ...
    Car_FyRL(:), ...
    Car_FyRR(:)
];
VerticalForces = [
    Car_FzFL(:), ...
    Car_FzFR(:), ...
    Car_FzRL(:), ...
    Car_FzRR(:)
];

Time = Time(:);
EndTime = Time(end);

MassLateralAcceleration = Sensor_Inertial_Vhcl_IMU_COG_Acc_B_y(:);
%MassLateralAcceleration = Car_ay(:);

incrementStart = 0;
incrementInterval = 200;
incrementDuration = 50;

velocityStart = 20;
velocityIncrement = 10;

%% Compute Table Values
for i = 1:size(SlipAngles,2)
    SlipAngle     = SlipAngles(:, i);
    LateralForce  = LateralForces(:, i);
    VerticalForce = VerticalForces(:, i);

    ii = 1;

    fprintf("\n==========\nWheel %s\n==========\n", Labels(i))

    max_ssa = 0;
    max_fz  = 0;

    while incrementStart + ii*incrementInterval <= EndTime
        fprintf("\n")
        time_end   = incrementStart + ii*incrementInterval;
        time_start = time_end - (incrementInterval - incrementDuration);

        steadyMask = (Time >= time_start) & (Time < time_end);
        steadyTime = Time(steadyMask);

        meanSlip = mean(SlipAngle(steadyMask));
        stdSlip  =  std(SlipAngle(steadyMask));
        meanFy   = mean(LateralForce(steadyMask));
        stdFy    =  std(LateralForce(steadyMask));
        meanFz   = mean(VerticalForce(steadyMask));
        stdFz    =  std(VerticalForce(steadyMask));

        %clf
        %plot(steadyTime, SlipAngle(steadyMask)); hold on
        %yline(meanSlip)
        %keyboard

        velocity = velocityStart + (ii-1)*velocityIncrement;

        fprintf("[%i] mean   ssa = \\num{%.4e}\n", velocity, meanSlip);
        fprintf("[%i]  std   ssa = \\num{%.4e}\n", velocity, stdSlip);
        fprintf("[%i] mean    Fy = \\num{%.4e}\n", velocity, meanFy);
        fprintf("[%i]  std    Fy = \\num{%.4e}\n", velocity, stdFy/meanFy);
        fprintf("[%i] mean    Fz = \\num{%.4e}\n", velocity, meanFz);
        fprintf("[%i]  std    Fz = \\num{%.4e}\n", velocity, stdFz);

        max_ssa = max([ max_ssa, abs(stdSlip) ]);
        max_fz  = max([ max_fz,  abs(stdFz) ]);

        ii = ii + 1;
    end

    fprintf("\nssa %.4e\n", max_ssa);
    fprintf("f_z %.4e\n", max_fz);
end

%% Lateral Acceleration
fprintf("\n==========\nAccel\n==========\n")

ii = 1;

max_ay = 0;

while incrementStart + ii*incrementInterval <= EndTime
    fprintf("\n")
    time_end   = incrementStart + ii*incrementInterval;
    time_start = time_end - (incrementInterval - incrementDuration);
    steadyMask = (Time >= time_start) & (Time < time_end);
    steadyTime = Time(steadyMask);

    meanAccel = mean(MassLateralAcceleration(steadyMask));
    stdAccel  =  std(MassLateralAcceleration(steadyMask));

    velocity = velocityStart + (ii-1)*velocityIncrement;

    fprintf("[%i] mean ay = %.4e\n", velocity, meanAccel);
    fprintf("[%i]  std ay = %.4e\n", velocity, stdAccel/meanAccel);

    max_ay = max([ max_ay, abs(stdAccel/meanAccel) ]);

    ii = ii + 1;
end

%% Compare Forces and Slip

mask=((Time>51)&(Time<999));
figure(1)
clf
plot(Time(mask), SlipAngles(mask,:))
legend(Labels)
xlim([51,999])

figure(2);
clf
plot(Time(mask), Car_FzRR(mask)-Car_FzRL(mask))