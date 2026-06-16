classdef OneTrack < muse.estimation.models.BaseModel
    properties
        name = 'OneTrack'
        parameters
        utility
    end

    %% ============================================================
    % MEASUREMENTS
    %% ============================================================
    methods
        function measurements = defineMeasurements(obj, vehicle)
            u = obj.utility;

            % Define accelerometer measurements (2 axes)
            acc = arrayfun(@(j) {'ACCELERATION', j, 'Sensor', 1}, 1:3, 'UniformOutput', false);
         
            %Define velocity meassurements (2 axes)
            vel = arrayfun(@(j) {'VELOCITY', j, 'Sensor', 1}, 1:3, 'UniformOutput', false);
            
            % Define gyroscope measurements (z axes)
            gyro = arrayfun(@(j) {'ANGULARVELOCITY', j, 'Sensor', 1}, 3, 'UniformOutput', false);
            
            % Define gyroscope measurements (3 axes)
            % acc = arrayfun(@(j) {'ACCELERATION', j, 'Sensor', 2}, 1:3, 'UniformOutput', false);
            % gyro = arrayfun(@(j) {'ANGULARVELOCITY', j, 'Sensor', 2}, 1:3, 'UniformOutput', false);
            wheelspeeds = {};

            % This model only considers the truck/tractor
            for axle = 2:length(vehicle.bodies(u.Tractor).activeAxles)
                if vehicle.bodies(u.Tractor).activeAxles(axle) == 0
                    continue;
                end
                posName = ['Axle' num2str(axle)];
                % Concatenate rows directly - each arrayfun element is already a 1x4 cell
                wheelspeeds = [wheelspeeds; ...
                    arrayfun(@(side) {'WHEELSPEED', side, posName, u.Tractor}, 1:2, 'UniformOutput', false)']; %#ok<AGROW>
            end
            
            % Vertically concatenate to create Nx4 cell array
            % Use cell2mat to convert cell-of-cells to simple cell array
            measurements = vertcat(acc{:}, gyro{:}, wheelspeeds{:}, vel{:});
        end
    end

    %% ============================================================
    % PARAMETER SETUP
    %% ============================================================
    methods
        function [parameters, processNoise, updateNoise, initialCovariance] = setupParameters(obj, vehicle)
            unc = prep_uncertainties_bicyMdl();
            params = prep_params_BicyclMdl(vehicle);

            parameters = params;
            initialCovariance = unc.p0;
            processNoise = unc.q;
            updateNoise = unc.r;
        end
    end

    %% ============================================================
    % MODEL CONFIGURATION
    %% ============================================================
    methods
        function modelConfiguration(obj, vehicle)
            modelConf = struct;
            modelConf.Units = length(vehicle.bodies);
            modelConf.MaxAxles = max([vehicle.bodies.numAxles]);

            axlePos = [vehicle.bodies.axlePositions];
            modelConf.ShiftToAxle = reshape(axlePos, modelConf.MaxAxles, modelConf.Units)';

            ShiftCouplingRear  = [vehicle.bodies.couplingPositionRear]';
            ShiftCouplingFront = [vehicle.bodies.couplingPositionFront]';
            modelConf.ShiftToSensor = vehicle.transforms.Sensor.Positions';

            modelConf.ShiftToCoupling = [ShiftCouplingFront(:,1), ShiftCouplingRear(:,1)];

            a = [vehicle.bodies.activeAxles];
            modelConf.UsedAxles = reshape(a, modelConf.MaxAxles, modelConf.Units)';

            a = [vehicle.bodies.trackWidth];
            modelConf.AxleWidths = reshape(a, modelConf.MaxAxles, modelConf.Units)';

            a = [vehicle.bodies.steerableAxles];
            modelConf.SteerableAxle = reshape(a, modelConf.MaxAxles, modelConf.Units)';

            r(1,:,:) = [vehicle.bodies.tireRadius];
            modelConf.radi = cat(1,r,r);

            modelConf.ShiftToRTOutput = vehicle.transforms.RT_Output.Positions';

            obj.modelConfig = modelConf;
        end
    end

    %% ============================================================
    % DEFINE PARAMETERS
    %% ============================================================
    methods
        function defineParameters(obj)
            %% not in use directly right now 
            obj.utility = muse.estimation.models.UtilityFunctions_Semitrailer(obj.modelConfig);
            u = obj.utility;
            
            % Initialize parameters with uncertainties
            obj.parameters.CoGPos = [u.q.add('CoG1'); u.q.add('CoG2'); u.q.add('CoG3')];
            obj.parameters.SenPos = [u.q.add('SenPos1'); u.q.add('SenPos2'); u.q.add('SenPos3')];
            obj.parameters.Rw = [u.q.add('Rw')];
            obj.parameters.Jz = [u.q.add('Jz')];
            obj.parameters.m  = [u.q.add('m')];
            obj.parameters.a = [u.q.add('a')];
            obj.parameters.b = [u.q.add('b')];
            obj.parameters.L = [u.q.add('L')];
            obj.parameters.r = [u.q.add('r1'); u.q.add('r2'); u.q.add('r3')];
        end
    end

    %% ============================================================
    % DEFINE EQUATIONS
    %% ============================================================
    methods
        function defineEquations(obj)
            u = obj.utility;
            p = obj.parameters;

            % ----------------------------
            % STATES
            % ----------------------------
            syms vx(t) vy(t) wz(t) Cf(t) Cr(t)

            obj.states.VELOCITY.CoG(u.AllDir,1) = [vx; vy; 0];
            obj.states.ANGULARVELOCITY.Reference(u.AllDir,1) = [0; 0; wz];
            obj.states.Cf.Reference(u.AllDir,1) = [0; Cf; 0];
            obj.states.Cr.Reference(u.AllDir,1) = [0; Cr; 0];
            u = u.chooseCenter(p.CoGPos(1));
            %u = u.specifyRadius(p.R);

            % ----------------------------
            % INPUTS
            % ----------------------------
            is = obj.states;
            in = obj.defineInputs(u);

            % ----------------------------
            % GEOMETRY
            % ----------------------------
            %xCog = u.ShiftToAxle(2);
            %a = u.ShiftToAxle - xCog;
            %af = a(find(u.SteerableAxle,1));
            %ar = mean(a(~u.SteerableAxle));

            % ----------------------------
            % SLIP ANGLES
            % ----------------------------
            L_CoG = abs(p.CoGPos(1));
            lr = p.L - L_CoG;
            lf = L_CoG;
            delta = in.STEERINGANGLE.Axle1(1,1);
            alpha_f = -delta + (vy + lf*wz)/vx ;
            alpha_r = (vy - lr*wz)/vx;
    
            % ----------------------------
            % TIRE FORCES
            % ----------------------------
            Ffy = -Cf * alpha_f;
            Fry = -Cr * alpha_r;
            Frx = in.APPLIEDTORQUE.Axle3(1,1)/p.Rw;
            Ffx = 0;

            % ----------------------------
            % VEHICLE DYNAMICS
            % ----------------------------
            % AXELUPDATE: Removed equation type statement, only declare change in time directly
            dvx = (Ffx*cos(delta) - Ffy*sin(delta) + Frx)/p.m + vy*wz;
            dvy = (Ffx*sin(delta) + Ffy*cos(delta) + Fry)/p.m - vx*wz;
            dwz = ((Ffy*cos(delta)+Ffx*sin(delta))*lf - lr*Fry)/p.Jz;

            dCf = 0;
            dCr = 0;
            width = u.AxleWidths(2,1);

            % ----------------------------
            % INTERMEDIATE
            % ----------------------------

            % Angular acceleration
            is.ANGULARACCELERATION.Reference(3, u.Tractor) = dwz;

            % Non-inertial acceleration
            is.VELOCITYDOT.CoG(:, u.Tractor) = [dvx; dvy; 0];

            % Acceleration measurements; first, CoG,
            a = is.VELOCITYDOT.CoG(:, u.Tractor) + cross([0;0;wz], [vx; vy; 0]);
            is.ACCELERATION.CoG(:,u.Tractor) = a;

            % then, sensor and ref.,
            is.ACCELERATION.Sensor(1:3,1) = ...
                AccelerationAtStatePos(u, is, a, 'CoG', 'Sensor', u.Tractor);
            is.ACCELERATION.Reference(u.AllDir, u.Tractor) = ...
                AccelerationAtStatePos(u, is, a, 'CoG', 'Reference', u.Tractor);
            is.ACCELERATION.Reference(u.AllDir, u.Tractor) = ...
                AccelerationAtStatePos(u, is, a, 'CoG', 'RT_Output', u.Tractor);

            % Veloctiy Measurements from RT
            is = u.calculateVelocityAtPoint(is, 'RT_Output', u.Tractor);
            is = u.calculateVelocityAtPoint(is, 'Reference', u.Tractor);
            is = u.calculateVelocityAtPoint(is, 'Sensor', u.Tractor);

            % Actual wheel speed, not angular velocity
            is.WHEELSPEED.Axle2(1,1) = (vx - width*wz)/1; % left
            is.WHEELSPEED.Axle2(2,1) = (vx + width*wz)/1; % right
            is.WHEELSPEED.Axle3(1,1) = (vx - width*wz)/1; % left
            is.WHEELSPEED.Axle3(2,1) = (vx + width*wz)/1; % right

            % Angular velocity (Yaw rate)
            is.ANGULARVELOCITY.Sensor = is.ANGULARVELOCITY.Reference;

            % add extra states for plotting
            is.Slip.Axle1(:,1) = alpha_f;
            is.Slip.Axle2(:,1) = alpha_r;

            is.TyreForce.Axle1(:,1) = -is.Slip.Axle1*Cf;
            is.TyreForce.Axle2(:,1) = -is.Slip.Axle2*Cr;

            % ----------------------------
            % EQUATIONS
            % ----------------------------
            eqns.VELOCITY.CoG(:,1) = is.VELOCITYDOT.CoG(:,1);
            eqns.ANGULARVELOCITY.Reference(3,1) = dwz;
            eqns.Cf.Reference(2,1) = dCf;
            eqns.Cr.Reference(2,1) = dCr;

            obj.equations = eqns;
            obj.inputs = in;
            obj.intermediate = is;
        end
    end

    %% ============================================================
    % INPUT DEFINITIONS
    %% ============================================================
    methods (Static)
        function in = defineInputs(u)
            %USUALINPUTS Sets up the usual inputs, SteerAngle, DriveTorque
            in = struct;

            %in.STEERINGANGLE = sym(zeros(1,u.MaxAxles));
            %in.APPLIEDTORQUE = sym(zeros(1,u.MaxAxles));
        
            %for axle = 1:u.MaxAxles
                %if ~u.UsedAxles(1,axle)
                %    continue
                %end
                axle_T=3;
                axle_ST=1;
                position = ['Axle' num2str(axle_T)];
                symbol = sym(['dt_1_' num2str(axle_T)], 'real');
                in.APPLIEDTORQUE.(position) = symbol;
    
                %if ~u.SteerableAxle(1,axle)
                %    continue
                %end
                position = ['Axle' num2str(axle_ST)];
                symbol = sym(['s_' num2str(axle_ST)],'real');
                in.STEERINGANGLE.(position) = symbol;
            %end
        end
    end
end
