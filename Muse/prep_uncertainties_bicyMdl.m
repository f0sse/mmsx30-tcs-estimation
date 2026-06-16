function unc = prep_uncertainties_bicyMdl()
        % Uncertainties for One Track (Bicycle) model – MUSE compatible
        
        %% Initial state covariance (P0)
        % Large initial uncertainty (filter convergence)
        p0.VELOCITY.CoG = [ ...
            1e-1;   % vx
            1e-1 ]; % vy
        
        p0.ANGULARVELOCITY.Reference(1:3,1) = 1e-2;  % yaw rate wr
        p0.ANGLE.Reference = 1e-1;            % yaw angle psi
        p0.POSITION.Reference = 1e0;          % lateral position y
        p0.Cf.Reference(1:2,1) = 5e5;         % Front cornerstiffness   
        p0.Cr.Reference(1:2,1) = 5e5;         % Front cornerstiffness 
        
        %% Process noise (Q)
        % Represents model uncertainty
        q.VELOCITY.CoG = [ ...
            5e-2;   % vx
            5e-2 ] * 100; % vy

        q.ANGULARVELOCITY.Reference(1:3,1) = 5e-3; % yaw rate
        %q.ANGLE.Reference = 1e-3;                 % yaw angle integration error
        %q.POSITION.Reference = 5e-2;              % lateral drift
        q.Cf.Reference(1:2,1)  =  0;               % Front cornerstiffness
        q.Cr.Reference(1:2,1) = 0;                 % Rear cornerstiffness      
        
        %% Measurement noise (R)
        % Must match measurement model exactly
        r.ACCELERATION = 0.1;          % ax, ay
        r.ANGULARVELOCITY = 5e-4;      % yaw rate
        r.WHEELSPEED = 0.1*0.49;
        r.VELOCITY = 0.1;              % vx, vy

        %% Pack output
        unc.q  = q;
        unc.r  = r;
        unc.p0 = p0;
end
