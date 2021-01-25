t20 = [];
t50 = [];
t80 = [];
sx = struct;
sx.l = [];
sx.r = [];

%% code
close all;
tpmClock = 48000000 / 4;

if exist('sx', 'var') 
    timeDelta = sx;
    sampleTime = 0.001;
    resampled = struct;
    cycleRate = struct;
    time = struct;
    reCycleRate = struct;
    
    
    figure; subplot(2,1,1); plot(timeDelta.l); subplot(2,1,2); plot(timeDelta.r);
    cycleRate.l = [0, 1 / 390 ./ ( timeDelta.l / tpmClock )];
    time.l = [0, cumsum(timeDelta.l / tpmClock)];
    cycleRate.r = [0, 1 / 390 ./ ( timeDelta.r / tpmClock )];
    time.r = [0, cumsum(timeDelta.r / tpmClock)];
    figure; subplot(2,1,1); plot(time.l, cycleRate.l); subplot(2,1,2); plot(time.r, cycleRate.r);
    stooop
end

figure; plot(t20, 'r'); hold on; plot(t50, 'g'); hold on; plot(t80, 'b');
[ori20, filter20, reCycleRate20, reTime20, sys20] = getSys(tpmClock, 20, t20);
[ori50, filter50, reCycleRate50, reTime50, sys50] = getSys(tpmClock, 50, t50);
[ori80, filter80, reCycleRate80, reTime80, sys80] = getSys(tpmClock, 80, t80);

sys20, sys50, sys80

figure;
subplot(2,1,1);
plot([0:length(ori20)-1], ori20, 'r'); hold on;
plot([0:length(ori50)-1], ori50, 'g'); hold on;
plot([0:length(ori80)-1], ori80, 'b'); hold on;
title('cycleRate (cycle/sec) - sample point');
subplot(2,1,2);
plot([0:length(filter20)-1], filter20, 'r'); hold on;
plot([0:length(filter50)-1], filter50, 'g'); hold on;
plot([0:length(filter80)-1], filter80, 'b'); hold on;
title('filted cycleRate (cycle/sec) - sample point');

figure;
subplot(2,1,1);
sysTime = [0:2*length(reTime20)] * reTime20(2);
sysSim20 = lsim(sys20, 20 * ones(length(sysTime), 1), sysTime);
sysSim50 = lsim(sys50, 50 * ones(length(sysTime), 1), sysTime);
sysSim80 = lsim(sys80, 80 * ones(length(sysTime), 1), sysTime);
plot(sysTime, sysSim20, 'r'); hold on; plot(reTime20, reCycleRate20, 'rx'); hold on; 
plot(sysTime, sysSim50, 'g'); hold on; plot(reTime50, reCycleRate50, 'gx'); hold on;
plot(sysTime, sysSim80, 'b'); hold on; plot(reTime80, reCycleRate80, 'bx'); hold on;
title('resampled cycleRate (cycle/sec) - time(sec); sys cycleRate - time');
subplot(2,1,2);
sysTime = [0:2*length(reTime20)] * reTime20(2);
sysSim20 = lsim(sys20, 100 * ones(length(sysTime), 1), sysTime);
sysSim50 = lsim(sys50, 100 * ones(length(sysTime), 1), sysTime);
sysSim80 = lsim(sys80, 100 * ones(length(sysTime), 1), sysTime);
plot(sysTime, sysSim20, 'r'); hold on;
plot(sysTime, sysSim50, 'g'); hold on;
plot(sysTime, sysSim80, 'b'); hold on;
title('sys20, sys50, sys80 with input 100');

simulation1(sys50, 1, 1, 1.0, 0.0);
simulation1(sys50, 2, 1, 1.0, 0.0);
simulation1(sys50, 3, 1, 1.0, 0.0);
simulation1(sys50, 4, 1, 1.0, 0.0);
simulation1(sys50, 5, 1, 1.0, 0.0);

simulation1(sys50, 1, 1, 1.0, 0.06);
simulation1(sys50, 2, 1, 1.0, 0.06);
simulation1(sys50, 3, 1, 1.0, 0.06);
simulation1(sys50, 4, 1, 1.0, 0.06);
simulation1(sys50, 5, 1, 1.0, 0.06);

simulation1(sys50, 1, 4, 1.2, 0.06);
simulation1(sys50, 2, 4, 1.2, 0.06);
simulation1(sys50, 3, 4, 1.2, 0.06);
simulation1(sys50, 4, 4, 1.2, 0.06);
simulation1(sys50, 5, 4, 1.2, 0.06);


% we assume that the motor is a L-R-E system, and with calculations , we
% get the system is like A = K / ((a * s + 1) * (b * s + 1))
% filter must be used, or the tfest would fail.
% we use mean filter as we think that periodic noise period is 13, so the faster the motor turns
% the higher the noise frequency would be. 
function [oriCycleRate, cycleRate, reCycleRate, reTime, sys, num, den] = getSys(tpmClock, duty, timeDelta)
    % timeDelta = hex2dec('FFFFFFFF') ./ a20;
    % duty = 20;
    % cut = 8;
    % max_timePeriod = 32 * hex2dec('5DC0');
    sampleTime = 0.01;
    oriCycleRate = [0, 1 / 390 ./ ( timeDelta / tpmClock )];
    
    % cut filter
    % figure; subplot(3,1,1); plot(timeDelta);
    % if (timeDelta(1) > cut * max_timePeriod || max_timePeriod > 3 * timeDelta(1))
    %     timeDelta(1) = max_timePeriod;
    % end
    % for i = 2: length(timeDelta)
    %     if timeDelta(i) > cut * timeDelta(i-1) || timeDelta(i-1) > cut * timeDelta(i)
    %         timeDelta(i) = timeDelta(i-1);
    %     end 
    % end
    % subplot(3,1,2); plot(timeDelta);

    % mean filter method 1
    period = 8;
    timeDeltaMod = zeros(1, floor(length(timeDelta) / period));
    for i = 1:floor(length(timeDelta) / period)
        timeDeltaMod(i) = sum(timeDelta(period*i-period+1:period*i))/ period;
    end
    timeDelta = timeDeltaMod;
    % subplot(3,1,3); plot(timeDelta);
    cycleRate = [0, 1 / 390 ./ ( timeDelta / tpmClock )];
    
    % proc
    time = [0, period * cumsum(timeDelta / tpmClock)];
    resampled = resample(timeseries(cycleRate,time), 0:sampleTime:time(length(time)), 'linear');
    reCycleRate = resampled.data;
    reTime = resampled.time;
    reCycleRate = reshape(reCycleRate, [length(reCycleRate), 1]);
    input = duty * ones(length(reCycleRate), 1);
    data = iddata([0;0;0;reCycleRate],[0;0;0;input],sampleTime);
    sys = tfest(data,2,0);
    num = sys.Numerator / sys.Denominator(3);  
    den = sys.Denominator / sys.Denominator(3);
end

% PID: C = Kp + Ki / s + Kd * s = (Kd * s^2 + Kp * s + Ki) / s
% but in this experiment, we use modified feedback: u = Kp * (e) + x / K
% K = numA(1); zeta = (t1 + t2) / (2 * sqrt(Kp * K * t1 * t2));
% thus: Kp = denA(2)^2 / 4 / zeta^2 / denA(1) / numA(1);
function simulation(Gs, target, meanTime, zet)
    % target = 0.5;
    % meanTime = 13
    % zet = 3.6;
    num = Gs.Numerator / Gs.Denominator(3);  
    den = Gs.Denominator / Gs.Denominator(3);
    K = num(1);
    Kp = (den(2) * den(2)) / ( den(1) * K * 4 * zet * zet );
    [K, Kp]
    
    
    cycleRate = [0];
    u = [0];
    time=[0 0.0184];

    % program bellow copyed from PPT may have problem: U(n) always get the nearest, so U(n) != u(n) may take place!
    % to solve this, do not use "u(n) = U(n)"
    % for n=2:1000
    %     u(n)= target / K + Kp * (target - cycleRate(n-1));
    %     T = 0:0.001:time(n);
    %     U = interp1(time(1:n), u, T, 'nearest');
    %     for i=1:length(U)
    %         if U(i)>100
    %             U(i)=100;
    %         elseif U(i)<0
    %             U(i)=0;
    %         end
    %     end
    %     u(n)= U(length(U));
    %     Rs = lsim(Gs,u,T);
    %     cycleRate(n) = Rs(length(Rs));
    %     time(n+1) = time(n) + 1 / 390 / cycleRate(n);
    % end

    % modified program
    for n=2:200
        u(n)= target / K + Kp * (target - cycleRate(n-1));
        if u(n) > 100
            u(n) = 100;
        elseif u(n) < 0
            u(n) = 0;
        end
        T = time(1):0.001:time(n);
        U = interp1(time(1:n), u(1:n), T, 'nearest');
        Rs = lsim(Gs,U,T);
        cycleRate(n) = Rs(length(Rs));
        % cycleRate(n) = cycleRate(n) * (1 + noiseAmp * (rand(1) - 0.5));
        time(n+1) = time(n) + meanTime / 390 / cycleRate(n);
    end

    figure;
    subplot(2,1,1); plot(time(1:length(time)-1),cycleRate, '-o'); 
    title(strcat('cycleRate-time; meanTime:', int2str(meanTime)));
    subplot(2,1,2); plot(time(1:length(time)-1),u, '-o'); title('duty-time');
end

function simulation1(Gs, target, meanTime, zet, noiseAmp)
    % target = 0.5;
    % meanTime = 13
    % zet = 3.6;
    % noiseAmp = 0.1;
    num = Gs.Numerator / Gs.Denominator(3);  
    den = Gs.Denominator / Gs.Denominator(3);
    K = num(1);
    Kp = (den(2) * den(2)) / ( den(1) * K * 4 * zet * zet );
    [K, Kp]

    % cycleRate = [0];
    cycleRate = zeros(1, meanTime);
    u = zeros(1, meanTime);
    time = 0.02*[0:meanTime];

    % modified program
    for n = meanTime+1:meanTime+400
        cr = 0;
        maxcr = 0;
        mincr = 0;
        if meanTime > 2 
            for m = 1:meanTime
                maxcr = max([cycleRate(n-m), maxcr]);
                mincr = min([cycleRate(n-m), maxcr]);
                cr = cr + cycleRate(n-m);
            end
            cr = (cr - maxcr - mincr)/ (meanTime - 2);
        else
            cr = cycleRate(n-1);
        end
        u(n)= target / K + Kp * (target - cr);
        if u(n) > 100
            u(n) = 100;
        elseif u(n) < 0
            u(n) = 0;
        end
        T = time(1):0.001:time(n);
        U = interp1(time(1:n), u(1:n), T, 'nearest');
        Rs = lsim(Gs,U,T);
        cycleRate(n) = Rs(length(Rs));
        time(n+1) = time(n) + 1 / 390 / cycleRate(n);
        cycleRate(n) = cycleRate(n) * (1 + noiseAmp * (rand(1) - 0.5));
    end

    figure;
    subplot(2,1,1); 
    plot(time(meanTime:length(time)-1)-time(meanTime),cycleRate(meanTime:length(cycleRate)), '-o'); 
    title(strcat('cycleRate-time; meanTime:', int2str(meanTime)));
    subplot(2,1,2); 
    plot(time(meanTime:length(time)-1)-time(meanTime),u(meanTime:length(cycleRate)), '-o'); 
    title('duty-time');
end



