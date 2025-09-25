clear;
clc;


%% Serial port utility
ports = serialportlist("available");
disp(ports);


%% Parameters
N = 6;
ts_us = 100000;
ticks = 1;
dt = 1e-6 * ts_us;
port = ports(1);
baudrate = 115200;

T = 0:dt:10;
% pwms = [255 * sin(2*pi.*T./10);
%         255 * cos(2*pi.*T./10);
%         2 * T;
%         255 * sin(2*pi.*T./10);
%         255 * cos(2*pi.*T./10);
%         2 * T];

pwms = [+0.0 * 255 * ones(1, length(T));
        +0.0 * 255 * ones(1, length(T));
        +0.0 * 255 * ones(1, length(T));
        -0.0 * 255 * ones(1, length(T));
        -0.0 * 255 * ones(1, length(T));
        -0.0 * 255 * ones(1, length(T))];

status = zeros(1, length(T));
deltas = zeros(1, length(T));
ends = zeros(N, length(T));
encs = zeros(N, length(T));


%% Initialization
clear robot
robot = Robot(N, ts_us, port, baudrate);
fprintf("\nWaiting for robot to be ready ");
for k = 1:15
    pause(1);
    fprintf(".");
end
fprintf(" Done\n");


%% Robot Setup
res = robot.setup_robot(ts_us, ticks);
if(~res)
    error("Error for robot setup");
end

for k = 1:N
    res = robot.setup_motor(k, 0, 0, 0);
    if(~res)
        error("Error for motor " + string(k) + " setup");
    end
end

for k = 1:N
    robot.setup_pid(k, 1, 0, 0, 0, 0, 0);
    if(~res)
        error("Error for pid " + string(k) + " setup");
    end
end


%% Test 1
last = tic();
for k = 1:length(T)
    res = robot.ctrl_pwm(pwms(:, k));
    delta = toc(last) * 1e6;
    last = tic();
    status(1, k) = res;
    deltas(1, k) = delta;
    ends(1:N, k) = robot.getEndstops();
    encs(1:N, k) = robot.getEncoders();
    % fprintf("cycle:    %d\n", k);
    % fprintf("res:      %s\n", string(res));
    % fprintf("delta:    %0.1f us\n", delta);
    % fprintf("switches:");
    % for m = 1:N
    %     fprintf(" %d", ends(m,k));
    % end
    % fprintf("\n");
    % fprintf("encoders:");
    % for m = 1:N
    %     fprintf(" %d", encs(m,k));
    % end
    % fprintf("\n");
end

robot.ctrl_idle();


%%
tiledlayout(5,1);

ax1 = nexttile;
plot(T', pwms');
grid on;
title("PWMs");
legend("pwm " + string(1:N));

ax2 = nexttile;
plot(T', status');
grid on;
title("Status");
legend("comm ok ");

ax3 = nexttile;
plot(T', deltas');
grid on;
title("Deltas");
legend("delta time ");

ax4 = nexttile;
plot(T', ends');
grid on;
title("Switches");
legend("switch " + string(1:N));

ax5 = nexttile;
plot(T', encs');
grid on;
title("Encoders");
legend("encoders " + string(1:N));

linkaxes([ax1, ax2, ax3, ax4, ax5], 'x');

