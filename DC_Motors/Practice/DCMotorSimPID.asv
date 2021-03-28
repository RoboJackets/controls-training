%% Simulation of DC Motor Systems
% By Varun Madabushi
% RoboJackets
% March 2021

close all

%% Parameters
% taken from Maxon EC 32
% (https://www.maxongroup.com/maxon/view/product/267121)
% Load is a wheel with mass 0.35kg and diameter 0.15m spinning freely
% Only things to be considered are inertia and viscous friction
J = .1*0.35*(0.15/2)^2; % Kg-m/s^2
R = 0.573; % Ohms
L = 0.09e-3; % H
Ki = 13e-3; % N-m/A
Kv = 1/Ki; % Rad/s/Volt
D = 2e-4; % Damping ratio


%% Initialize Arrays Holding State Variables
dt = 0.0001;
t = 0:dt:10; % 10 seconds
theta = zeros(1,length(t));
omega = zeros(1,length(t));
current = zeros(1,length(t));
V=0;

%% Initialize PID Constants
KP = 0;
KI = 0; % Not to be confused with Ki
KD = 0;

r = 900; % goal of 900 rad/s
sigma = 0; % Variable to hold integral
prev = 0; % Variable to hold previous value for difference equation

%% Forward Euler Simulation

for k = 1:length(t)-1
        
    % Evaluate derivatives of all state variables
    % Speed is the integral of position
    thetadot = ;
    
    % Acceleration (omegadot)
    % Viscous friction model: friction torque proportional to speed
    omegadot = ; % tau_L = D*omega
    
    % Current derivative
    idot = ; 
   
    % PID Control:
    error = ; % Compute Error
    V = KP*error + KI*sigma + KD*(error - prev)/dt; % Implement PID equation
    V = min(24, max(-24, V)); % Apply voltage max limits
    sigma = sigma + error * dt; % Update sum for integral
    prev = error; % Record previous value for derivative
    
    % Forward Euler Method (First Order Taylor Series)
    theta(k+1) = theta(k) + dt*thetadot;
    omega(k+1) = omega(k) + dt*omegadot;
    current(k+1) = current(k) + dt*idot;
end

%% Plots
subplot(3,1,1)
plot(t,theta)
subplot(3,1,2)
hold on
plot(t,omega)
plot(t, r*ones(1,length(t)), 'r--')
subplot(3,1,3)
plot(t, current)