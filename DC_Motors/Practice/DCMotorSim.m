%% Simulation of DC Motor Systems
% By Varun Madabushi
% RoboJackets
% March 2021

close all

%% Parameters
% taken from Maxon EC 32
% Load is a wheel with mass 0.35kg and diameter 0.15m spinning freely
% Only things to be considered are inertia and viscous friction
J = .1*0.35*(0.15/2)^2; % Kg-m/s^2
R = 0.573; % Ohms
L = 0.09e-3; % H
Ki = 13e-3; % N-m/A
Kv = 1/Ki; % Rad/s/Volt
D = 2e-4; % Viscous friction coefficient


%% Initialize Arrays Holding State Variables
dt = 0.0001;
t = 0:dt:2; % 10 seconds
theta = zeros(1,length(t));
omega = zeros(1,length(t));
current = zeros(1,length(t));
V = 18; % V

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
    
    
    % Forward Euler Method (First Order Taylor Series)
    theta(k+1) = ;
    omega(k+1) = ;
    current(k+1) = ;
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