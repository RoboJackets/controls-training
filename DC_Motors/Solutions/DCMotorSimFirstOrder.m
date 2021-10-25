%% Simulation of First Order DC Motor
% By Varun Madabushi
% RoboJackets
% Oct 2021

close all

%% Parameters
% Load is a wheel with mass 0.35kg and diameter 0.15m spinning freely
% Only things to be considered are inertia and viscous friction
J = .1*0.35*(0.15/2)^2; % Kg-m/s^2
R = 0.573; % Ohms
Ki = 13e-3; % N-m/A
Kv = 1/Ki; % Rad/s/Volt
% Viscous friction coefficient is usually unknown but can be set to 0
D = 2e-8; % Viscous friction coefficient (N-M/(rad/s))


%% Initialize Arrays Holding State Variables
dt = 0.01;
t = 0:dt:10; % 10 seconds
theta = zeros(1,length(t));
omega = zeros(1,length(t));
current = zeros(1,length(t));
V = 12; % V
r = Kv*V;

%% Forward Euler Simulation

for k = 1:length(t)-1
    
    % Evaluate derivatives of all state variables
    % Speed is the integral of position
    thetadot = omega(k);
    
    % In first order model, ignore inductance. Current is calculated
    % from Ohm's Law
    current = (V - (omega(k)/Kv))/R;
    
    % Acceleration (omegadot)
    % Viscous friction model: friction torque proportional to speed
    omegadot = (1/J)*(Ki*current - D*omega(k)); % tau_L = D*omega
    
    
    % Forward Euler Method (First Order Taylor Series)
    theta(k+1) = theta(k) + dt*thetadot;
    omega(k+1) = omega(k) + dt*omegadot;
end

%% Plots
subplot(2,1,1)
plot(t,theta)
subplot(2,1,2)
hold on
plot(t,omega)
plot(t, r*ones(1,length(t)), 'r--')