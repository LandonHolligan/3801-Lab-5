%% Main function for 3801 lab 5
clear;clc;close all;
% initialize aircraft_parameters struct
run("ttwistor.m")

%% Problem 2.1
% initialize flight conditions
x0 = [0; 0; -1609.34; 0; 0; 0; 21; 0; 0; 0; 0; 0];
    % position      attitude velocity  omega
aircraft_surfaces = zeros(4,1);
wind_inertial = zeros(3,1);

% setup and call ode45
tspan = [0 20];
options = odeset("RelTol",10e-8,"AbsTol",10e-8);
[t,x] = ode45(@(t,x) AircraftEOM(t, x, aircraft_surfaces, wind_inertial, aircraft_parameters), tspan, x0, options);

% plot results
PlotAircraftSim(t, x, aircraft_surfaces, 1:6, "r")


%% Problem 2.2
% initialize flight conditions
x0 = [0; 0; -1800;
    0; rad2deg(0.02780); 0;
    20.99; 0; 0.5837;
    0; 0; 0];
aircraft_surfaces = [rad2deg(0.1079); 0; 0; 0.3182];
wind_inertial = zeros(3,1);

% setup and call ode45
[t,x] = ode45(@(t,x) AircraftEOM(t, x, aircraft_surfaces, wind_inertial, aircraft_parameters), tspan, x0, options);

% plot results
PlotAircraftSim(t, x, aircraft_surfaces, 1:6, "r")