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
surfaces_matrix = repmat(aircraft_surfaces,1,length(t));
PlotAircraftSim(t, x', surfaces_matrix, 1:6, "r")


%% Problem 2.2
% initialize flight conditions
x0 = [0; 0; -1800;
    0; 0.02780; 0;
    20.99; 0; 0.5837;
    0; 0; 0];
aircraft_surfaces = [0.1079; 0; 0; 0.3182];
wind_inertial = zeros(3,1);

% call ode45
[t,x] = ode45(@(t,x) AircraftEOM(t, x, aircraft_surfaces, wind_inertial, aircraft_parameters), tspan, x0, options);

% plot results
surfaces_matrix = repmat(aircraft_surfaces,1,length(t));
PlotAircraftSim(t, x', surfaces_matrix, 7:12, "r")


%% Problem 2.3 
% initialize flight conditions

x0 = [0; 0; -1800;
    deg2rad(15); deg2rad(-12); deg2rad(270);
    19; 3; -2;
    deg2rad(0.08); deg2rad(-0.2); 0];
aircraft_surfaces = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3];
wind_inertial = zeros(3,1);

% call ode45
[t,x] = ode45(@(t,x) AircraftEOM(t, x, aircraft_surfaces, wind_inertial, aircraft_parameters), tspan, x0, options);

% plot results
surfaces_matrix = repmat(aircraft_surfaces,1,length(t));
PlotAircraftSim(t, x', surfaces_matrix, 13:18, "r")

%% problem 3.1

tspan = [0 100];
options = odeset("RelTol",10e-8,"AbsTol",10e-8);


% initialize flight conditions
x0 = [0; 0; -1800;
    0; 0.02780; 0;
    20.99; 0; 0.5837;
    0; 0; 0];
aircraft_surfaces = [0.1079; 0; 0; 0.3182];
wind_inertial = zeros(3,1);
doublet_size = deg2rad(15);
doublet_time = 0.25;

% setup and call ode45
[tdot,xdot] = ode45(@(tspan,x0) AircraftEOMDoublet(tspan, x0, aircraft_surfaces, doublet_size, doublet_time, wind_inertial, aircraft_parameters), tspan, x0, options);

% plot results for 0.5 to 1.5 seconds
ind = (tdot(:,1) >= 0.5 & tdot(:,1) <= 1.5);
tdot_3_1 = tdot(ind);
xdot_3_1 = xdot(ind,:);
surfaces_matrix_3_1 = repmat(aircraft_surfaces,1,length(tdot_3_1));


PlotAircraftSim(tdot_3_1, transpose(xdot_3_1), surfaces_matrix_3_1, 19:24, "r")

%% Problem 3.2 
% plot results for 0.5 to 100 seconds
ind = (tdot(:,1) >= 0.5 & tdot(:,1) <= 100);
tdot_3_2 = tdot(ind);
xdot_3_2 = xdot(ind,:);
surfaces_matrix_3_2 = repmat(aircraft_surfaces,1,length(tdot_3_2));


PlotAircraftSim(tdot_3_2, transpose(xdot_3_2), surfaces_matrix_3_2, 25:30, "r")
