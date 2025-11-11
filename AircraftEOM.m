function xdot = AircraftEOM(time, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

density = stdatmo(abs(aircraft_state(3)));
ap = aircraft_parameters;
[aero_forces, aero_moments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);


phi = aircraft_state(4); 
theta = aircraft_state(5); 
psi = aircraft_state(6);

u = aircraft_state(7); 
v = aircraft_state(8); 
w = aircraft_state(9);

p = aircraft_state(10); 
q = aircraft_state(11); 
r = aircraft_state(12);

cphi = cos(phi); 
sphi = sin(phi);
cth = cos(theta); 
sth = sin(theta);
cpsi = cos(psi); 
spsi = sin(psi);

R_BE = [cth*cpsi, sphi*sth*cpsi - cphi*spsi, cphi*sth*cpsi + sphi*spsi; cth*spsi, sphi*sth*spsi + cphi*cpsi, cphi*sth*spsi - sphi*cpsi; -sth, sphi*cth, cphi*cth];

X = aero_forces(1);
Y = aero_forces(2);
Z = aero_forces(3);

L = aero_moments(1);
M = aero_moments(2);
N = aero_moments(3);

u_dot = r * v - q * w - ap.g * sth + (1 / ap.m) * X;
v_dot = p * w - r * u + ap.g * sphi * cth + (1 / ap.m) * Y;
w_dot = q * u - p * v + ap.g * cphi * cth + (1 / ap.m) * Z;

gamma = ap.Ix * ap.Iz - ap.Ixz^2;
gamma1 = (ap.Ixz*(ap.Ix-ap.Iy+ap.Iz))/ gamma;
gamma2 = (ap.Iz*(ap.Iz - ap.Iy)+ap.Ixz^2) / gamma;
gamma3 = ap.Iz / gamma;
gamma4 = ap.Ixz / gamma;
gamma5 = (ap.Iz - ap.Ix) / ap.Iy;
gamma6 = ap.Ixz / ap.Iy;
gamma7 = (ap.Ix*(ap.Ix - ap.Iy)+ap.Ixz^2)/gamma;
gamma8 = ap.Ix / gamma;

p_dot = gamma1 * p * q - gamma2 * q * r + gamma3 * L + gamma4 * N;
q_dot =gamma5 * p * r - gamma6*(p^2 - r^2) + (M / ap.Iy);
r_dot = gamma7 * p * q - gamma1 * q * r + gamma4 * L + gamma8 * N;

phi_dot = p + q * (sphi * tan(theta)) + r * (cphi * tan(theta));
theta_dot = q * cphi - r * sphi;
psi_dot = q * (sphi * sec(theta)) + r * (cphi * sec(theta));

pos_dot = R_BE * [u; v; w];

xdot = [pos_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end