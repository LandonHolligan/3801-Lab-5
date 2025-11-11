function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_state)

phi = aircraft_state(1); 
theta = aircraft_state(2); 
psi = aircraft_state(3);

cphi = cos(phi); 
sphi = sin(phi);
cth = cos(theta); 
sth = sin(theta);
cpsi = cos(psi); 
spsi = sin(psi);

R_BE = [cth*cpsi, sphi*sth*cpsi - cphi*spsi, cphi*sth*cpsi + sphi*spsi; cth*spsi, sphi*sth*spsi + cphi*cpsi, cphi*sth*spsi - sphi*cpsi; -sth, sphi*cth, cphi*cth];

wind_body = R_BE * wind_inertial;


end