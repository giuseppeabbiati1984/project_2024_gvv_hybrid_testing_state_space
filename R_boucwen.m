function R = R_boucwen(x,element)

% Allocate
R = zeros(height(element.M), 1);

% Retrieve
d = x(1:2);
v = x(3:4);
r = x(5);

alpha = element.alpha;
beta = element.beta;
gamma = element.gamma;
n = element.n;

% Generalized restoring force
% - state space trick
R(1:2) = -v;

% - balance equation
R(3:4) = element.k * d + element.c * v + [r; -r];

% - evolutionary equation restoring force
dv = v(1) - v(2); 
R(5) = - (alpha - (beta * sign(r*dv) + gamma) * abs(r)^n) * dv;
% R(5) = (alpha - (beta * sign(r*dv) + gamma) * abs(r)^n) * dv;

% - evolutionary equation of the dissipated energy
% R(6) = -r * (v(1)-v(2));


end