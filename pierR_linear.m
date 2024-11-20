function R = pierR_linear(x,element)

% Allocate
R = zeros(height(element.M), 1);

% Retrieve
d = x(1:2);
v = x(3:4);

% Generalized restoring force
% - state space trick
R(1:2) = -v;

% - balance equation
R(3:4) = element.k * d + element.c * v;


end