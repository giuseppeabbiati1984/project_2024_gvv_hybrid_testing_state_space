function R = pierR_mostaghel(x,element)

% Allocate
R = zeros(element.n, 1);

% Retrieve
d = x(1:2);
v = x(3:4);
r = x(5);

d0 = element.pars.d0;
d1 = element.pars.d1;

% Generalized restoring force
% - state space trick
R(1:2) = -v;

% - balance equation
R(3:4) = element.k * d + element.c * v + [r; -r];

% - evolutionary equation restoring force
R(5) = (nbw((v(1)-v(2))) * mbw(r-(d0-d1)) + mw((v(1)-v(2))) * nw(r+(d0-d1))) * (v(1)-v(2));
% equation for a sdof
% dydt(3,1) = (nbw(y(2,1)) * mbw(y(3,1)-D) + mw(y(2,1)) * nw(y(3,1)+D)) * y(2,1); 


% - evolutionary equation of the dissipated energy
R(6) = -r * (v(1)-v(2));

end


function n = nw(w)

    n = 0.5 * (1 + sign(w)) * (1 + (1 - sign(w)));

end

function m = mw(w)

    m = 1 - nw(w);

end

function nb = nbw(w)

    nb = mw(-w);

end

function mb = mbw(w)

    mb = nw(-w);

end