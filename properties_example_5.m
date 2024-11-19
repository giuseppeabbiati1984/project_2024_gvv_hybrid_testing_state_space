% This model represents a bridge with 3 deck elements, using Mostaghel
% springs to mimic seismic isolators. The bridge contains 2 piers, where
% one pier will be PS.

element = struct([]);

% Load into Workspace
% - deck elements (type 1)
for i = 1:3
    element(i).m = 1e2*eye(2);
    element(i).k = 10*[1 -1;-1 1];
    element(i).type = 1;
    element(i).type1 = 'linear';
end

% - pier elements: seismic isolators (type 3)
for i = 4:7
    element(i).m = 1*[1 0; 0 1];
    element(i).k = 1*[1 -1;-1 1];
    element(i).pars.d0 = 1.5e-3;
    element(i).pars.d1 = 0.07;
    element(i).type = 3;
end

% - pier elements: PS 
for i = 8
    element(i).m = 1*[1 0; 0 0.1];
    element(i).k = 0*[1 -1;-1 1];
    element(i).pars.d0 = 1.5e-3;
    element(i).pars.d1 = 0.07;
    element(i).type = 3;

end

% - pier elements: NS linear (type 3)
for i = 9
    element(i).m = 1*[1 0; 0 0.1];
    element(i).k = 1*[1 -1;-1 1];
    % element(i).pars.alpha = 1.0 ;  % linear stiffness component
    % element(i).pars.beta = 1.0 ;  % nonlinear stiffness component
    % element(i).pars.gamma = 1.0 ;  % nonlinear stiffness component
    % element(i).pars.n = 2;  % exponent for the hysteresis loop
    element(i).type = 3;
end


% Elements
element(1).dofs = [1,1; 2,1];
element(2).dofs = [2,1; 3,1];
element(3).dofs = [3,1; 4,1];
element(4).dofs = [1,1; 5,1];
element(5).dofs = [2,1; 6,1];
element(6).dofs = [3,1; 7,1];
element(7).dofs = [4,1; 8,1];
element(8).dofs = [6,1; 9,1];
element(9).dofs = [7,1; 10,1];


% Model
model.dofs_f = [1,1; 2,1; 3,1; 4,1; 6,1; 7,1];
model.dofs_d = [5,1; 8,1; 9,1; 10,1];