% Script to generate collocation matrices
% To be implemented in general parser
% ---------------------------------------
% close all; clc; clear;

function [element, model] = create_model(element, model)

%% Model
model.dofs = [];
model.ndofs_ext = 0;

for i = 1:numel(element)
    model.dofs = [model.dofs; element{i}.dofs];
    model.ndofs_ext = element{i}.ndofs + model.ndofs_ext;
end

model.dofs = sortrows(unique(model.dofs, "rows"));
model.ndofs = height(model.dofs);

% Verify if provided loading corresponds to defined dofs with loading
if numel(model.loads_f) ~= height(model.dofs_f)
    error('The number of load trajectories are not equal to the number of selected dofs with applied loading.')
end
if numel(model.loads_d) ~= height(model.dofs_d)
    error('The number of displacement trajectories are not equal to the number of selected dofs with applied displacement.')
end


%% B MATRICES
% Collocation matrix B
for i = 1:1:numel(element)
    element{i}.B = [zeros(element{i}.ndofs);eye(element{i}.ndofs);zeros(element{i}.nvars,element{i}.ndofs)];
end


% Collocation matrix Bbar
for i = 1:1:numel(element)
    element{i}.Bbar = zeros(height(model.dofs),element{i}.ndofs);
    idx = find_dofs(model.dofs, element{i}.dofs);
    for j = 1:1:numel(idx)
        element{i}.Bbar(idx(j),j) = -1;
    end
end


% Collocation matrix Bbar - displacement controlled
model.Bbar_d = zeros(height(model.dofs), height(model.dofs_d));  % Allocate
idx = find_dofs(model.dofs, model.dofs_d); 
for i = 1:1:numel(idx)
    model.Bbar_d(idx(i),i) = 1;
end


% Collocation matrix Bbar - force controlled
model.Bbar_f = zeros(height(model.dofs), height(model.dofs_f));  % Allocate
idx = find_dofs(model.dofs, model.dofs_f);
for i = 1:1:numel(idx)
    model.Bbar_f(idx(i),i) = 1;
end


%% G MATRICES
% Collocation matrix G - displacement controlled
for i = 1:1:numel(element)
    idx = find_dofs(element{i}.dofs, model.dofs_d);
    element{i}.G_d = zeros(max(height(idx), 1), 2*element{i}.ndofs+element{i}.nvars);

    for j = 1:1:numel(idx)
        element{i}.G_d(j, element{i}.ndofs+idx(j)) = 1;
    end
end


% Collocation matrix G - force controlled
for i = 1:1:numel(element)
    idx = find_dofs(element{i}.dofs, model.dofs_f);
    element{i}.G_f = zeros(max(height(idx), 1), 2*element{i}.ndofs+element{i}.nvars);
    for j = 1:1:numel(idx)
        element{i}.G_f(j, element{i}.ndofs+idx(j)) = 1;
    end
end


% Collocation matrix Gbar - displacement controlled
for i = 1:1:numel(element)
    idx = find_dofs(model.dofs_d, element{i}.dofs);
    % element{i}.Gbar_d = zeros(height(idx), height(model.dofs_d));
    element{i}.Gbar_d = zeros(max(height(idx), 1), height(model.dofs_d));
    for j = 1:1:numel(idx)
        element{i}.Gbar_d(j, idx(j)) = -1;
    end
end


% Collocation matrix Gbar - force controlled
for i = 1:1:numel(element)
    idx = find_dofs(model.dofs_f, element{i}.dofs);
    % element{i}.Gbar_f = zeros(height(idx), height(model.dofs_f));
    element{i}.Gbar_f = zeros(max(height(idx), 1), height(model.dofs_f));
    for j = 1:1:numel(idx)
        element{i}.Gbar_f(j, idx(j)) = -1;
    end
end


clear i j idx

end

function idx = find_dofs(dofs1, dofs2)

idx = find(ismember(dofs1, dofs2, "rows"));

end