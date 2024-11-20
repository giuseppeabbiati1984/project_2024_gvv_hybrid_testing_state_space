% Script to generate collocation matrices
% To be implemented in general parser
% ---------------------------------------
% close all; clc; clear;


%% B MATRICES
% Collocation matrix B
for i = 1:1:numel(element)
    element(i).B = [zeros(element(i).ndofs);eye(element(i).ndofs);zeros(element(i).nvars,element(i).ndofs)];
end


% Collocation matrix Bbar
for i = 1:1:numel(element)
    element(i).Bbar = zeros(height(model.dofs),element(i).ndofs);
    idx = find(ismember(model.dofs, element(i).dofs, "rows"));
    for j = 1:1:numel(idx)
        element(i).Bbar(idx(j),j) = -1;
    end
end


% Collocation matrix Bbar - displacement controlled
model.Bbar_d = zeros(height(model.dofs), height(model.dofs_d));  % Allocate
idx = find(ismember(model.dofs, model.dofs_d, "rows")); 
for i = 1:1:numel(idx)
    model.Bbar_d(idx(i),i) = 1;
end


% Collocation matrix Bbar - force controlled
model.Bbar_f = zeros(height(model.dofs), height(model.dofs_f));  % Allocate
idx = find(ismember(model.dofs, model.dofs_f, "rows")); 
for i = 1:1:numel(idx)
    model.Bbar_f(idx(i),i) = 1;
end


%% G MATRICES
% Certainly to be discussed!
% ...

% Collocation matrix G - displacement controlled
for i = 1:1:numel(element)
    idx = find(ismember(element(i).dofs, model.dofs_d, "rows"));
    element(i).G_d = zeros(max(height(idx), 1), 2*element(i).ndofs+element(i).nvars);

    for j = 1:1:numel(idx)
        element(i).G_d(j, idx(j)) = 1;
    end
end


% Collocation matrix G - force controlled
for i = 1:1:numel(element)
    idx = find(ismember(element(i).dofs, model.dofs_f, "rows"));
    element(i).G_f = zeros(max(height(idx), 1), 2*element(i).ndofs+element(i).nvars);

    for j = 1:1:numel(idx)
        element(i).G_f(j, idx(j)) = 1;
    end
end


% Collocation matrix Gbar - displacement controlled
for i = 1:1:numel(element)
    idx = find(ismember(model.dofs_d, element(i).dofs, "rows"));
    % element(i).Gbar_d = zeros(height(idx), height(model.dofs_d));
    element(i).Gbar_d = zeros(max(height(idx), 1), height(model.dofs_d));
    for j = 1:1:numel(idx)
        element(i).Gbar_d(j, idx(j)) = -1;
    end
end


% Collocation matrix Gbar - force controlled
for i = 1:1:numel(element)
    idx = find(ismember(model.dofs_f, element(i).dofs, "rows"));
    % element(i).Gbar_f = zeros(height(idx), height(model.dofs_f));
    element(i).Gbar_f = zeros(max(height(idx), 1), height(model.dofs_f));
    for j = 1:1:numel(idx)
        element(i).Gbar_f(j, idx(j)) = -1;
    end
end


clear i j idx
