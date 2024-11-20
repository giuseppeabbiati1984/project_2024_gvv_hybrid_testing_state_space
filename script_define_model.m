%% GENERAL - START



%% EXAMPLE PROPERTIES
% Load by running script of selected example
eval(['properties_example_' num2str(example) ';']);



%% GENERAL - END
% Model
model.dofs = sortrows(unique(vertcat(element.dofs), 'rows'));
