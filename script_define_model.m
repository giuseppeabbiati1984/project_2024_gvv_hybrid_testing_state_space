%% GENERAL - START



%% EXAMPLE PROPERTIES
% Load by running script of selected example
eval(['properties_example_' num2str(example) ';']);



%% GENERAL - END
% Model
model.dofs = sortrows(unique(vertcat(element.dofs), 'rows'));

% Number of degrees of freedom
for i = 1:1:numel(element)
    element(i).ndofs = height(element(i).dofs);
end
