%% GENERAL - START



%% EXAMPLE PROPERTIES
% Load by running script of selected example
eval(['properties_example_' num2str(example) ';']);



%% GENERAL - END
% Model
model.dofs = [];
model.ndofs_ext = 0;

for i = 1:numel(element)
    model.dofs = [model.dofs; element{i}.dofs];
    model.ndofs_ext = element{i}.ndofs + model.ndofs_ext;
end

model.dofs = sortrows(unique(model.dofs, "rows"));
