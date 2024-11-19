%% INPUT ------------------------------------------------------------------
close all; clear; clc;

% ---
example = 5;

% ---
script_define_model
script_generate_collocation_matrices


% This will be the file name
modelname = ['Elem' num2str(numel(element)) '_D' num2str(sum([element.type]==1)) '_P' num2str(sum([element.type]==2)) '_NL' num2str(sum([element.nonlin]))];



%% INITIALIZATION ---------------------------------------------------------
Simulink.data.dictionary.closeAll('-save')

% Check if file already exists
if exist(modelname, 'file') == 4
    % Check if file is open if it exists
    if bdIsLoaded(modelname)
        % Close file if opened (without saving)
        close_system(modelname, 0)
    end
    % Delete file
    delete([modelname, '.slx']);
    delete([modelname, '.sldd']);
end


% Create system
new_system(modelname);


% Generating bus'
for i = 1:1:numel(element)
    % Element parameter bus
    ePar_bus = Simulink.Bus.createObject(element(i)) ;
    eval(['ePar' num2str(i) ' = evalin("base", ePar_bus.busName);']) 
end


% Bus for model structure
mPar_bus = Simulink.Bus.createObject(model);
mPar = evalin("base", mPar_bus.busName);

% Bus for element structure
ePar_bus = Simulink.Bus.createObject(element);
ePar = evalin("base", ePar_bus.busName);


%% ALLOCATION -------------------------------------------------------------
% Allocate 
state_conn = cell(numel(element),1);
state_handles = cell(numel(element),1);

selector_conn = cell(numel(element),1);
selector_handles = cell(numel(element),1);

lagrange_conn = cell(numel(element),1);
lagrange_handles = cell(numel(element),1);

z_conn = cell(numel(element),1);
z_handles = cell(numel(element),1);

elementStructure_conn = cell(numel(element),1);
elementStructure_handles = cell(numel(element),1);

elementBlock_conn = cell(numel(element),1);
elementBlock_handles = cell(numel(element),1);

integrator_conn = cell(numel(element),1);
integrator_handles = cell(numel(element),1);

state_out_conn = cell(numel(element),1);
state_out_handles = cell(numel(element),1);

scope_conn = cell(numel(element),1);
scope_handles = cell(numel(element),1);

coupling_element_conn = cell(numel(element),1);
coupling_element_handles = cell(numel(element),1);

coupling_xd_conn = cell(numel(element),1);
coupling_xd_handles = cell(numel(element),1);

coupling_f_conn = cell(numel(element),1);
coupling_f_handles = cell(numel(element),1);


%% BUILDING ELEMENT BLOCKS ------------------------------------------------

% For every element ...
for i = 1:1:numel(element)

    % Element block - properties
    dx = 100; dy = 250;
    x = 0; y = (i-1)*(dy+100);

    % Element block
    blockname = ['elem', num2str(i)];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [modelname '/' blockname], ...
        'Position', [x y x+dx y+dy]);

    % Attach function to element block
    S = sfroot;
    B = S.find('Name', blockname, '-isa', 'Stateflow.EMChart');  % Retrieves all open models
    B = B.find('Path', [modelname '/' blockname], '-isa', 'Stateflow.EMChart');  % Selects model in development
    B.Script = fun_element_model(element(i).type, element(i).nonlin);

    % Store connections and handles
    elementBlock_conn{i} = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    elementBlock_handles{i} = get_param([modelname '/elem' num2str(i)], 'PortHandles');

end



%% BUILD INPUTS FOR ELEMENT BLOCKS ----------------------------------------
% ....
number_of_lagrange_multipliers = height(sortrows(vertcat(element.dofs)));
number_of_unknowns = height(model.dofs);
size_of_z = number_of_lagrange_multipliers + number_of_unknowns;

% For every element ...
for i = 1:1:numel(element)

    % (1) state - properties
    xy = elementBlock_conn{i}(1).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    % (1) state
    add_block('simulink/Signal Routing/From', ...
        [modelname '/x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);


    % (2a) lagrange selector - properties
    xy = elementBlock_conn{i}(2).Position;
    dx = 40; dy = 10*size_of_z;
    x = xy(1) - 200; y = xy(2) - 0.5*dy;

    idx_start = 1;
    if i > 1
        for j = 1:i-1
            idx_start = idx_start + height(element(j).dofs);
        end
    end
    idx_end = idx_start + height(element(i).dofs) - 1;
    indices = idx_start:1:idx_end;

    % (2a) lagrange selector
    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/selector' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(size_of_z), ... 
        'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);


    % (2b) lagrange multipliers - properties
    xy = elementBlock_conn{i}(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy - 1.1*dy;

    % (2b) lagrange multipliers
    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/lagrange multipliers ' num2str(i)],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['f' num2str(i)]);


    % (2c) vector z - properties
    xy = elementBlock_conn{i}(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 300; y = xy(2) - 0.5*dy;

    % (2c) vector z
    add_block('simulink/Signal Routing/From', ...
        [modelname '/z' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'z');    


    % (3) element structure - properties
    xy = elementBlock_conn{i}(3).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    % (3) element structure
    add_block('simulink/Sources/Constant', ...
        [modelname '/Elem_par' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/Elem_par' num2str(i)],...
        'Value', strcat('element(',num2str(i),')'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;


    % Store connections and handles
    state_conn{i} = get_param([modelname '/x' num2str(i)], 'PortConnectivity');
    state_handles{i} = get_param([modelname '/x' num2str(i)], 'PortHandles');
  
    selector_conn{i} = get_param([modelname '/selector' num2str(i)], 'PortConnectivity');
    selector_handles{i} = get_param([modelname '/selector' num2str(i)], 'PortHandles');

    lagrange_conn{i} = get_param([modelname '/lagrange multipliers ' num2str(i)], 'PortConnectivity');
    lagrange_handles{i} = get_param([modelname '/lagrange multipliers ' num2str(i)], 'PortHandles');

    z_conn{i} = get_param([modelname '/z' num2str(i)], 'PortConnectivity');
    z_handles{i} = get_param([modelname '/z' num2str(i)], 'PortHandles');

    elementStructure_conn{i} = get_param([modelname '/Elem_par' num2str(i)], 'PortConnectivity');
    elementStructure_handles{i} = get_param([modelname '/Elem_par' num2str(i)], 'PortHandles');    

end


%% BUILD OUTPUTS FOR ELEMENT BLOCKS ---------------------------------------

% For every element ...
for i = 1:1:numel(element)

    % (1a) integrator - properties
    xy = elementBlock_conn{i}(4).Position;
    dx = 35; dy = 35;
    x = xy(1) + 50; y = xy(2) - 0.5*dy;

    % (1a) integrator
    add_block('simulink/Continuous/Integrator', ...
        [modelname '/Integrator' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InitialCondition', ['zeros(' num2str(element(i).n) ',1)']); 


    % (1b) state - properties
    xy = elementBlock_conn{i}(4).Position;
    dx = 40; dy = 30;
    x = xy(1) + 150; y = xy(2) - 0.5*dy;

    % (1b) state 
    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) '_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i)]);  


    % (1c) state derivative - properties
    xy = elementBlock_conn{i}(4).Position;
    dx = 40; dy = 30;
    x = xy(1) + 50; y = xy(2) - 0.5*dy + 50;

    % (1c) state derivative
    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) 'd_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i) 'd']);  


    % Store connections and handles
    integrator_conn{i} = get_param([modelname '/Integrator' num2str(i)], 'PortConnectivity');
    integrator_handles{i} = get_param([modelname '/Integrator' num2str(i)], 'PortHandles');

    state_out_conn{i} = get_param([modelname '/x' num2str(i) '_out'], 'PortConnectivity');
    state_out_handles{i} = get_param([modelname '/x' num2str(i) '_out'], 'PortHandles');

    state_d_out_conn{i} = get_param([modelname '/x' num2str(i) 'd_out'], 'PortConnectivity');
    state_d_out_handles{i} = get_param([modelname '/x' num2str(i) 'd_out'], 'PortHandles');
end
% 
% 
% %% SCOPES
% for i = 1:1:numel(element)
% 
%     % Scope - properties
%     xy = integrator_conn{i}(2).Position;
%     dx = 35; dy = 50;
%     x = xy(1) + 50; y = xy(2) - 0.5*dy;
% 
%     % Scope for x after integrator
%     add_block('simulink/Commonly Used Blocks/Scope', ...
%         [modelname '/Scope' num2str(i)], ...
%         'MakeNameUnique', 'on', ...
%         'Position', [x y x+dx y+dy])
% 
%     % Store connections and handles
%     scope_conn{i} = get_param([modelname '/Scope' num2str(i)], 'PortConnectivity');
%     scope_handles{i} = get_param([modelname '/Scope' num2str(i)], 'PortHandles');
% 
% end
% 
% 



%% BUILD COUPLING BLOCK ---------------------------------------------------
% Coupling block - properties
dx = 400; dy = (4+numel(element))*110;
x = 750; y = 0;

% Coupling block
blockname = 'coupling';
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [modelname '/' blockname], ...
    'Position', [x y x+dx y+dy]);

% Attach function to coupling block
S = sfroot;
B = S.find('Name', blockname, '-isa', 'Stateflow.EMChart');  % Retrieves all open models
B = B.find('Path', [modelname '/' blockname], '-isa', 'Stateflow.EMChart');  % Selects model in development
B.Script = fun_coupling_model(element);

% Store connections and handles
couplingBlock_conn = get_param([modelname '/coupling'], 'PortConnectivity');
couplingBlock_handles = get_param([modelname '/coupling'], 'PortHandles');
 

%% BUILD INPUTS FOR COUPLING BLOCK ----------------------------------------

% For every element ...
for i = 1:1:numel(element)
    % (1) Element - properties
    xy = couplingBlock_conn(3*(i-1)+1).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;
    
    % (1) Element structure
    add_block('simulink/Sources/Constant', ...
        [modelname '/coupling elem' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/coupling elem' num2str(i)],...
        'Value', strcat('element(',num2str(i),')'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;


    % (2) xd - properties
    xy = couplingBlock_conn(3*(i-1)+2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    % (2) xd 
    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling x' num2str(i) 'd'], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i) 'd']);


    % (3) f - properties
    xy = couplingBlock_conn(3*(i-1)+3).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    % (3) f
    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling f' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['f' num2str(i)]);


    % Store connections and handles
    coupling_element_conn{i} = get_param([modelname '/coupling elem' num2str(i)], 'PortConnectivity');
    coupling_element_handles{i} = get_param([modelname '/coupling elem' num2str(i)], 'PortHandles');

    coupling_xd_conn{i} = get_param([modelname '/coupling x' num2str(i) 'd'], 'PortConnectivity');
    coupling_xd_handles{i} = get_param([modelname '/coupling x' num2str(i) 'd'], 'PortHandles');

    coupling_f_conn{i} = get_param([modelname '/coupling f' num2str(i)], 'PortConnectivity');
    coupling_f_handles{i} = get_param([modelname '/coupling f' num2str(i)], 'PortHandles');

end
 

% (4) model structure - properties
xy = couplingBlock_conn(3*numel(element)+1).Position;
dx = 80; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

% (4) model structure
add_block('simulink/Sources/Constant', ...
    [modelname '/coupling model'],...
        'Position', [x y x+dx y+dy]) ;
set_param([modelname '/coupling model'],...
    'Value', 'model', ...
    'OutDataTypeStr','Bus: mPar') ;


% (5) xdd - properties
xy = couplingBlock_conn(3*numel(element)+2).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

% (5) xdd
add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xdd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xdd');


% (6) xfd - properties
xy = couplingBlock_conn(3*numel(element)+3).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

% (6) xfd
add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xfd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xfd');


% (7) fd - properties
xy = couplingBlock_conn(3*numel(element)+4).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

% (7) fd
add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling fd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'fd');


% (8) ff - properties
xy = couplingBlock_conn(3*numel(element)+5).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

% (8) ff
add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling ff'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'ff');


% Store connections and handles
coupling_model_conn = get_param([modelname '/coupling model'], 'PortConnectivity');
coupling_model_handles = get_param([modelname '/coupling model'], 'PortHandles');

coupling_xdd_conn = get_param([modelname '/coupling xdd'], 'PortConnectivity');
coupling_xdd_handles = get_param([modelname '/coupling xdd'], 'PortHandles');

coupling_xfd_conn = get_param([modelname '/coupling xfd'], 'PortConnectivity');
coupling_xfd_handles = get_param([modelname '/coupling xfd'], 'PortHandles');

coupling_fd_conn = get_param([modelname '/coupling fd'], 'PortConnectivity');
coupling_fd_handles = get_param([modelname '/coupling fd'], 'PortHandles');

coupling_ff_conn = get_param([modelname '/coupling ff'], 'PortConnectivity');
coupling_ff_handles = get_param([modelname '/coupling ff'], 'PortHandles');



%% BUILD OUTPUT FOR COUPLING BLOCK ----------------------------------------
% (1) residuals - properties
xy = couplingBlock_conn(end).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% (1) residuals
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/coupling_res'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'res');  


% Store connections and handles
coupling_res_conn = get_param([modelname '/coupling_res'], 'PortConnectivity');
coupling_res_handles = get_param([modelname '/coupling_res'], 'PortHandles');



%% BUILD LOADING 
% fd 
% -----
% selector - properties
xy = get_param([modelname '/selector' num2str(numel(element))], 'Position');
dx = 40; dy = 10*size_of_z;
x = xy(1); y = xy(2) + 250;

idx_start = number_of_lagrange_multipliers + 1;
idx_end = idx_start + height(model.dofs_d) - 1;
indices = idx_start:1:idx_end;

% selector
add_block('simulink/Signal Routing/Selector', ...
    [modelname '/selector_fd'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);

% Store connections and handles
selector_fd_conn = get_param([modelname '/selector_fd'], 'PortConnectivity');
selector_fd_handles = get_param([modelname '/selector_fd'], 'PortHandles');


% vector z - properties
xy = selector_fd_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

% vector z
add_block('simulink/Signal Routing/From', ...
    [modelname '/z_fd' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  

% fd - properties
xy = selector_fd_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% fd
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/fd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'fd');  


% xfd 
% -----
% selector - properties
xy = get_param([modelname '/selector_fd'], 'Position');
dx = 40; dy = 10*size_of_z;
x = xy(1); y = xy(2) + dy + 25;

idx_start = number_of_lagrange_multipliers + height(model.dofs_d) + 1;
idx_end = idx_start + height(model.dofs_f) - 1;
indices = idx_start:1:idx_end;

% selector
add_block('simulink/Signal Routing/Selector', ...
    [modelname '/selector_xfd'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);

% Store connections and handles
selector_xfd_conn = get_param([modelname '/selector_xfd'], 'PortConnectivity');
selector_xfd_handles = get_param([modelname '/selector_xfd'], 'PortHandles');


% vector z - properties
xy = selector_xfd_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

% vector z
add_block('simulink/Signal Routing/From', ...
    [modelname '/z_xfd' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  

% xfd - properties
xy = selector_xfd_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% fd
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xfd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xfd');  

% Store connections and handles
z_fd_conn = get_param([modelname '/z_fd'], 'PortConnectivity');
z_fd_handles = get_param([modelname '/z_fd'], 'PortHandles');

z_xfd_conn = get_param([modelname '/z_xfd'], 'PortConnectivity');
z_xfd_handles = get_param([modelname '/z_xfd'], 'PortHandles');

fd_conn = get_param([modelname '/fd'], 'PortConnectivity');
fd_handles = get_param([modelname '/fd'], 'PortHandles');

xfd_conn = get_param([modelname '/xfd'], 'PortConnectivity');
xfd_handles = get_param([modelname '/xfd'], 'PortHandles');


% ff
% ------
% Input - properties
xy1 = elementBlock_conn{end}(1).Position;
xy2 = selector_fd_conn(2).Position;
dx = 40; dy = 30;
x = xy1(1); y = xy2(2) - 0.5*dy;

% Input
add_block('simulink/Sources/Constant', ...
    [modelname '/ff_input'], ...
    'Position', [x y x+dx y+dy], ...
    'Value', ['zeros(' num2str(height(model.dofs_f)) ',1)']);

% Store connections and handles
input_ff_conn = get_param([modelname '/ff_input'], 'PortConnectivity');
input_ff_handles = get_param([modelname '/ff_input'], 'PortHandles');


% ff - properties
xy = input_ff_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% ff
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/ff'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'ff');  

% Store connections and handles
ff_conn = get_param([modelname '/ff'], 'PortConnectivity');
ff_handles = get_param([modelname '/ff'], 'PortHandles');


% xd
% ------
% Input - properties
xy1 = elementBlock_conn{end}(1).Position;
xy2 = selector_xfd_conn(2).Position;
dx = 40; dy = 30;
x = xy1(1); y = xy2(2) - 0.5*dy;

% Input
add_block('simulink/Sources/Constant', ...
    [modelname '/xd_input'], ...
    'Position', [x y x+dx y+dy], ...
    'Value', ['zeros(' num2str(height(model.dofs_d)) ',1)']);

% Store connections and handles
input_xd_conn = get_param([modelname '/xd_input'], 'PortConnectivity');
input_xd_handles = get_param([modelname '/xd_input'], 'PortHandles');


% xd - properties
xy = input_xd_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy + 75;

% xd
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xd');  

% Store connections and handles
xd_conn = get_param([modelname '/xd'], 'PortConnectivity');
xd_handles = get_param([modelname '/xd'], 'PortHandles');


% integrator - properties
xy = input_xd_conn(1).Position;
dx = 35; dy = 35;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% integrator
add_block('simulink/Continuous/Integrator', ...
    [modelname '/integrator_xd'], ...
    'Position', [x y x+dx y+dy], ...
    'InitialCondition', ['zeros(' num2str(height(model.dofs_d)) ',1)']); 

% Store connections and handles
integrator_xd_conn = get_param([modelname '/integrator_xd'], 'PortConnectivity');
integrator_xd_handles = get_param([modelname '/integrator_xd'], 'PortHandles');


% xdd - properties
xy = integrator_xd_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% xdd
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xdd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xdd');  

% Store connections and handles
xdd_conn = get_param([modelname '/xdd'], 'PortConnectivity');
xdd_handles = get_param([modelname '/xdd'], 'PortHandles');



%% IMPOSE THE CONSTRAINT
% Constraint - properties
xy = get_param([modelname '/coupling'], 'Position');
dx = 60; dy = 40;
x = xy(1) + 0.5*(xy(3)-xy(1)-dx); y = xy(2) + xy(4) + 50;

% Constraint block
add_block('simulink/Math Operations/Algebraic Constraint', ...
    [modelname '/impose the constraint'],...
    'Position', [x y x+dx y+dy]);

% Store connections and handles
constraint_conn = get_param([modelname '/impose the constraint'], 'PortConnectivity');
constraint_handles = get_param([modelname '/impose the constraint'], 'PortHandles');


% Residuals - properties
xy = constraint_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) - dx - 50; y = xy(2) - 0.5*dy;

% Residuals
add_block('simulink/Signal Routing/From', ...
    [modelname '/res_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'res');

% State variable - properties
xy = constraint_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

% State variable
add_block('simulink/Signal Routing/Goto', ...
    [modelname '/z_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'z');

% Store connections and handles
res_constraint_conn = get_param([modelname '/res_constraint'], 'PortConnectivity');
res_constraint_handles = get_param([modelname '/res_constraint'], 'PortHandles');
z_constraint_conn = get_param([modelname '/z_constraint'], 'PortConnectivity');
z_constraint_handles = get_param([modelname '/z_constraint'], 'PortHandles');



%% PLOT SECTION
% % Mux - properties
% dx = 5; dy = numel(element)*100;
% x = 1750; y = 0;
% 
% % Mux
% add_block('simulink/Signal Routing/Mux', ...
%     [modelname '/mux'], ...
%     'Inputs', num2str(numel(element)), ...
%     'Position', [x y x+dx y+dy]);
% 
% % Store connections and handles
% mux_conn = get_param([modelname '/mux'], 'PortConnectivity');
% mux_handles = get_param([modelname '/mux'], 'PortHandles');
% 
% 
% % 
% for i = 1:1:numel(element)
%     % Selector - properties
%     xy = mux_conn(i).Position;
%     dx = 40; dy = 10*element(i).ndofs*2;
%     x = xy(1) - 100; y = xy(2) - 0.5*dy;    
% 
%     % Selector
%     add_block('simulink/Signal Routing/Selector', ...
%         [modelname '/selector mux' num2str(i)], ...
%         'Position', [x y x+dx y+dy], ...
%         'InputPortWidth', num2str(4), ... 
%         'Indices', ['[' num2str(1) ']']);
% end



%% BLOCK CONNECTIVITY -----------------------------------------------------
for i = 1:1:numel(element)
    % Element blocks
    % -----
    % State - element block
    add_line(modelname, state_handles{i}.Outport(1), elementBlock_handles{i}.Inport(1));

    % Selector - element block
    add_line(modelname, selector_handles{i}.Outport(1), elementBlock_handles{i}.Inport(2));

    % Selector - go to f
    add_line(modelname, selector_handles{i}.Outport(1), lagrange_handles{i}.Inport(1), 'autorouting', 'on');

    % Vector z - selector
    add_line(modelname, z_handles{i}.Outport(1), selector_handles{i}.Inport(1));

    % Element structure - element block
    add_line(modelname, elementStructure_handles{i}.Outport(1), elementBlock_handles{i}.Inport(3));

    % Element block - integrator
    add_line(modelname, elementBlock_handles{i}.Outport(1), integrator_handles{i}.Inport(1));

    % Integrator - state
    add_line(modelname, integrator_handles{i}.Outport(1), state_out_handles{i}.Inport(1));

    % Element block - state derivative
    add_line(modelname, elementBlock_handles{i}.Outport(1), state_d_out_handles{i}.Inport(1), 'autorouting', 'on');



    % Coupling block
    % ------
    % Element structure - coupling block
    add_line(modelname, coupling_element_handles{i}.Outport(1), couplingBlock_handles.Inport(3*(i-1)+1));

    % State derivative - coupling block
    add_line(modelname, coupling_xd_handles{i}.Outport(1), couplingBlock_handles.Inport(3*(i-1)+2));

    % Lagrange multiplier - coupling block
    add_line(modelname, coupling_f_handles{i}.Outport(1), couplingBlock_handles.Inport(3*(i-1)+3));


end


% Coupling block
% -----
% Model
add_line(modelname, coupling_model_handles.Outport(1), couplingBlock_handles.Inport(3*numel(element)+1));
% xdd 
add_line(modelname, coupling_xdd_handles.Outport(1), couplingBlock_handles.Inport(3*numel(element)+2));
% xfd 
add_line(modelname, coupling_xfd_handles.Outport(1), couplingBlock_handles.Inport(3*numel(element)+3));
% fd 
add_line(modelname, coupling_fd_handles.Outport(1), couplingBlock_handles.Inport(3*numel(element)+4));
% ff 
add_line(modelname, coupling_ff_handles.Outport(1), couplingBlock_handles.Inport(3*numel(element)+5));
% res 
add_line(modelname, couplingBlock_handles.Outport(1), coupling_res_handles.Inport(1));



% Loading
% ------
add_line(modelname, z_fd_handles.Outport(1), selector_fd_handles.Inport(1));
add_line(modelname, z_xfd_handles.Outport(1), selector_xfd_handles.Inport(1));

add_line(modelname, selector_fd_handles.Outport(1), fd_handles.Inport(1));
add_line(modelname, selector_xfd_handles.Outport(1), xfd_handles.Inport(1));

add_line(modelname, input_ff_handles.Outport(1), ff_handles.Inport(1));
add_line(modelname, input_xd_handles.Outport(1), xd_handles.Inport(1), 'autorouting', 'on');
add_line(modelname, input_xd_handles.Outport(1), integrator_xd_handles.Inport(1));
add_line(modelname, integrator_xd_handles.Outport(1), xdd_handles.Inport(1));

add_line(modelname, res_constraint_handles.Outport(1), constraint_handles.Inport(1));
add_line(modelname, constraint_handles.Outport(1), z_constraint_handles.Inport(1));


%% FINALIZE ---------------------------------------------------------------
% Identify all model variables that are defined in the base workspace
varsToImport = Simulink.findVars(modelname,'SourceType','base workspace');
varNames = {varsToImport.Name};

% Create the data dictionary
dictionaryObj = Simulink.data.dictionary.create([modelname, '.sldd']);

% Import to the dictionary the model variables defined in the base workspace, and clear the variables from the base workspace
[importSuccess,importFailure] = importFromBaseWorkspace(dictionaryObj,...
 'varList',varNames,'clearWorkspaceVars',true);

% Link the dictionary to the model
set_param(modelname,'DataDictionary',[modelname '.sldd']);

% Close dictionary
saveChanges(dictionaryObj);
close(dictionaryObj);

% Set model parameters
set_param(gcs, 'Solver', 'ode4')

% Save model
save_system(modelname);

disp(['End of creating file: ' modelname])

clear;