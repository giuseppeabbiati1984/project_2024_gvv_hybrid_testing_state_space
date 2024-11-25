%% INPUT ---------------------------------------------------n---------------
close all; clear; clc;


script_input_2springs
element = element_processing(element);
[element, model] = create_model(element, model);


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
    ePar_bus = Simulink.Bus.createObject(rmfield(element{i},'type')) ;
    eval(['ePar' num2str(i) ' = evalin("base", ePar_bus.busName);']) 
end


% Bus for model structure
mPar_bus = Simulink.Bus.createObject(rmfield(model,{'loads_f', 'loads_d'}));
mPar = evalin("base", mPar_bus.busName);


%% ALLOCATION -------------------------------------------------------------
% Determine size of state vector
size_of_z = model.ndofs_ext + model.ndofs;

% Allocate 
ff_input_conn = cell(numel(model.loads_f),1);
ff_input_handles = cell(numel(model.loads_f),1);

xd_input_conn = cell(numel(model.loads_d),1);
xd_input_handles = cell(numel(model.loads_d),1);

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


%% BUILD LOADING 
% Part for ff (applied forces): ff
xy = [250, 100]; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = xy(1); y = xy(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/ff'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'ff');  
ff_conn = get_param([modelname '/ff'], 'PortConnectivity');
ff_handles = get_param([modelname '/ff'], 'PortHandles');


% Part for ff (applied forces): mux
xy = ff_conn(1).Position; % Reference coordinates
dx = 5; dy = 20+50*numel(model.loads_f);
x = xy(1) - 25; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/ff_mux'], ...
    'Inputs', num2str(numel(model.loads_f)), ...
    'Position', [x y x+dx y+dy]);
ff_mux_conn = get_param([modelname '/ff_mux'], 'PortConnectivity');
ff_mux_handles = get_param([modelname '/ff_mux'], 'PortHandles');


% Part for ff (applied forces): ff_input_i
for i = 1:numel(model.loads_f)
    xy = ff_mux_conn(i).Position;
    dx = 40; dy = 40;
    x = xy(1) - 50 - 0.5*dx; y = xy(2) - 0.5*dy;

    if model.loads_f{i}.type == "Sine Wave"
        add_block('simulink/Sources/Sine Wave', ...
            [modelname '/ff_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Amplitude', num2str(model.loads_f{i}.Amplitude), ...
            'Bias', num2str(model.loads_f{i}.Bias), ...
            'Frequency', num2str(model.loads_f{i}.Frequency), ...
            'Phase', num2str(model.loads_f{i}.Phase));

    elseif model.loads_f{i}.type == "Constant"
        add_block('simulink/Sources/Constant', ...
            [modelname '/ff_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(model.loads_f{i}.Value));
    end

    ff_input_conn{i} = get_param([modelname '/ff_input_' num2str(i)], 'PortConnectivity');
    ff_input_handles{i} = get_param([modelname '/ff_input_' num2str(i)], 'PortHandles');
end

% -------------------------------------------------------------------------
% Part for xd: xd_input_i
for i = 1:numel(model.loads_d)
    xy = ff_input_conn{end}(1).Position;
    dx = 40; dy = 40;
    y = xy(2) - 0.5*dy + 100;

    if model.loads_d{i}.type == "Sine Wave"
        add_block('simulink/Sources/Sine Wave', ...
            [modelname '/xd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Amplitude', num2str(model.loads_f{i}.Amplitude), ...
            'Bias', num2str(model.loads_f{i}.Bias), ...
            'Frequency', num2str(model.loads_f{i}.Frequency), ...
            'Phase', num2str(model.loads_f{i}.Phase));

    elseif model.loads_d{i}.type == "Constant"
        add_block('simulink/Sources/Constant', ...
            [modelname '/xd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(model.loads_f{i}.Value));
    end

    xd_input_conn{i} = get_param([modelname '/xd_input_' num2str(i)], 'PortConnectivity');
    xd_input_handles{i} = get_param([modelname '/xd_input_' num2str(i)], 'PortHandles');    
end


% Part for xd (applied displacements): mux
x = ff_mux_conn(1).Position; % Reference coordinates
y1 = xd_input_conn{1}(1).Position; % Reference coordinates
y2 = xd_input_conn{end}(1).Position; % Reference coordinates
dx = 5; dy = 20+50*numel(model.loads_d);
x = x(1)+dx; y = 0.5*(y1(2)+y2(2))-0.5*dy;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/xd_mux'], ...
    'Inputs', num2str(numel(model.loads_d)), ...
    'Position', [x y x+dx y+dy]);
xd_mux_conn = get_param([modelname '/xd_mux'], 'PortConnectivity');
xd_mux_handles = get_param([modelname '/xd_mux'], 'PortHandles');


% Part for xd (applied displacements): derivative
xy = xd_mux_conn(1).Position;
dx = 40; dy = 40;
x = x(1)+100; y = xy(2)-0.5*dy;
add_block('simulink/Continuous/Derivative', ...
    [modelname '/xd_derivative'], ...
    'Position', [x y x+dx y+dy])
xd_derivative_conn = get_param([modelname '/xd_derivative'], 'PortConnectivity');
xd_derivative_handles = get_param([modelname '/xd_derivative'], 'PortHandles');


% Part for xd (applied displacements): xd
xy = xd_mux_conn(1).Position;
dx = 40; dy = 30; % Size of block
x = xy(1)+50; y = xy(2) - 0.5*dy + 50; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xd');  
xd_conn = get_param([modelname '/xd'], 'PortConnectivity');
xd_handles = get_param([modelname '/xd'], 'PortHandles');


% Part for xd (applied displacements): xdd
xy = xd_derivative_conn(1).Position;
dx = 40; dy = 30; % Size of block
x = xy(1)+dx+50; y = xy(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xdd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xdd');  
xdd_conn = get_param([modelname '/xdd'], 'PortConnectivity');
xdd_handles = get_param([modelname '/xdd'], 'PortHandles');


%% STATE VECTOR SELECTIONS

% Part for fd: Selector
xy = xd_input_conn{end}(1).Position; % Reference coordinates
dx = 40; dy = 10*size_of_z; % Size of block
x = xy(1); y = xy(2)+250; % Base coordinates for block

idx_start = model.ndofs_ext + 1; % First index to select in state vector
idx_end = idx_start + height(model.dofs_d) - 1; % Final index to select in state vector
indices = idx_start:1:idx_end; % All indices to select in state vector

add_block('simulink/Signal Routing/Selector', ...
    [modelname '/selector_fd'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);
fd_selector_conn = get_param([modelname '/selector_fd'], 'PortConnectivity');
fd_selector_handles = get_param([modelname '/selector_fd'], 'PortHandles');


% Part for fd: state vector z
xy = fd_selector_conn(1).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = xy(1) - 100; y = xy(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/From', ...
    [modelname '/z_fd' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  
fd_z_conn = get_param([modelname '/z_fd'], 'PortConnectivity');
fd_z_handles = get_param([modelname '/z_fd'], 'PortHandles');


% Part for fd: fd
xy = fd_selector_conn(2).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = xy(1) + 50; y = xy(2) - 0.5*dy; % Base coordinates

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/fd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'fd');  
fd_conn = get_param([modelname '/fd'], 'PortConnectivity');
fd_handles = get_param([modelname '/fd'], 'PortHandles');


% Part for xfd: Selector
xy = fd_selector_conn(1).Position;
dx = 40; dy = 10*size_of_z;
x = xy(1); y = xy(2) + dy + 25;

idx_start = model.ndofs_ext + height(model.dofs_d) + 1;
idx_end = idx_start + height(model.dofs_f) - 1;
indices = idx_start:1:idx_end;

add_block('simulink/Signal Routing/Selector', ...
    [modelname '/selector_xfd'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);
xfd_selector_conn = get_param([modelname '/selector_xfd'], 'PortConnectivity');
xfd_selector_handles = get_param([modelname '/selector_xfd'], 'PortHandles');


% Part for xfd: State vector z
xy = xfd_selector_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/xfd_z' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  
xfd_z_conn = get_param([modelname '/xfd_z'], 'PortConnectivity');
xfd_z_handles = get_param([modelname '/xfd_z'], 'PortHandles');


% Part for xfd: xfd
xy = xfd_selector_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xfd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xfd');  
xfd_conn = get_param([modelname '/xfd'], 'PortConnectivity');
xfd_handles = get_param([modelname '/xfd'], 'PortHandles');


% Update model structure to delete strings
model = rmfield(model,{'loads_f', 'loads_d'});


%% BUILDING ELEMENT BLOCKS ------------------------------------------------

% For every element ...
for i = 1:1:numel(element)

    % Element block - properties
    xy = xdd_conn(1).Position;
    dx = 100; dy = 250;
    x = xy(1)+500; y = (i-1)*(dy+100);

    % Element block
    blockname = ['elem', num2str(i)];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [modelname '/' blockname], ...
        'Position', [x y x+dx y+dy]);

    % Attach function to element block
    S = sfroot;
    B = S.find('Name', blockname, '-isa', 'Stateflow.EMChart');  % Retrieves all open models
    B = B.find('Path', [modelname '/' blockname], '-isa', 'Stateflow.EMChart');  % Selects model in development
    B.Script = import_element(element{i}.type);
    element{i} = rmfield(element{i}, 'type');

    % Store connections and handles
    elementBlock_conn{i} = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    elementBlock_handles{i} = get_param([modelname '/elem' num2str(i)], 'PortHandles');

end


%% BUILD INPUTS FOR ELEMENT BLOCKS ----------------------------------------
% ....

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
            idx_start = idx_start + height(element{j}.dofs);
        end
    end
    idx_end = idx_start + height(element{i}.dofs) - 1;
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
        'Value', strcat('element{',num2str(i),'}'), ...
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
        'InitialCondition', ['zeros(' num2str(height(element{i}.M+element{i}.nvars)) ',1)']); 


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


%% BUILD COUPLING BLOCK ---------------------------------------------------
% Coupling block - properties
dx = 400; dy = (4+numel(element))*110;
x = 1500; y = 0;

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
        'Value', strcat('element{',num2str(i),'}'), ...
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
% % % Mux - properties
% % dx = 5; dy = numel(element)*100;
% % x = 1750; y = 0;
% % 
% % % Mux
% % add_block('simulink/Signal Routing/Mux', ...
% %     [modelname '/mux'], ...
% %     'Inputs', num2str(numel(element)), ...
% %     'Position', [x y x+dx y+dy]);
% % 
% % % Store connections and handles
% % mux_conn = get_param([modelname '/mux'], 'PortConnectivity');
% % mux_handles = get_param([modelname '/mux'], 'PortHandles');
% % 
% % 
% % % 
% % for i = 1:1:numel(element)
% %     % Selector - properties
% %     xy = mux_conn(i).Position;
% %     dx = 40; dy = 10*element{i}.ndofs*2;
% %     x = xy(1) - 100; y = xy(2) - 0.5*dy;    
% % 
% %     % Selector
% %     add_block('simulink/Signal Routing/Selector', ...
% %         [modelname '/selector mux' num2str(i)], ...
% %         'Position', [x y x+dx y+dy], ...
% %         'InputPortWidth', num2str(4), ... 
% %         'Indices', ['[' num2str(1) ']']);
% % end
% 
% 
% 
%% BLOCK CONNECTIVITY -----------------------------------------------------
% Applied loading - forces
for i = 1:numel(ff_input_handles)
    add_line(modelname, ff_input_handles{i}.Outport(1), ff_mux_handles.Inport(i));
end
add_line(modelname, ff_mux_handles.Outport(1), ff_handles.Inport(1));

% Applied loading - displacements
for i = 1:numel(xd_input_handles)
    add_line(modelname, xd_input_handles{i}.Outport(1), xd_mux_handles.Inport(i));
end
add_line(modelname, xd_mux_handles.Outport(1), xd_derivative_handles.Inport(1));
add_line(modelname, xd_mux_handles.Outport(1), xd_handles.Inport(1), 'autorouting', 'on');
add_line(modelname, xd_derivative_handles.Outport(1), xdd_handles.Inport(1));



% State vector selections
add_line(modelname, fd_z_handles.Outport(1), fd_selector_handles.Inport(1));
add_line(modelname, fd_selector_handles.Outport(1), fd_handles.Inport(1));

add_line(modelname, xfd_z_handles.Outport(1), xfd_selector_handles.Inport(1));
add_line(modelname, xfd_selector_handles.Outport(1), xfd_handles.Inport(1));

% Constraint
add_line(modelname, res_constraint_handles.Outport(1), constraint_handles.Inport(1));
add_line(modelname, constraint_handles.Outport(1), z_constraint_handles.Inport(1));




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
set_param(gcs, 'SaveState', 'on')
set_param(gcs, 'AlgebraicLoopMsg', 'none') % To neglect algebraic loop warning

% Save model
save_system(modelname);

disp(['End of creating file: ' modelname])

clear;