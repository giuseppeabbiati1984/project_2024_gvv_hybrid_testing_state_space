%% INPUT ---------------------------------------------------n---------------
close all; clear; clc;


script_input_2springs
% script_input_bridge
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
ff_input_handles = cell(numel(model.loads_f),1);
xd_input_handles = cell(numel(model.loads_d),1);
state_handles = cell(numel(element),1);
selector_handles = cell(numel(element),1);
lagrange_handles = cell(numel(element),1);
z_handles = cell(numel(element),1);
elementStructure_handles = cell(numel(element),1);
elementBlock_conn = cell(numel(element),1);
elementBlock_handles = cell(numel(element),1);
integrator_handles = cell(numel(element),1);
state_out_handles = cell(numel(element),1);
state_d_out_handles = cell(numel(element),1);
coupling_element_handles = cell(numel(element),1);
coupling_xd_handles = cell(numel(element),1);
coupling_f_handles = cell(numel(element),1);
plot_d_selector_conn = cell(numel(element),1);
plot_d_selector_handles = cell(numel(element),1);
plot_d_x_handles = cell(numel(element),1);
plot_v_selector_conn = cell(numel(element),1);
plot_v_selector_handles = cell(numel(element),1);
plot_v_x_handles = cell(numel(element),1);


%% BUILD LOADING 
% Part for ff (applied forces): mux
xy = [0 0]; % Top coordinates
dx = 5; dy = 20+50*numel(model.loads_f);
x = xy(1); y = xy(2);

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/ff_mux'], ...
    'Inputs', num2str(numel(model.loads_f)), ...
    'Position', [x y x+dx y+dy]);

ff_mux_conn = get_param([modelname '/ff_mux'], 'PortConnectivity');
ff_mux_handles = get_param([modelname '/ff_mux'], 'PortHandles');


% Part for ff (applied forces): ff
pos = get_param([modelname '/ff_mux'], 'Position');
dx = 40; dy = 30; % Size of block
x = pos(3)+50; y = mean(pos([2,4]))-0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/ff'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'ff');  
ff_conn = get_param([modelname '/ff'], 'PortConnectivity');
ff_handles = get_param([modelname '/ff'], 'PortHandles');





% Part for ff (applied forces): ff_input_i
for i = 1:numel(model.loads_f)
    dx = 40; dy = 40;
    xy = get_param([modelname '/ff_mux'], 'Position'); x = xy(1)-50-dx;
    xy = ff_mux_conn(i).Position; y = xy(2) - 0.5*dy;
    
    if model.loads_f{i}.Type == "Sine Wave"
        add_block('simulink/Sources/Sine Wave', ...
            [modelname '/ff_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Amplitude', num2str(model.loads_f{i}.Amplitude), ...
            'Bias', num2str(model.loads_f{i}.Bias), ...
            'Frequency', num2str(model.loads_f{i}.Frequency), ...
            'Phase', num2str(model.loads_f{i}.Phase));

    elseif model.loads_f{i}.Type == "Constant"
        add_block('simulink/Sources/Constant', ...
            [modelname '/ff_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(model.loads_f{i}.Value));
    end

    ff_input_handles{i} = get_param([modelname '/ff_input_' num2str(i)], 'PortHandles');
end


% -------------------------------------------------------------------------
% Part for xd (applied displacements): mux
dx = 5; dy = 20+50*numel(model.loads_d);
pos_1 = get_param([modelname '/ff_mux'], 'Position'); 
x = pos(1); y = pos_1(4)+ 50;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/xd_mux'], ...
    'Inputs', num2str(numel(model.loads_d)), ...
    'Position', [x y x+dx y+dy]);
xd_mux_conn = get_param([modelname '/xd_mux'], 'PortConnectivity');
xd_mux_handles = get_param([modelname '/xd_mux'], 'PortHandles');


% Part for xd: xd_input_i
for i = 1:numel(model.loads_d)
    dx = 40; dy = 40;
    xy = get_param([modelname '/xd_mux'], 'Position'); x = xy(1)-50-dx;
    xy = xd_mux_conn(i).Position; y = xy(2) - 0.5*dy;


    if model.loads_d{i}.Type == "Sine Wave"
        add_block('simulink/Sources/Sine Wave', ...
            [modelname '/xd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Amplitude', num2str(model.loads_d{i}.Amplitude), ...
            'Bias', num2str(model.loads_d{i}.Bias), ...
            'Frequency', num2str(model.loads_d{i}.Frequency), ...
            'Phase', num2str(model.loads_d{i}.Phase));

    elseif model.loads_d{i}.Type == "Constant"
        add_block('simulink/Sources/Constant', ...
            [modelname '/xd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(model.loads_d{i}.Value));
    end

    xd_input_handles{i} = get_param([modelname '/xd_input_' num2str(i)], 'PortHandles');    
end


% Part for xd (applied displacements): derivative
xy = xd_mux_conn(end).Position;
dx = 40; dy = 40;
x = xy(1)+100; y = xy(2)-0.5*dy;
add_block('simulink/Continuous/Derivative', ...
    [modelname '/xd_derivative'], ...
    'Position', [x y x+dx y+dy])
xd_derivative_conn = get_param([modelname '/xd_derivative'], 'PortConnectivity');
xd_derivative_handles = get_param([modelname '/xd_derivative'], 'PortHandles');


% Part for xd (applied displacements): xd
xy = xd_mux_conn(end).Position;
dx = 40; dy = 30; % Size of block
x = xy(1)+50; y = xy(2) - 0.5*dy - 50; % Base coordinate for block

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


% ------------------------------------------------------------------------
% Annotation area
pos_x1 = get_param([modelname '/ff_input_1'], 'Position');
pos_x2 = get_param([modelname '/xdd'], 'Position');
pos_y1 = get_param([modelname '/ff_mux'], 'Position');
pos_y2 = get_param([modelname '/xd_mux'], 'Position');
dx = 25; dy = 25;

add_block('built-in/Area',[modelname '/APPLIED LOADING'],...
    'Position',[pos_x1(1)-dx pos_y1(2)-dy pos_x2(3)+dx pos_y2(4)+dy])



%% STATE VECTOR SELECTIONS
% Part for fd: Selector
pos1 = get_param([modelname '/xd_mux'], 'Position');
pos2 = get_param([modelname '/APPLIED LOADING'], 'Position');

% xy = xd_input_conn{end}(1).Position; % Reference coordinates
dx = 40; dy = 10*size_of_z; % Size of block
x = 0.5*(pos1(1)+pos1(3)-dx); y = pos2(4)+125; % Base coordinates for block

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
pos_x = get_param([modelname '/ff_input_1'], 'Position');
pos_y = fd_selector_conn(1).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = pos_x(1); y = pos_y(2) - 0.5*dy; % Base coordinate for block

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


% -------------------------------------------------------------------------
% Part for xfd: Selector
pos = get_param([modelname '/selector_fd'], 'Position');
dx = 40; dy = 10*size_of_z;
x = pos(1); y = pos(4) + 25;

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
pos_x = get_param([modelname '/ff_input_1'], 'Position');
pos_y = xfd_selector_conn(1).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = pos_x(1); y = pos_y(2) - 0.5*dy; % Base coordinate for block

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


% -------------------------------------------------------------------------
% Update model structure to delete strings
model = rmfield(model,{'loads_f', 'loads_d'});


% ------------------------------------------------------------------------
% Annotation area
pos_x = get_param([modelname '/APPLIED LOADING'], 'Position');
pos_y1 = get_param([modelname '/selector_fd'], 'Position');
pos_y2 = get_param([modelname '/selector_xfd'], 'Position');
dx = 25; dy = 25;

add_block('built-in/Area',[modelname '/SELECT FROM RESULTING VECTOR'],...
    'Position',[pos_x(1) pos_y1(2)-dy pos_x(3) pos_y2(4)+dy])


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

    % Remove string fields from element
    element{i} = rmfield(element{i}, 'type');

    % Store connections and handles
    elementBlock_conn{i} = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    elementBlock_handles{i} = get_param([modelname '/elem' num2str(i)], 'PortHandles');

end


%% BUILD INPUTS FOR ELEMENT BLOCKS ----------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Part for element input: state vector x
    xy = elementBlock_conn{i}(1).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);
    state_handles{i} = get_param([modelname '/x' num2str(i)], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for element input: lagrange selector
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

    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/selector' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(size_of_z), ... 
        'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);
    selector_handles{i} = get_param([modelname '/selector' num2str(i)], 'PortHandles');

% -------------------------------------------------------------------------
    % Part for element input: lagrange multipliers
    xy = elementBlock_conn{i}(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy - 1.1*dy;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/lagrange multipliers ' num2str(i)],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['f' num2str(i)]);
    lagrange_handles{i} = get_param([modelname '/lagrange multipliers ' num2str(i)], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for element input: state vector z
    xy = elementBlock_conn{i}(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 300; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/z' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'z');    
    z_handles{i} = get_param([modelname '/z' num2str(i)], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for element input: element structure 
    xy = elementBlock_conn{i}(3).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Sources/Constant', ...
        [modelname '/Elem_par' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/Elem_par' num2str(i)],...
        'Value', strcat('element{',num2str(i),'}'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;
    elementStructure_handles{i} = get_param([modelname '/Elem_par' num2str(i)], 'PortHandles');    

end


%% BUILD OUTPUTS FOR ELEMENT BLOCKS ---------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Part for element output: integrator
    xy = elementBlock_conn{i}(4).Position;
    dx = 35; dy = 35;
    x = xy(1) + 50; y = xy(2) - 0.5*dy;

    add_block('simulink/Continuous/Integrator', ...
        [modelname '/Integrator' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InitialCondition', ['zeros(' num2str(height(element{i}.M+element{i}.nvars)) ',1)']); 
    integrator_handles{i} = get_param([modelname '/Integrator' num2str(i)], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for element output: state vector x
    xy = elementBlock_conn{i}(4).Position;
    dx = 40; dy = 30;
    x = xy(1) + 150; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) '_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i)]);  
    state_out_handles{i} = get_param([modelname '/x' num2str(i) '_out'], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for element output: state derivative
    xy = elementBlock_conn{i}(4).Position;
    dx = 40; dy = 30;
    x = xy(1) + 50; y = xy(2) - 0.5*dy + 50;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) 'd_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i) 'd']);  
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
    % Part for coupling input: element structure
    xy = couplingBlock_conn(3*(i-1)+1).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Sources/Constant', ...
        [modelname '/coupling elem' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/coupling elem' num2str(i)],...
        'Value', strcat('element{',num2str(i),'}'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;
    coupling_element_handles{i} = get_param([modelname '/coupling elem' num2str(i)], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for coupling input: state derivative xd
    xy = couplingBlock_conn(3*(i-1)+2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling x' num2str(i) 'd'], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i) 'd']);
    coupling_xd_handles{i} = get_param([modelname '/coupling x' num2str(i) 'd'], 'PortHandles');


% -------------------------------------------------------------------------
    % Part for coupling input: lagrange multiplier
    xy = couplingBlock_conn(3*(i-1)+3).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling f' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['f' num2str(i)]);
    coupling_f_handles{i} = get_param([modelname '/coupling f' num2str(i)], 'PortHandles');

end


% -------------------------------------------------------------------------
% Part for coupling input: model structure
xy = couplingBlock_conn(3*numel(element)+1).Position;
dx = 80; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

add_block('simulink/Sources/Constant', ...
    [modelname '/coupling model'],...
        'Position', [x y x+dx y+dy]) ;
set_param([modelname '/coupling model'],...
    'Value', 'model', ...
    'OutDataTypeStr','Bus: mPar') ;
coupling_model_conn = get_param([modelname '/coupling model'], 'PortConnectivity');
coupling_model_handles = get_param([modelname '/coupling model'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for coupling input: xdd
xy = couplingBlock_conn(3*numel(element)+2).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xdd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xdd');
coupling_xdd_conn = get_param([modelname '/coupling xdd'], 'PortConnectivity');
coupling_xdd_handles = get_param([modelname '/coupling xdd'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for coupling input: xfd
xy = couplingBlock_conn(3*numel(element)+3).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xfd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xfd');
coupling_xfd_conn = get_param([modelname '/coupling xfd'], 'PortConnectivity');
coupling_xfd_handles = get_param([modelname '/coupling xfd'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for coupling input: fd
xy = couplingBlock_conn(3*numel(element)+4).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling fd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'fd');
coupling_fd_conn = get_param([modelname '/coupling fd'], 'PortConnectivity');
coupling_fd_handles = get_param([modelname '/coupling fd'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for coupling input: ff
xy = couplingBlock_conn(3*numel(element)+5).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling ff'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'ff');
coupling_ff_conn = get_param([modelname '/coupling ff'], 'PortConnectivity');
coupling_ff_handles = get_param([modelname '/coupling ff'], 'PortHandles');


%% BUILD OUTPUT FOR COUPLING BLOCK ----------------------------------------
% Part for coupling output: residuals
xy = couplingBlock_conn(end).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/coupling_res'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'res');  
coupling_res_conn = get_param([modelname '/coupling_res'], 'PortConnectivity');
coupling_res_handles = get_param([modelname '/coupling_res'], 'PortHandles');



%% IMPOSE THE CONSTRAINT
% Part for constraint: impose the constraint
xy = get_param([modelname '/coupling'], 'Position');
dx = 60; dy = 40;
x = xy(1) + 0.5*(xy(3)-xy(1)-dx); y = xy(2) + xy(4) + 50;

add_block('simulink/Math Operations/Algebraic Constraint', ...
    [modelname '/impose the constraint'],...
    'Position', [x y x+dx y+dy]);
constraint_conn = get_param([modelname '/impose the constraint'], 'PortConnectivity');
constraint_handles = get_param([modelname '/impose the constraint'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for constraint: residuals
xy = constraint_conn(1).Position;
dx = 40; dy = 30;
x = xy(1) - dx - 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/res_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'res');
res_constraint_conn = get_param([modelname '/res_constraint'], 'PortConnectivity');
res_constraint_handles = get_param([modelname '/res_constraint'], 'PortHandles');


% -------------------------------------------------------------------------
% Part for constraint: state variable
xy = constraint_conn(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/z_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'z');
z_constraint_conn = get_param([modelname '/z_constraint'], 'PortConnectivity');
z_constraint_handles = get_param([modelname '/z_constraint'], 'PortHandles');



%% PLOT SECTION
% Displacements: mux 
dx = 5; dy = numel(element)*100;
x = 2500; y = 0;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/d_mux'], ...
    'Inputs', num2str(numel(element)), ...
    'Position', [x y x+dx y+dy]);
plot_d_mux_conn = get_param([modelname '/d_mux'], 'PortConnectivity');
plot_d_mux_handles = get_param([modelname '/d_mux'], 'PortHandles');

% Displacements: selector & state vector x
for i = 1:1:numel(element)
    xy = plot_d_mux_conn(i).Position;
    dx = 40; dy = 10*element{i}.ndofs*2;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;    

    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/state element ' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(height(element{i}.M)), ... 
        'Indices', '[1 2]');
    plot_d_selector_conn{i} = get_param([modelname '/state element ' num2str(i)], 'PortConnectivity');
    plot_d_selector_handles{i} = get_param([modelname '/state element ' num2str(i)], 'PortHandles');

    xy = plot_d_selector_conn{i}.Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/d_x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);
    plot_d_x_handles{i} = get_param([modelname '/d_x' num2str(i)], 'PortHandles');

end

% Displacements: scope
xy = plot_d_mux_conn(end).Position;
dx = 40; dy = 50;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Sinks/Scope', ...
    [modelname '/displacements'], ...
    'Position', [x y x+dx y+dy], ...
    'Title', 'Displacements', ...
    'ShowLegend', 'on', ...
    'YLabel', 'Displacements (m)')
plot_d_scope_conn = get_param([modelname '/displacements'], 'PortConnectivity'); 
plot_d_scope_handles = get_param([modelname '/displacements'], 'PortHandles'); 


% -------------------------------------------------------------------------
% Velocities: mux 
xy = get_param([modelname '/d_mux'], 'Position');
dx = 5; dy = numel(element)*100;
x = xy(1); y = xy(4) + 50;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/v_mux'], ...
    'Inputs', num2str(numel(element)), ...
    'Position', [x y x+dx y+dy]);
plot_v_mux_conn = get_param([modelname '/v_mux'], 'PortConnectivity');
plot_v_mux_handles = get_param([modelname '/v_mux'], 'PortHandles');

% Velocities: selector & state vector x
for i = 1:1:numel(element)
    xy = plot_v_mux_conn(i).Position;
    dx = 40; dy = 10*element{i}.ndofs*2;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;    

    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/v_state element ' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(height(element{i}.M)), ... 
        'Indices', '[3 4]');
    plot_v_selector_conn{i} = get_param([modelname '/v_state element ' num2str(i)], 'PortConnectivity');
    plot_v_selector_handles{i} = get_param([modelname '/v_state element ' num2str(i)], 'PortHandles');

    xy = plot_v_selector_conn{i}.Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/v_x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);
    plot_v_x_handles{i} = get_param([modelname '/v_x' num2str(i)], 'PortHandles');

end

% Displacements: scope
xy = plot_v_mux_conn(end).Position;
dx = 40; dy = 50;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Sinks/Scope', ...
    [modelname '/velocities'], ...
    'Position', [x y x+dx y+dy], ...
    'Title', 'Velocities', ...
    'ShowLegend', 'on', ...
    'YLabel', 'Displacements (m)')
plot_v_scope_conn = get_param([modelname '/velocities'], 'PortConnectivity'); 
plot_v_scope_handles = get_param([modelname '/velocities'], 'PortHandles'); 


% ------------------------------------------------------------------------
% Annotation area
pos_x1 = get_param([modelname '/d_x1'], 'Position');
pos_x2 = get_param([modelname '/displacements'], 'Position');
pos_y1 = get_param([modelname '/d_mux'], 'Position');
pos_y2 = get_param([modelname '/v_mux'], 'Position');
dx = 25; dy = 25;

add_block('built-in/Area',[modelname '/PLOT DISPLACEMENTS AND VELOCITIES'],...
    'Position',[pos_x1(1)-dx pos_y1(2)-dy pos_x2(3)+dx pos_y2(4)+dy])



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


% Plots (Scopes)
% Displacements
for i = 1:numel(element)
    add_line(modelname, plot_d_x_handles{i}.Outport(1), plot_d_selector_handles{i}.Inport(1));
    add_line(modelname, plot_d_selector_handles{i}.Outport(1), plot_d_mux_handles.Inport(i));
end
add_line(modelname, plot_d_mux_handles.Outport(1), plot_d_scope_handles.Inport(1));
% Velocities
for i = 1:numel(element)
    add_line(modelname, plot_v_x_handles{i}.Outport(1), plot_v_selector_handles{i}.Inport(1));
    add_line(modelname, plot_v_selector_handles{i}.Outport(1), plot_v_mux_handles.Inport(i));
end
add_line(modelname, plot_v_mux_handles.Outport(1), plot_v_scope_handles.Inport(1));



%% FINALIZE ---------------------------------------------------------------
% Identify all model variables that are defined in the base workspace
varsToImport = Simulink.findVars(modelname,'SourceType','base workspace');
varNames = {varsToImport.Name};

% Create the data dictionary
dictionaryObj = Simulink.data.dictionary.create([modelname, '.sldd']);

% Import to the dictionary the model variables defined in the base workspace, and clear the variables from the base workspace
% [importSuccess,importFailure] = importFromBaseWorkspace(dictionaryObj,...
%  'varList',varNames,'clearWorkspaceVars',true);
[importSuccess,importFailure] = importFromBaseWorkspace(dictionaryObj,...
 'varList',varNames);

% Link the dictionary to the model
set_param(modelname,'DataDictionary',[modelname '.sldd']);

% Close dictionary
saveChanges(dictionaryObj);
close(dictionaryObj);

% Set model parameters
set_param(gcs, 'Solver', 'ode4')
set_param(gcs, 'SolverType', 'Fixed-step');
set_param(gcs, 'FixedStep', num2str(model.dt));
set_param(gcs, 'StopTime', num2str(model.tmax));
set_param(gcs, 'SaveState', 'on')
set_param(gcs, 'AlgebraicLoopMsg', 'none') % To neglect algebraic loop warning

% Save model
save_system(modelname);

disp(['End of creating file: ' modelname])

clear;