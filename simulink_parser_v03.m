%% INPUT ------------------------------------------------------------------
close all; clear; clc;

script_input_2springs
% script_input_bridge


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

% Size of vector z
size_of_z = model.ndofs_ext + model.ndofs;


%% BUILD LOADING 
% Part for ff (applied forces): mux
xy = [0 0]; % Top coordinates
dx = 5; dy = 20+50*numel(model.loads_f);
x = xy(1); y = xy(2);

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/ff_mux'], ...
    'Inputs', num2str(numel(model.loads_f)), ...
    'Position', [x y x+dx y+dy]);


% Part for ff (applied forces): ff
ports = get_param([modelname '/ff_mux'], 'PortConnectivity');
pos = ports(end).Position;
dx = 40; dy = 30; % Size of block
x = pos(1)+50; y = pos(2)-0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/ff'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'ff');  


% Part for ff (applied forces): ff_input_i
for i = 1:numel(model.loads_f)
    dx = 40; dy = 40;
    xy = get_param([modelname '/ff_mux'], 'Position'); x = xy(1)-50-dx;
    pos = get_param([modelname '/ff_mux'], 'PortConnectivity');
    xy = pos(i).Position; y = xy(2) - 0.5*dy;
    
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

end


% -------------------------------------------------------------------------
% Part for xdd (applied displacements): mux
dx = 5; dy = 20+50*numel(model.loads_d);
pos_1 = get_param([modelname '/ff_mux'], 'Position'); 
x = pos_1(1); y = pos_1(4)+ 50;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/xdd_mux'], ...
    'Inputs', num2str(numel(model.loads_d)), ...
    'Position', [x y x+dx y+dy]);


% Part for xdd: xd_input_i
for i = 1:numel(model.loads_d)
    dx = 40; dy = 40;
    xy = get_param([modelname '/xdd_mux'], 'Position'); x = xy(1)-50-dx;
    pos = get_param([modelname '/xdd_mux'], 'PortConnectivity');
    xy = pos(i).Position; y = xy(2) - 0.5*dy;


    if model.loads_d{i}.Type == "Sine Wave"
        add_block('simulink/Sources/Sine Wave', ...
            [modelname '/xdd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Amplitude', num2str(model.loads_d{i}.Amplitude), ...
            'Bias', num2str(model.loads_d{i}.Bias), ...
            'Frequency', num2str(model.loads_d{i}.Frequency), ...
            'Phase', num2str(model.loads_d{i}.Phase));

    elseif model.loads_d{i}.Type == "Constant"
        add_block('simulink/Sources/Constant', ...
            [modelname '/xdd_input_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(model.loads_d{i}.Value));
    end   
end


% Part for xd (applied displacements): xdd
pos = get_param([modelname '/xdd_mux'], 'PortConnectivity');
xy = pos(end).Position;
dx = 40; dy = 30; % Size of block
x = xy(1)+50; y = xy(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xdd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xdd');  


% ------------------------------------------------------------------------
% Annotation area
pos_x1 = get_param([modelname '/ff_input_1'], 'Position');
pos_x2 = get_param([modelname '/xdd'], 'Position');
pos_y1 = get_param([modelname '/ff_mux'], 'Position');
pos_y2 = get_param([modelname '/xdd_mux'], 'Position');
dx = 25; dy = 25;

add_block('built-in/Area',[modelname '/APPLIED LOADING'],...
    'Position',[pos_x1(1)-dx pos_y1(2)-dy pos_x2(3)+dx pos_y2(4)+dy])



%% STATE VECTOR SELECTIONS
% Part for fd: Selector
pos1 = get_param([modelname '/xdd_mux'], 'Position');
pos2 = get_param([modelname '/APPLIED LOADING'], 'Position');

% xy = xd_input_conn{end}(1).Position; % Reference coordinates
dx = 40; dy = 10*size_of_z; % Size of block
x = 0.5*(pos1(1)+pos1(3)-dx); y = pos2(4)+125; % Base coordinates for block

idx_start = model.ndofs_ext + 1; % First index to select in state vector
idx_end = idx_start + height(model.dofs_d) - 1; % Final index to select in state vector
indices = idx_start:1:idx_end; % All indices to select in state vector

add_block('simulink/Signal Routing/Selector', ...
    [modelname '/fd_selector'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);


% Part for fd: state vector z
pos_x = get_param([modelname '/ff_input_1'], 'Position');
pos_y = get_param([modelname '/fd_selector'], 'PortConnectivity');
pos_y = pos_y(1).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = pos_x(1); y = pos_y(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/From', ...
    [modelname '/fd_z' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  


% Part for fd: fd
pos = get_param([modelname '/fd_selector'], 'PortConnectivity');
xy = pos(2).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = xy(1) + 50; y = xy(2) - 0.5*dy; % Base coordinates

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/fd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'fd');  


% -------------------------------------------------------------------------
% Part for xfd: Selector
pos = get_param([modelname '/fd_selector'], 'Position');
dx = 40; dy = 10*size_of_z;
x = pos(1); y = pos(4) + 25;

idx_start = model.ndofs_ext + height(model.dofs_d) + 1;
idx_end = idx_start + height(model.dofs_f) - 1;
indices = idx_start:1:idx_end;

add_block('simulink/Signal Routing/Selector', ...
    [modelname '/xfd_selector'], ...
    'Position', [x y x+dx y+dy], ...
    'InputPortWidth', num2str(size_of_z), ... 
    'Indices', ['[' num2str(reshape(indices', 1, [])) ']']);


% Part for xfd: State vector z
pos_x = get_param([modelname '/ff_input_1'], 'Position');
pos_y = get_param([modelname '/xfd_selector'], 'PortConnectivity');
pos_y = pos_y(1).Position; % Reference coordinates
dx = 40; dy = 30; % Size of block
x = pos_x(1); y = pos_y(2) - 0.5*dy; % Base coordinate for block

add_block('simulink/Signal Routing/From', ...
    [modelname '/xfd_z' ], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'z');  


% Part for xfd: xfd
pos = get_param([modelname '/xfd_selector'], 'PortConnectivity');
xy = pos(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/xfd'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'xfd');  


% -------------------------------------------------------------------------
% Update model structure to delete strings
model = rmfield(model,{'loads_f', 'loads_d'});


% ------------------------------------------------------------------------
% Annotation area
pos_x = get_param([modelname '/APPLIED LOADING'], 'Position');
pos_y1 = get_param([modelname '/fd_selector'], 'Position');
pos_y2 = get_param([modelname '/xfd_selector'], 'Position');
dy = 25;

add_block('built-in/Area',[modelname '/SELECT FROM RESULTING VECTOR'],...
    'Position',[pos_x(1) pos_y1(2)-dy pos_x(3) pos_y2(4)+dy])


%% BUILDING ELEMENT BLOCKS ------------------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Element block - properties
    pos = get_param([modelname '/xdd'], 'PortConnectivity');
    xy = pos(1).Position;
    dx = 100; dy = 275;
    x = xy(1)+450; y = (i-1)*(dy+100);

    % Element block
    blockname = ['elem', num2str(i)];
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [modelname '/' blockname], ...
        'Position', [x y x+dx y+dy]);

    % Attach function to element block
    S = sfroot;
    B = S.find('Name', blockname, '-isa', 'Stateflow.EMChart');  % Retrieves all open models
    B = B.find('Path', [modelname '/' blockname], '-isa', 'Stateflow.EMChart');  % Selects model in development
    B.Script = import_element(element{i}.type, ismember(i,model.PS(:,1)));

    % Remove field 'type' from element
    element{i} = rmfield(element{i}, 'type');

end


%% BUILD INPUTS FOR ELEMENT BLOCKS ----------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Part for element input: state vector x
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(1).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);


% -------------------------------------------------------------------------
    % Part for element input: lagrange selector
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(2).Position;
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


% -------------------------------------------------------------------------
    % Part for element input: lagrange multipliers
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy - 1.1*dy;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/lagrange multipliers ' num2str(i)],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['f' num2str(i)]);


% -------------------------------------------------------------------------
    % Part for element input: state vector z
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 300; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/z' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'z');    


% -------------------------------------------------------------------------
    % Part for element input: element structure 
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(3).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Sources/Constant', ...
        [modelname '/Elem_par' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/Elem_par' num2str(i)],...
        'Value', strcat('element{',num2str(i),'}'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;


% -------------------------------------------------------------------------
    % Restoring force by measurement
    if ismember(i,model.PS(:,1))
        pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
        xy = pos(4).Position;
        dx = 40; dy = 30;
        x = xy(1) - 100; y = xy(2) - 0.5*dy;
        add_block('simulink/Sources/Constant', ...
            [modelname '/R_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Value', num2str(0)) ;        
    end


end


%% BUILD OUTPUTS FOR ELEMENT BLOCKS ---------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Part for element output: integrator
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(end).Position;
    dx = 35; dy = 35;
    x = xy(1) + 50; y = xy(2) - 0.5*dy;

    add_block('simulink/Continuous/Integrator', ...
        [modelname '/Integrator' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InitialCondition', ['zeros(' num2str(height(element{i}.M+element{i}.nvars)) ',1)']); 

% -------------------------------------------------------------------------
    % Part for element output: state vector x
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(end).Position;
    dx = 40; dy = 30;
    x = xy(1) + 150; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) '_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i)]);  


% -------------------------------------------------------------------------
    % Part for element output: state derivative
    pos = get_param([modelname '/elem' num2str(i)], 'PortConnectivity');
    xy = pos(end).Position;
    dx = 40; dy = 30;
    x = xy(1) + 50; y = xy(2) - 0.5*dy + 50;

    add_block('simulink/Signal Routing/Goto', ...
        [modelname '/x' num2str(i) 'd_out'],...
            'Position', [x y x+dx y+dy], ...
            'GotoTag', ['x' num2str(i) 'd']);  


% -------------------------------------------------------------------------
% in case of PS ...
    if ismember(i,model.PS(:,1))
    % Part for element output: selection of PS node

        pos = get_param([modelname '/Integrator' num2str(i)], 'Position');
        dx = 40; dy = 40;

        idx = find(model.PS(:,1)==i);

        % Select reference node 1
        x = pos(1) + 75; y = pos(4)+30;
        add_block('simulink/Signal Routing/Selector', ...
            [modelname '/selectorPS_' num2str(i) '-1'], ...
            'Position', [x y x+dx y+dy], ...
            'InputPortWidth', num2str(height(element{i}.M+element{i}.nvars)), ... 
            'Indices', num2str(model.PS(idx,2)));  

        % Select reference node 2
        x = pos(1) + 75; y = pos(4)+100;
        add_block('simulink/Signal Routing/Selector', ...
            [modelname '/selectorPS_' num2str(i) '-2'], ...
            'Position', [x y x+dx y+dy], ...
            'InputPortWidth', num2str(height(element{i}.M+element{i}.nvars)), ... 
            'Indices', num2str(model.PS(idx,3)));   

        % Determine relative displacement
        x = pos(1) + 150; y = pos(4)+65;
        add_block('simulink/Math Operations/Sum', ...
            [modelname '/u_rel_' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'IconShape', 'rectangular', ...
            'Inputs', '+-')

    % Part for element output: selected displacement to impose on PS
        pos = get_param([modelname '/u_rel_' num2str(i)], 'Position');
        dx = 40; dy = 50;
        x = pos(3) + 50; y = mean(pos([2,4])) - dy - 15;

        add_block('simulink/Sinks/Display', ...
            [modelname '/uPS_' num2str(i)], ...
            'Position', [x y x+dx y+dy]);     

    % Part for element output: relative displacement to be applied on PS
        dx = 40; dy = 50;
        x = pos(3) + 50; y = mean(pos([2,4])) + 15 ;
        
        add_block('simulink/Sinks/Scope', ...
            [modelname '/uPS_plot' num2str(i)], ...
            'Position', [x y x+dx y+dy], ...
            'Title', 'Relative displacements', ...
            'ShowLegend', 'on', ...
            'YLabel', 'Displacements (m)')
    end
end


%% BUILD COUPLING BLOCK ---------------------------------------------------
% Coupling block - properties
pos = get_param([modelname '/x1_out'], 'Position');
dx = 400; dy = (4+numel(element))*110;
x = pos(3)+300; y = 0;

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



%% BUILD INPUTS FOR COUPLING BLOCK ----------------------------------------
% For every element ...
for i = 1:1:numel(element)
    % Part for coupling input: element structure
    pos = get_param([modelname '/coupling'], 'PortConnectivity');
    xy = pos(3*(i-1)+1).Position;
    dx = 80; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Sources/Constant', ...
        [modelname '/coupling elem' num2str(i)],...
            'Position', [x y x+dx y+dy]) ;
    set_param([modelname '/coupling elem' num2str(i)],...
        'Value', strcat('element{',num2str(i),'}'), ...
        'OutDataTypeStr',['Bus: ePar' num2str(i)]) ;


% -------------------------------------------------------------------------
    % Part for coupling input: state derivative xd
    pos = get_param([modelname '/coupling'], 'PortConnectivity');
    xy = pos(3*(i-1)+2).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling x' num2str(i) 'd'], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i) 'd']);


% -------------------------------------------------------------------------
    % Part for coupling input: lagrange multiplier
    pos = get_param([modelname '/coupling'], 'PortConnectivity');
    xy = pos(3*(i-1)+3).Position;
    dx = 40; dy = 30;
    x = xy(1) - 75; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/coupling f' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['f' num2str(i)]);

end


% -------------------------------------------------------------------------
% Part for coupling input: model structure
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(3*numel(element)+1).Position;
dx = 80; dy = 30;
x = xy(1) - 100; y = xy(2) - 0.5*dy;

add_block('simulink/Sources/Constant', ...
    [modelname '/coupling model'],...
        'Position', [x y x+dx y+dy]) ;
set_param([modelname '/coupling model'],...
    'Value', 'model', ...
    'OutDataTypeStr','Bus: mPar') ;


% -------------------------------------------------------------------------
% Part for coupling input: xdd
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(3*numel(element)+2).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xdd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xdd');


% -------------------------------------------------------------------------
% Part for coupling input: xfd
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(3*numel(element)+3).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling xfd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'xfd');


% -------------------------------------------------------------------------
% Part for coupling input: fd
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(3*numel(element)+4).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling fd'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'fd');


% -------------------------------------------------------------------------
% Part for coupling input: ff
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(3*numel(element)+5).Position;
dx = 40; dy = 30;
x = xy(1) - 75; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/coupling ff'], ...
    'Position', [x y x+dx y+dy], ...
    'GotoTag', 'ff');


%% BUILD OUTPUT FOR COUPLING BLOCK ----------------------------------------
% Part for coupling output: residuals
pos = get_param([modelname '/coupling'], 'PortConnectivity');
xy = pos(end).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/coupling_res'],...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', 'res');  



%% IMPOSE THE CONSTRAINT
% Part for constraint: impose the constraint
xy = get_param([modelname '/coupling'], 'Position');
dx = 60; dy = 40;
x = xy(1) + 0.5*(xy(3)-xy(1)-dx); y = xy(2) + xy(4) + 50;

add_block('simulink/Math Operations/Algebraic Constraint', ...
    [modelname '/impose the constraint'],...
    'Position', [x y x+dx y+dy]);


% -------------------------------------------------------------------------
% Part for constraint: residuals
pos = get_param([modelname '/impose the constraint'], 'PortConnectivity');
xy = pos(1).Position;
dx = 40; dy = 30;
x = xy(1) - dx - 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/From', ...
    [modelname '/res_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'res');


% -------------------------------------------------------------------------
% Part for constraint: state variable
pos = get_param([modelname '/impose the constraint'], 'PortConnectivity');
xy = pos(2).Position;
dx = 40; dy = 30;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Signal Routing/Goto', ...
    [modelname '/z_constraint'], ...
    'Position',  [x y x+dx y+dy], ...
    'GotoTag', 'z');



%% PLOT SECTION
% Displacements: mux 
pos = get_param([modelname '/coupling_res'], 'Position');
dx = 5; dy = numel(element)*100;
x = pos(3)+300; y = 0;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/d_mux'], ...
    'Inputs', num2str(numel(element)), ...
    'Position', [x y x+dx y+dy]);

% Displacements: selector & state vector x
for i = 1:1:numel(element)
    pos = get_param([modelname '/d_mux'], 'PortConnectivity');
    xy = pos(i).Position;
    dx = 40; dy = 10*element{i}.ndofs*2;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;    

    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/state element ' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(height(element{i}.M)), ... 
        'Indices', '[1 2]');

    xy = get_param([modelname '/state element ' num2str(i)], 'PortConnectivity').Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/d_x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);

end


% Displacements: scope
pos = get_param([modelname '/d_mux'], 'PortConnectivity');
xy = pos(end).Position;
dx = 40; dy = 50;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Sinks/Scope', ...
    [modelname '/displacements'], ...
    'Position', [x y x+dx y+dy], ...
    'Title', 'Displacements', ...
    'ShowLegend', 'on', ...
    'YLabel', 'Displacements (m)')


% -------------------------------------------------------------------------
% Velocities: mux 
xy = get_param([modelname '/d_mux'], 'Position');
dx = 5; dy = numel(element)*100;
x = xy(1); y = xy(4) + 50;

add_block('simulink/Signal Routing/Mux', ...
    [modelname '/v_mux'], ...
    'Inputs', num2str(numel(element)), ...
    'Position', [x y x+dx y+dy]);


% Velocities: selector & state vector x
for i = 1:1:numel(element)
    pos = get_param([modelname '/v_mux'], 'PortConnectivity');
    xy = pos(i).Position;
    dx = 40; dy = 10*element{i}.ndofs*2;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;    

    add_block('simulink/Signal Routing/Selector', ...
        [modelname '/v_state element ' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'InputPortWidth', num2str(height(element{i}.M)), ... 
        'Indices', '[3 4]');

    xy = get_param([modelname '/v_state element ' num2str(i)], 'PortConnectivity').Position;
    dx = 40; dy = 30;
    x = xy(1) - 100; y = xy(2) - 0.5*dy;

    add_block('simulink/Signal Routing/From', ...
        [modelname '/v_x' num2str(i)], ...
        'Position', [x y x+dx y+dy], ...
        'GotoTag', ['x' num2str(i)]);

end


% Displacements: scope
pos = get_param([modelname '/v_mux'], 'PortConnectivity');
xy = pos(end).Position;
dx = 40; dy = 50;
x = xy(1) + 50; y = xy(2) - 0.5*dy;

add_block('simulink/Sinks/Scope', ...
    [modelname '/velocities'], ...
    'Position', [x y x+dx y+dy], ...
    'Title', 'Velocities', ...
    'ShowLegend', 'on', ...
    'YLabel', 'Displacements (m)')


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
for i = 1:height(model.dofs_f)
    add_line(modelname, get_param([modelname '/ff_input_' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/ff_mux'], 'PortHandles').Inport(i));
end
add_line(modelname, get_param([modelname '/ff_mux'], 'PortHandles').Outport(1), get_param([modelname '/ff'], 'PortHandles').Inport(1));


% Applied loading - displacements
for i = 1:height(model.dofs_d)
    add_line(modelname, get_param([modelname '/xdd_input_' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/xdd_mux'], 'PortHandles').Inport(i));
end
add_line(modelname, get_param([modelname '/xdd_mux'], 'PortHandles').Outport(1), get_param([modelname '/xdd'], 'PortHandles').Inport(1), 'autorouting', 'on');

% State vector selections
add_line(modelname, get_param([modelname '/fd_z'], 'PortHandles').Outport(1), get_param([modelname '/fd_selector'], 'PortHandles').Inport(1));
add_line(modelname, get_param([modelname '/fd_selector'], 'PortHandles').Outport(1), get_param([modelname '/fd'], 'PortHandles').Inport(1));

add_line(modelname, get_param([modelname '/xfd_z'], 'PortHandles').Outport(1), get_param([modelname '/xfd_selector'], 'PortHandles').Inport(1));
add_line(modelname, get_param([modelname '/xfd_selector'], 'PortHandles').Outport(1), get_param([modelname '/xfd'], 'PortHandles').Inport(1));

% Constraint
add_line(modelname, get_param([modelname '/res_constraint'], 'PortHandles').Outport(1), get_param([modelname '/impose the constraint'], 'PortHandles').Inport(1));
add_line(modelname, get_param([modelname '/impose the constraint'], 'PortHandles').Outport(1), get_param([modelname '/z_constraint'], 'PortHandles').Inport(1));


for i = 1:1:numel(element)
    % State - element block
    add_line(modelname, get_param([modelname '/x' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/elem' num2str(i)], 'PortHandles').Inport(1));
    % Selector - element block
    add_line(modelname, get_param([modelname '/selector' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/elem' num2str(i)], 'PortHandles').Inport(2));
    % Selector - go to f
    add_line(modelname, get_param([modelname '/selector' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/lagrange multipliers ' num2str(i)], 'PortHandles').Inport(1), 'autorouting', 'on');
    % Vector z - selector
    add_line(modelname, get_param([modelname '/z' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/selector' num2str(i)], 'PortHandles').Inport(1));
    % Element structure - element block
    add_line(modelname, get_param([modelname '/Elem_par' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/elem' num2str(i)], 'PortHandles').Inport(3));
    % Element block - integrator
    add_line(modelname, get_param([modelname '/elem' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/Integrator' num2str(i)], 'PortHandles').Inport(1));
    % Integrator - state
    add_line(modelname, get_param([modelname '/Integrator' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/x' num2str(i) '_out'], 'PortHandles').Inport(1));
    % Element block - state derivative
    add_line(modelname, get_param([modelname '/elem' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/x' num2str(i) 'd_out'], 'PortHandles').Inport(1), 'autorouting', 'on');
    % Element structure - coupling block
    add_line(modelname, get_param([modelname '/coupling elem' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*(i-1)+1));
    % State derivative - coupling block
    add_line(modelname, get_param([modelname '/coupling x' num2str(i) 'd'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*(i-1)+2));
    % Lagrange multiplier - coupling block
    add_line(modelname, get_param([modelname '/coupling f' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*(i-1)+3));

    if ismember(i,model.PS(:,1))
        add_line(modelname, get_param([modelname '/R_' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/elem' num2str(i)], 'PortHandles').Inport(4));
        add_line(modelname, get_param([modelname '/Integrator' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/selectorPS_' num2str(i) '-1'], 'PortHandles').Inport(1), 'autorouting', 'on');
        add_line(modelname, get_param([modelname '/Integrator' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/selectorPS_' num2str(i) '-2'], 'PortHandles').Inport(1), 'autorouting', 'on');
        add_line(modelname, get_param([modelname '/selectorPS_' num2str(i) '-1'], 'PortHandles').Outport(1), get_param([modelname '/u_rel_' num2str(i)], 'PortHandles').Inport(1), 'autorouting', 'on');
        add_line(modelname, get_param([modelname '/selectorPS_' num2str(i) '-2'], 'PortHandles').Outport(1), get_param([modelname '/u_rel_' num2str(i)], 'PortHandles').Inport(2), 'autorouting', 'on');
        add_line(modelname, get_param([modelname '/u_rel_' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/uPS_' num2str(i)], 'PortHandles').Inport(1), 'autorouting', 'on');
        add_line(modelname, get_param([modelname '/u_rel_' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/uPS_plot' num2str(i)], 'PortHandles').Inport(1), 'autorouting', 'on');
    end

end


% Coupling block
% Model
add_line(modelname, get_param([modelname '/coupling model'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*numel(element)+1));
% xdd 
add_line(modelname, get_param([modelname '/coupling xdd'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*numel(element)+2));
% xfd 
add_line(modelname, get_param([modelname '/coupling xfd'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*numel(element)+3));
% fd 
add_line(modelname, get_param([modelname '/coupling fd'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*numel(element)+4));
% ff 
add_line(modelname, get_param([modelname '/coupling ff'], 'PortHandles').Outport(1), get_param([modelname '/coupling'], 'PortHandles').Inport(3*numel(element)+5));
% res 
add_line(modelname, get_param([modelname '/coupling'], 'PortHandles').Outport(1), get_param([modelname '/coupling_res'], 'PortHandles').Inport(1));


% Plots (Scopes)
% Displacements
for i = 1:numel(element)
    add_line(modelname, get_param([modelname '/d_x' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/state element ' num2str(i)], 'PortHandles').Inport(1));
    add_line(modelname, get_param([modelname '/state element ' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/d_mux'], 'PortHandles').Inport(i));
end
add_line(modelname, get_param([modelname '/d_mux'], 'PortHandles').Outport(1), get_param([modelname '/displacements'], 'PortHandles').Inport(1));
% Velocities
for i = 1:numel(element)
    add_line(modelname, get_param([modelname '/v_x' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/v_state element ' num2str(i)], 'PortHandles').Inport(1));
    add_line(modelname, get_param([modelname '/v_state element ' num2str(i)], 'PortHandles').Outport(1), get_param([modelname '/v_mux'], 'PortHandles').Inport(i));
end
add_line(modelname, get_param([modelname '/v_mux'], 'PortHandles').Outport(1), get_param([modelname '/velocities'], 'PortHandles').Inport(1));



%% FINALIZE ---------------------------------------------------------------
% Identify all model variables that are defined in the base workspace
varsToImport = Simulink.findVars(modelname,'SourceType','base workspace');
varNames = {varsToImport.Name};

% Set model parameters
set_param(gcs, 'Solver', 'ode4')
set_param(gcs, 'SolverType', 'Fixed-step');
set_param(gcs, 'FixedStep', 'model.dt');
set_param(gcs, 'StopTime', 'model.tmax');
set_param(gcs, 'SaveState', 'on')
set_param(gcs, 'AlgebraicLoopMsg', 'none') % To neglect algebraic loop warning
set_param(gcs, 'ZoomFactor', 'FitSystem');
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

% Save model
save_system(modelname);

% Output message
disp(['End of creating file: ' modelname])
clear;