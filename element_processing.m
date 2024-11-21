function element = element_processing(element)

% For each element ...
for i = 1:1:numel(element)

    % Check for required element parameters
    if ~isfield(element{i}, 'm') || ~isfield(element{i}, 'dofs')
        error('m and dofs has to be defined for each element.')
    end

    % Check for stiffness component
    if ~isfield(element{i}, 'k')
        element{i}.k = zeros(size(element{i}.m));
    end

    % Check for type parameters
    %    possible types: 
    %       (1) linear 
    %       (2) boucwen 
    %       (3) mostaghel
    switch element{i}.type
        case 'linear'
            element{i}.nvars = 0;

        case 'boucwen'
            element{i}.nvars = 1;

            if ~isfield(element{i}, 'alpha') 
                element{i}.alpha = 1.0 ; % linear stiffness component
            end 
            if ~isfield(element{i}, 'beta')
                element{i}.beta = element{i}.alpha/10; % nonlinear stiffness component
            end
            if ~isfield(element{i}, 'gamma') 
                element{i}.gamma = element{i}.beta/2; % nonlinear stiffness component
            end
            if ~isfield(element{i}, 'n') 
                element{i}.n = 1; % exponent for the hysteresis loop
            end

            % Variables, related to other types
            element{i}.d0 = 0;
            element{i}.d1 = 0;

        case 'mostaghel'
            element{i}.nvars = 1;
            if ~isfield(element{i}, 'd0')
                element{i}.d0 = 1.5e-3;
            end
            if ~isfield(element{i}, 'd1') 
                element{i}.d1 = 0.07;
            end

      
    end

    % Check for Rayleigh damping parameters
    if ~isfield(element{i}, 'a0')
        element{i}.a0 = 0;
    end
    if ~isfield(element{i}, 'a1') 
        element{i}.a1 = 0;
    end

    % Define number of dofs 
    element{i}.ndofs = height(element{i}.m);

    % Generalized mass matrix
    element{i}.M = blkdiag(eye(size(element{i}.m)),element{i}.m, eye(element{i}.nvars));
   
    % Proportional damping
    element{i}.c = element{i}.a0 * element{i}.m + element{i}.a1 * element{i}.k ;

end


end