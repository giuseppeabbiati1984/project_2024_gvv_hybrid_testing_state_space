function element = element_processing(element)
% mass, damping and stiffness matrices
for i = 1:1:numel(element)
    if ~isfield(element(i), 'm') || ~isfield(element(i), 'dofs')
        error('m and dofs has to be defined for each element.')
    end

    if ~isfield(element(i), 'k')
        element(i).k = zeros(size(element(i).m));
    end

    switch element(i).type1 
        case 'linear'
            element(i).nvars = 0;
        case 'boucwen'
            element(i).nvars = 1;
            if ~isfield(element(i), 'alpha')
                element(i).alpha = 1.0 ; % linear stiffness component
            end 
            if ~isfield(element(i), 'beta')
                element(i).beta = element(i).alpha/10; % nonlinear stiffness component
            end
            if ~isfield(element(i), 'gamma')
                element(i).gamma = element(i).beta/2; % nonlinear stiffness component
            end
            if ~isfield(element(i), 'n')
                element(i).n = 1; % exponent for the hysteresis loop
            end

        case 'mostaghel'
            element(i).nvars = 1;
    end

    if ~isfield(element(i), 'a0')
        element(i).a0 = 0;
    end
    if ~isfield(element(i), 'a1')
        element(i).a1 = 0;
    end

    element(i).ndofs = height(element(i).m);

    % generalized mass matrix
    element(i).M = blkdiag(eye(size(element(i).m)),element(i).m, eye(element(i).nvars));
   
    % proportional damping
    element(i).c = element(i).a0 * element(i).m + element(i).a1 * element(i).k ;

end
end