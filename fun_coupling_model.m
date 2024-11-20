function [string] = fun_coupling_model(element)
%
% n  : identifies number of elements
%

n = numel(element);

%% General function
string = 'function res = fcn(';
for i = 1:1:n
    string = [string 'element' num2str(i) ','];
    string = [string 'x' num2str(i) 'd,'];
    string = [string 'f' num2str(i) ','];
end
string = [string 'model,xdd,xfd,fd,ff)' newline newline];


%% Equilibrium
string = [string '% Equilibrium' newline];
string = [string 'epsilon_B = '];
for i = 1:1:n
    string = [string 'element' num2str(i) '.Bbar * f' num2str(i)];
    string = [string ' + '];
end
string = [string 'model.Bbar_f * ff + model.Bbar_d * fd;' newline newline];



%% Compatibility
string = [string '% Compatibility' newline];
string = [string 'epsilon_G = [' newline];

% Before running into trouble with [] matrix for Gf
% for i = 1:1:n
%     string = [string '[element' num2str(i) '.G_f; element' num2str(i) '.G_d] * x' num2str(i) 'd'];
%     string = [string ' + blkdiag(element' num2str(i) '.Gbar_f, element' num2str(i) '.Gbar_d) * [xfd; xdd] ;' newline];
% end

for i = 1:1:n
    if nnz(element(i).G_f) > 0 && nnz(element(i).G_d) > 0
        string = [string '[element' num2str(i) '.G_f; element' num2str(i) '.G_d] * x' num2str(i) 'd'];
        string = [string ' + blkdiag(element' num2str(i) '.Gbar_f, element' num2str(i) '.Gbar_d) * [xfd; xdd] ;' newline];

    elseif nnz(element(i).G_f) > 0 && nnz(element(i).G_d) == 0
        string = [string 'element' num2str(i) '.G_f * x' num2str(i) 'd'];
        string = [string ' + element' num2str(i) '.Gbar_f * xfd ;' newline];

    elseif nnz(element(i).G_f) == 0 && nnz(element(i).G_d) > 0
        string = [string 'element' num2str(i) '.G_d * x' num2str(i) 'd'];
        string = [string ' + element' num2str(i) '.Gbar_d * xdd ;' newline];
    end
end
string = [string '];' newline newline];



%% Conclude function
string = [string '% Residuals' newline 'res = [epsilon_B; epsilon_G];'];
string = [string newline newline 'end'];

end