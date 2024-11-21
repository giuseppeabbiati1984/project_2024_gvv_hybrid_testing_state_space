function [string] = import_element(type)
%
% id  : identifies which function for R to include
%       'linear'        Linear element
%       'boucwen'       Bouc-Wen spring
%       'mostaghel'     Mostaghel spring


%% General function to compute state derivative xd
string = ['function xd = fcn(x,f,element)' newline newline];
string = [string 'xd = element.M \ (element.B*f - R_' type '(x,element));' newline newline 'end'];

end