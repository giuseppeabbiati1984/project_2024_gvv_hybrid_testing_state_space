function [string] = import_element(type, tf_PS)
%
% id  : identifies which function for R to include
%       'linear'        Linear element
%       'boucwen'       Bouc-Wen spring
%       'mostaghel'     Mostaghel spring


%% General function to compute state derivative xd
if ~tf_PS
    string = ['function xd = fcn(x,f,element)' newline newline];    
    string = [string 'xd = element.M \ (element.B*f - R_' type '(x,element));' newline newline 'end'];

else
    string = ['function xd = fcn(x,f,element,R)' newline newline];   
    string = [string 'xd = element.M \ (element.B*f - R);' newline newline 'end'];
end
end