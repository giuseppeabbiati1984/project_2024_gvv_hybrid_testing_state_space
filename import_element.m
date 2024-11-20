function [string] = import_element(type)
%
% id  : identifies which function for R to include
%       'linear'        Linear element
%       'boucwen'       Bouc-Wen spring
%       'mostaghel'     Mostaghel spring


%% General function to compute state derivative xd
string = ['function xd = fcn(x,f,element)' newline newline];
string = [string 'xd = element.M \ (element.B*f - R_' type '(x,element));' newline newline 'end'];


%% Include function to compute R
% Initialisiation of function
% string = [string newline newline 'function R = evalR(x, element)' newline newline];
% 
% switch id
%     case 1
%         type = 'linear';
%     case 2
%         type = 'boucwen';
%     case 3
%         type = 'mostaghel';
% end
% string = [string 'R = R_' type '(x, element);'];
% 
% % % R - for linear deck element
% % if id == 1
% %     string = [string 'R = deckR(x, element);'];
% % % R - for pier element Bouc-Wen
% % elseif id == 2
% %     if nonlin == 0
% %         string = [string 'R = pierR(x, element);'];
% %     else
% %         string = [string 'R = pierR_boucwen(x, element);'];
% %     end
% % % R - for pier element Mostaghel
% % elseif id == 3
% %     if nonlin == 0
% %         string = [string 'R = pierR(x, element);'];
% %     else
% %         string = [string 'R = pierR_mostaghel(x, element);'];
% %     end
% % end
% 
% % Conclude function of R
% string = [string newline newline 'end'];

end