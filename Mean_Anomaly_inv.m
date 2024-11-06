function [E] = Mean_Anomaly_inv(e,Mean_Anomaly)

% E_1=0;
% E_2=Mean_Anomaly;
% 
% while (E_2-E_1) > 10^(-16) 
%     temp=E_2+e*sin(E_2);
%     E_1=E_2;
%     E_2=temp;
% end
% 
% E=E_2;
tol=10^(-4);
Etemp = Mean_Anomaly;
ratio = 1;
% while abs(ratio) > tol
for i=1:10
    f_E = Etemp - e*sin(Etemp) - Mean_Anomaly;
    f_Eprime = 1 - e*cos(Etemp);
    ratio = f_E/f_Eprime;
    % if abs(ratio) > tol
        Etemp = Etemp - ratio;
    % else
        % E = Etemp;
    % end
end
E = Etemp;
end