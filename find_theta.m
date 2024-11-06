function [theta]=find_theta(t,e,n,Mean_Anomaly0)
% calculates the true anomaly at time t

Mean_Anomaly=Mean_Anomaly0+n*t;

E=Mean_Anomaly_inv(e,Mean_Anomaly);

temp=((1-e)/(1+e))*tan(E/2);

theta=2*atan(temp);

if theta>pi
    theta=theta-pi;
end

if theta<0
    theta=2*pi+theta;
end

end