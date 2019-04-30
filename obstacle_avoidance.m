%%%%%%%%function 
%%%%%%%%obstacle avoidance
%%%%%%%%%%%%%%%%
function [x,y]=obstacle_avoidance(x,y)

%initialization
lem=0.2;          %velocity
ga=40;            % constant parameter

sig=50;          %constant parameter
Wa=20;            %constant parameter

Po=[50 50];       %postion of obstacle                                                         
Pg=[100 100];     %postion of target
W2=2;             %constant parameter
m=zeros(1,9);     %array that stores potential field in 9 directions 
%%%%%loop that find best value of potential field for 9 postions around
%%%%%current postion
 for q=0:8
    %to generate potential position
    Pt=[x,y]+[lem*cos((ga/180)*pi*q) lem*sin((ga/180)*pi*q)];
    
    
    %calculate attractive force
    Ua = Wa*((Pg(1,1)-Pt(1,1))^2+(Pg(1,2)-Pt(1,2))^2);
    %calculate repulsive force
    Uob = W2*exp((-1/sig^2)*((Pt(1,1)-Po(1,1))^2+(Pt(1,2)-Po(1,2))^2));
    %summation
    Ut=Ua+Uob;
    %store the 9 potential field to array
    m(1,q+1)=Ut;
    
    
 end
   %find the minimum value with potential field
[~,I] = min(m);
   %generate new coordinates(position)
x=x+lem*cos((ga/180)*pi*I);
y=y+lem*sin((ga/180)*pi*I);



end