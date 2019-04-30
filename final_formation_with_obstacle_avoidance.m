
%%%%%%%%%%%%%%%initialization

target=[100 100];
lem=2;
obs=[50 50];



% agents system dynamics which accelaration is the only determination.
A = [0 1; 0 0];
B = [0 0; -1 -1];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




% Laplacian matrix
L=[3 -1 -1 -1;-1 3 -1 -1;-1 -1 3 -1;0 0 0 0];
% create system dynamic%number of agents is 4 which is inside eye()
Ac = kron(eye(4),A);
Bc = kron(L,(B));



%digitization
%%%%%%%%%%%%%%
Ad = eye(2*4)+Ac*0.1;
Bd = 0.1*Bc;
%%%%%%%%%%%%%%



% Initial Conditions
% the matrix that stores all the states of all 4 agents.
% positons of 4 agents are stored in x 1,3,5,7 respectively.
% velocity of 4 agents are stored in x 2,4,6,8 respectively.
x(1:8,:) = [0 0;5 0;60 5;0 3;30 50;2 2;20 20;1 1];
% Formation shape that agents are going to follow
D = [20 0;0 0;0 0;0 0;0 20;0 0;20 20;0 0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% main loop
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%update all the states for leader and followers
%plot the results every (pause)secs
for i=1:800        %iteration that go through
    

    
%if there are any agent whose distance between obstacle and itself is less
%then 10, call the obstacle_avoidance function, else continuoue formation.
 if (((obs(1,1)-x(1,1))^2+(obs(1,2)-x(1,2))^2>200) && ((obs(1,1)-x(3,1))^2+(obs(1,2)-x(3,2))^2>200) && ((obs(1,1)-x(5,1))^2+(obs(1,2)-x(5,2))^2>200) && ((obs(1,1)-x(7,1))^2+(obs(1,2)-x(7,2))^2>200))
    
    
    
%for the leader to go to target
    dist=((target(1,1)-x(7,1))^2+(target(1,2)-x(7,2))^2)^0.5;
    cos=(target(1,1)-x(7,1))/dist;
    sin=(target(1,2)-x(7,2))/dist;
    x(8,:)=[lem*cos lem*sin];
%formation control   
 y(1:8,:) = Ad*x(1:8,:)+Bd*(x(1:8,:)-D);
 
 x1=y(1,1);
 y1=y(1,2);
 x2=y(3,1);
 y2=y(3,2);
 x3=y(5,1);
 y3=y(5,2);
 x4=y(7,1);
 y4=y(7,2);
 %update the states
 x(1:7,:)=y(1:7,:);

 else
     %%%%%%%%%call function obstacle_avoidance%%%%%%%%%%%%
 [x1,y1] = obstacle_avoidance(x(1,1), x(1,2));
 [x2,y2] = obstacle_avoidance(x(3,1), x(3,2));
 [x3,y3] = obstacle_avoidance(x(5,1), x(5,2));
 [x4,y4] = obstacle_avoidance(x(7,1), x(7,2));
 

 

 %update the states (positons only)
 x(1,1)=x1;x(1,2)=y1;
 x(3,1)=x2;x(3,2)=y2;
 x(5,1)=x3;x(5,2)=y3;
 x(7,1)=x4;x(7,2)=y4;
 
 
 
 
 
 end
 
 x5=target(1,1);y5=target(1,2);
 x6=obs(1,1);y6=obs(1,2);
 %plot postions for all agents as well as target and obstacle
 xx=[x1;x2;x3;x4;x5;x6];
 yy=[y1;y2;y3;y4;y5;y6];
 
 plot(xx,yy,'x');

 xlim([0 200]);
 ylim([0 200]);

 pause(0.04);
     
 
 
 
 
 
 
 
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
