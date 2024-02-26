
% Grid
[X_Length,Y_Length]=size(Map);
for x = 1:X_Length
    plot([x,x],[0,Y_Length],'black');
    hold on;
end
for y = 1:Y_Length
    plot([0,X_Length],[y,y],'black');
    hold on;
end
axis equal;
axis([0,X_Length,0,Y_Length]);

%Start_Node
Start_Node_X = Start_point(1);
Start_Node_Y = Start_point(2);
fill([Start_Node_X,Start_Node_X,Start_Node_X-1,Start_Node_X-1,Start_Node_X],[Start_Node_Y,Start_Node_Y-1,Start_Node_Y-1,Start_Node_Y,Start_Node_Y],'yellow');
%Target_Node
Target_Node_X = End_point(1);
Target_Node_Y = End_point(2);
fill([Target_Node_X,Target_Node_X,Target_Node_X-1,Target_Node_X-1,Target_Node_X],[Target_Node_Y,Target_Node_Y-1,Target_Node_Y-1,Target_Node_Y,Target_Node_Y],'green');
%Obs_Node
obstacles1=find(Map==1);
[n,~]=size(obstacles1);
for i=1:n
    obstacles(i,:)=[mod(obstacles1(i,1)-1,a)+1,ceil(obstacles1(i,1)/a)];
end
for i = 1:length(obstacles)
    Obs_Node_X = obstacles(i,1);
    Obs_Node_Y = obstacles(i,2);
    fill([Obs_Node_X,Obs_Node_X,Obs_Node_X-1,Obs_Node_X-1,Obs_Node_X],[Obs_Node_Y,Obs_Node_Y-1,Obs_Node_Y-1,Obs_Node_Y,Obs_Node_Y],'black');    
end

%Path_List
plot(P1(:,1)-0.5,P1(:,2)-0.5,'*'); 
plot(P1(:,1)-0.5,P1(:,2)-0.5,'-'); 