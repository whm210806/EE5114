figure(1);
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
%Closed_List
for i = 1:length(Closed_list)
    closed_list=[];
    closed_list(i,:)=[mod(Closed_list(i,1)-1,a)+1,ceil(Closed_list(i,1)/a)];
    Close_Node = closed_list(i,:);
    if Close_Node(1) == Start_point(1) && Close_Node(2) == Start_point(2)%如果是Start_Node就不当作CloseNode画
        continue;
    elseif Close_Node(1) == End_point(1) && Close_Node(2) == End_point(2)%如果是Target_Node就不当作CloseNode画
        continue;
    end
    Close_Node_X = Close_Node(1);
    Close_Node_Y = Close_Node(2);
    fill([Close_Node_X,Close_Node_X,Close_Node_X-1,Close_Node_X-1,Close_Node_X],[Close_Node_Y,Close_Node_Y-1,Close_Node_Y-1,Close_Node_Y,Close_Node_Y],'blue'); 
end
%Open_List
for i = 1:length(Open_list)
    open_list=[];
    open_list(i,:)=[mod(Open_list(i,1)-1,a)+1,ceil(Open_list(i,1)/a)];
    Open_Node = open_list(i,:);
    if Open_Node(1) == Start_point(1) && Open_Node(2) == Start_point(2)%如果是Start_Node就不当作CloseNode画
        continue;
    elseif Open_Node(1) == End_point(1) && Open_Node(2) == End_point(2)%如果是Target_Node就不当作CloseNode画
        continue;
    end
    Open_Node_X = Open_Node(1);
    Open_Node_Y = Open_Node(2);
    fill([Open_Node_X,Open_Node_X,Open_Node_X-1,Open_Node_X-1,Open_Node_X],[Open_Node_Y,Open_Node_Y-1,Open_Node_Y-1,Open_Node_Y,Open_Node_Y],'red'); 
end

% end