close all;clear all;
load('Map.mat');
Start_point=[1,1];
End_point=[10,10];
% A-star path planning
[P,finish]=Path_search_A_star(Map,Start_point,End_point);
% Dijkstra path planning
% [P,finish]=Path_search_Dijkstra(Map,Start_point,End_point);
if finish
    [a,b]=size(Map);
    [m,~]=size(P);
    for i=1:m
        P1(i,:)=[mod(P(i,:)-1,a)+1,ceil(P(i,:)/a)];
        PP(2*(i-1)+1,:)=P1(i,:);
    end
    for i=1:m-1
        PP(2*(i-1)+1:2*i+1,:)=[linspace(P1(i,1),P1(i+1,1),3);linspace(P1(i,2),P1(i+1,2),3)]';
    end
    figure;
    plot_path;
    obstacles1=find(Map==1);
    for i=1:length(obstacles1)
        obstacles(i,:)=[mod(obstacles1(i,:)-1,a)+1,ceil(obstacles1(i,:)/a)];
    end
    % including the boundary of the map
    for i=1:10
        obstacles=[obstacles;0,i;11,i;i,0;i,11];
    end
    % including the corner of the map
    obstacles=[obstacles;0,0;0,11;11,0;11,11];
    total_force=1;
    P1=PP;
    P2=P1;
    delta=2; % threshold of the force of the obstacles
    epsilon=0.1;
    figure;
    for I=1:20
        plot_path;
        for i=2:length(P1)-1
            F=zeros(length(P1),2);
            F1=zeros(length(P1),2);
            repulsiveForce=zeros(1,2);
            F(i,:)=F(i,:)+P1(i-1,:)-2*P1(i,:)+P1(i+1,:);
            distances = sqrt(sum((obstacles - P1(i,:)).^2, 2));
            % Calculate the repulsive force of the closest obstacle 
            minDist = min(distances);
            minDistIdx=find(distances==minDist);
            if minDist < delta
                for p=1:length(minDistIdx)
                    repulsiveForce =repulsiveForce+(1 / minDist - 1 / delta)*(1 / minDist^2) * (P1(i,:) - obstacles(minDistIdx(p),:));
                end
            end
            F(i,:)=F(i,:)+repulsiveForce;
            F1(i,:)=F(i,:)/norm(F(i,:));
            P2=P1+0.1*F1;
            if isequal(P1,P2)
                break
            end
            P1=P2;
        end
    end
    figure;
    plot_path;
end