function [P,finish]=Path_search_Dijkstra(Map,Start_point,End_point)
P=[];
finish=0;
[a,b]=size(Map);
Open_list=[];
Closed_list=[];
% d_open=inf*ones(a*b,1);
Parent=[];
g_open=inf*ones(a*b,1);
index_e=(End_point(2)-1)*a+End_point(1);
index_i=(Start_point(2)-1)*a+Start_point(1);
if Map(index_i)==0 && Map(index_e)==0
    if norm(Start_point-End_point)~=0
        Open_list=[Open_list;index_i];
        g_open(index_i,1)=0;
    else
        P=index_i;
        return
    end
    while ~isempty(Open_list)
        [~,k]=min(g_open(Open_list,1));
        P_curr=Open_list(k(1),1);
        Closed_list=[Closed_list;P_curr];
        Open_list=Open_list(~ismember(Open_list,P_curr));
        P_n=find_neighbor(P_curr,8,a,b);
        l=size(P_n);
        if finish
            break
        end
        for m=1:l
            i=P_n(m,1);
            j=P_n(m,2);
            index1=(j-1)*a+i;
            if index1==index_e
                Parent(index1,1)=P_curr;
                finish=1;
                break
            end
            if Map(i,j)~=1
                g=g_open(P_curr)+1;
                if ~ismember(index1,Open_list) && ~ismember(index1,Closed_list)
                    Open_list=[Open_list;index1];
                    g_open(index1,1)=g;
                    Parent(index1,1)=P_curr;
                elseif g<g_open(index1,1)
                    Parent(index1,1)=P_curr;
                    g_open(index1,1)=g;
                end
            end
        end
        plotsearch;
    end
    
    if finish
        index1=index_e;
        while index1~=index_i
            P=[index1;P];
            index1=Parent(index1,1);
        end
        P=[index_i;P];
    else
        fprintf('Robot gets stuck.')
    end
else
    fprintf('Robot gets stuck, becasue start point or end point is occupied.')
end
    
end