function P_n=find_neighbor(P,l,a,b)
% l=4, 4 neighborhood
% l=8, 8 neighborhood
% k=1;
P=[mod(P-1,a)+1,ceil(P/a)];
P_n=[];
if l==4
    for i=[-1,1]
        if min(P+[i,0])>0 && P(1)+i < a+1  
            P_n=[P_n;P+[i,0]];
%                 k=k+1;
        end
    end
    for j=[-1,1]
        if min(P+[0,j])>0 && P(2)+j < b+1
            P_n=[P_n;P+[0,j]];
%                 k=k+1;
        end
    end
end
if l==8
    for i=-1:1
        for j=-1:1
            if i~=0 || j~=0
                if min(P+[i,j])>0 && P(1)+i < a+1 && P(2)+j < b+1
                    P_n=[P_n;P+[i,j]];
    %                 k=k+1;
                end
            end
        end
    end
end

end