function R=R_ne(lamda,phi)
R=[-sind(phi)*cosd(lamda),-sind(phi)*sind(lamda),cosd(phi);
    -sind(lamda),cosd(lamda),0;
    -cosd(phi)*cosd(lamda),-cosd(phi)*sind(lamda),-sind(phi)];
end