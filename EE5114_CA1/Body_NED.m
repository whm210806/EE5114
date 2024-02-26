function A=Body_NED(phi,tht,psi)
A=[cos(psi)*cos(tht),cos(psi)*sin(tht)*sin(phi)-sin(psi)*cos(phi),cos(psi)*sin(tht)*cos(phi)+sin(psi)*sin(phi);
   sin(psi)*cos(tht),sin(psi)*sin(tht)*sin(phi)+cos(psi)*cos(phi),sin(psi)*sin(tht)*cos(phi)-cos(psi)*sin(phi);
   -sin(tht),cos(tht)*sin(phi),cos(tht)*cos(phi)];
end