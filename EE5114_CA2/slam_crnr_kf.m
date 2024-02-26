function [known_corner, weight] = slam_crnr_kf(known_corner, detected_corner)
% Updates means, covariance for map features and update particle weight 
	
	% Calculate Kalman filter gain
	Qt = detected_corner.covariance;
	Q = known_corner.covariance + Qt;
	K = known_corner.covariance * Q^-1;
    
	% Calculate innovation 
    x_innov = detected_corner.x - known_corner.x;
    y_innov = detected_corner.y - known_corner.y;
    heading_innov = slam_in_pi(detected_corner.heading - known_corner.heading);
    angle_innov = slam_in_pi(detected_corner.angle - known_corner.angle);
    innovation = [x_innov; y_innov; heading_innov; angle_innov];
    
    % Missing codes start here ...
 
    % Update mean of this corner 
    P=[known_corner.x;known_corner.y;known_corner.heading;known_corner.angle];
    P=P+K*innovation;
    known_corner.x=P(1);
    known_corner.y=P(2);
    known_corner.heading=P(3);
    known_corner.angle=P(4);
	% Update covariance of this corner 
    known_corner.covariance=(eye(4,4)-K)*known_corner.covariance;
    % Missing codes end here ...

%     %update the covariance and the violation of particles after the KF
%     Q = known_corner.covariance + Qt;
%     x_innov = detected_corner.x - known_corner.x;
%     y_innov = detected_corner.y - known_corner.y;
%     heading_innov = slam_in_pi(detected_corner.heading - known_corner.heading);
%     angle_innov = slam_in_pi(detected_corner.angle - known_corner.angle);
%     innovation = [x_innov; y_innov; heading_innov; angle_innov];
    
    % Calculate importance weight introduced by this corner  
    weight = (det(2*pi*Q)^-0.5) * exp(-0.5*(innovation)'*(Q^-1)*(innovation));
%     innovation
	% Increment view count
	known_corner.view_count = known_corner.view_count + 1;
    
end

