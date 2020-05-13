% error cov measurement update
sigma_x = sigma_x - K*sigma_y*K';
[~,S,V] = svd(sigma_x);
% hager method to ensure positive semidefinite
HH = V*S*V';
sigma_x = (sigma_x + sigma_x' + HH + HH')/4;