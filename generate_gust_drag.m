function drag = generate_gust_drag(gusts)
%
% creates drag from wind gusts
%
% input: gusts (x,y,z)
%
% output: drag (x,y,z)
%

areas = [.0197 .0197 .0197*2*1.3];
drag = [0 0 0];
rho = 1.225;
coef_d = 1.0;

for i=1:3
    drag(i) = -.5*rho*gusts(i)^2*areas(i)*coef_d;
    if(gusts(i)>0)
        drag(i) = drag(i) * -1;
    end   
end