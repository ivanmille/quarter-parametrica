function [] = plot_jounce_force(src,event)

displacement = evalin('base','displacement');
slider_dx.Value = evalin('base','slider_dx.Value');
steering = evalin('base','steering');
slider_steering.Value = evalin('base','slider_steering.Value');
out = evalin('base','out');

ii=find(displacement==slider_dx.Value,2);
jj=find(steering==slider_steering.Value,2);
kk=intersect(ii,jj);

plot(out(kk).FL_jounce_pos.Data,out(kk).FL_WHEEL_LOAD.Data)

legend(['dx = ' , num2str(displacement(kk)) ,newline, 'steering= ', num2str(steering(kk))]);

end

