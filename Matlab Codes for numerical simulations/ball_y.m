function [y] = ball_y(t,v0y, y0)

y  =  y0 + v0y * t - 1/2 * 9.8 * t * t;

end

