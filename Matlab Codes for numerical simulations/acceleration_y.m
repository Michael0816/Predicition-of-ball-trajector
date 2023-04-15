function a_y = acceleration_y(C, p, r, v, m, theta)

a_y = - (m * 9.8 + 1 / 2 * C * p * pi * r * r * v * v * sin(theta)) / m ; 

end

