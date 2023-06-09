function A = A_double_pend_springs(in1,in2)
%A_double_pend_springs
%    A = A_double_pend_springs(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.0.
%    16-Aug-2022 18:56:07

I1 = in2(7,:);
I2 = in2(8,:);
c1 = in2(3,:);
c2 = in2(4,:);
l1 = in2(1,:);
m1 = in2(5,:);
m2 = in2(6,:);
th2 = in1(2,:);
t2 = conj(I2);
t3 = conj(c2);
t4 = conj(l1);
t5 = conj(m2);
t6 = cos(th2);
t7 = t3.^2;
t9 = t3.*t4.*t5.*t6;
t8 = t5.*t7;
t10 = t2+t8+t9;
A = reshape([t9+t10+conj(I1)+conj(c1).^2.*conj(m1)+t4.^2.*t5,t10,t10,t2+t8],[2,2]);
