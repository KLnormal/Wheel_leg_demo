function  K = get_k_lenth(leg_length)
% lenth = 0.1;
%     syms R L Lm l mw mp M Iw Ip Im
%     syms phi(t) theta(t) x(t) dotdot_phi dotdot_theta dotdot_x
%     syms T Tp N P Nm Pm Nf
%     g = 9.8;
%     R = 0.06; %轮子的半径
%     L = lenth/2; %摆杆重心到驱动轮轴距离
%     Lm = lenth/2; %摆杆重心到机体转轴距离
%     l = lenth; %腿长
%     mw = 0.8; %轮子质量
%     mp = 0.168; %摆杆质量
%     M = 4.56; %机体质量
%     Iw = mw/2*R*R; %轮子转动惯量
%     Ip = mp*l*l/12; %摆杆转动惯量
%     Im = 13121/1000/1000; %机体转动惯量
%     Nm = M*diff(x+(L+Lm)*sin(theta)-l*sin(phi),2,t);
%     Pm = M*diff((L+Lm)*cos(theta)-l*cos(phi),2,t);
%     N = mp*diff(x+L*sin(theta),2,t)+Nm;
%     P = mp*diff(L*cos(theta))+mp*g+Pm;
% 
%     eq1 = Ip*dotdot_theta == (P*L+Pm*Lm);
%     eq2 = Im*dotdot_phi == Tp + Nm*l*cos(phi)+Pm*l*sin(phi);
%     eq3 = dotdot_x == (T-N*R)/(Iw/R + mw*R);
% 
%     answer = solve([eq1,eq2,eq3],[dotdot_theta,dotdot_x,dotdot_phi]);
%     temp1 = answer.dotdot_theta;
%     temp2 = answer.dotdot_x;
%     temp3 = answer.dotdot_phi;
%     disp(vpa(temp1, 6));
%     disp(vpa(temp2, 6));
%     disp(vpa(temp3, 6));


    syms x(t) T R Iw mw M L LM theta(t) l phi(t) mp g Tp Ip IM
    syms f1 f2 f3 d_theta d_x d_phi theta0 x0 phi0 

    R1=0.06;                         %驱动轮半径
    L1=leg_length/2;                  %摆杆重心到驱动轮轴距离
    LM1=leg_length/2;                 %摆杆重心到其转轴距离
    l1=0.005;                          %机体质心距离转轴距离
    mw1=0.8;                         %驱动轮质量
    mp1=0.168;                         %杆质量
    M1=4.56;                          %机体质量
    Iw1=mw1*R1^2;                     %驱动轮转动惯量
    Ip1=mp1*((L1+LM1)^2+0.05^2)/12.0; %摆杆转动惯量
    IM1=M1*(0.3^2+0.12^2)/12.0;       %机体绕质心转动惯量

    
    NM = M*diff(x + (L + LM )*sin(theta)-l*sin(phi),t,2);
    N = NM + mp*diff(x + L*sin(theta),t,2);
    PM = M*g + M*diff((L+LM)*cos(theta)+l*cos(phi),t,2);
    P = PM +mp*g+mp*diff(L*cos(theta),t,2);

    eqn1 = diff(x,t,2) == (T -N*R)/(Iw/R + mw*R);
    eqn2 = Ip*diff(theta,t,2) == (P*L + PM*LM)*sin(theta)-(N*L+NM*LM)*cos(theta)-T+Tp;
    eqn3 = IM*diff(phi,t,2) == Tp +NM*l*cos(phi)+PM*l*sin(phi);
    
    eqn10 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn1,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn20 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn2,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);
    eqn30 = subs(subs(subs(subs(subs(subs(subs(subs(subs(eqn3,diff(theta,t,2),f1),diff(x,t,2),f2),diff(phi,t,2),f3),diff(theta,t),d_theta),diff(x,t),d_x),diff(phi,t),d_phi),theta,theta0),x,x0),phi,phi0);

    [f1,f2,f3] = solve(eqn10,eqn20,eqn30,f1,f2,f3);
   
    A=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[theta0,d_theta,x0,d_x,phi0,d_phi]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    A=subs(A,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    A=double(A);
    B=subs(jacobian([d_theta,f1,d_x,f2,d_phi,f3],[T,Tp]),[theta0,d_theta,d_x,phi0,d_phi,T,Tp],[0,0,0,0,0,0,0]);
    B=subs(B,[R,L,LM,l,mw,mp,M,Iw,Ip,IM,g],[R1,L1,LM1,l1,mw1,mp1,M1,Iw1,Ip1,IM1,9.8]);
    B=double(B);
    
    Q=diag([80 10 400 150 8000 10]);%theta d_theta x d_x phi d_phi%700 1 600 200 1000 1
    R=[240 0;0 25];                %T Tp
    
    K=lqr(A,B,Q,R);
fprintf('float K[%d] = {', numel(K));
for i = 1:numel(K)
    if i < numel(K)
        fprintf('%.6f, ', K(i));
    else
        fprintf('%.6f', K(i)); %最后一个元素不加逗号
    end
end
fprintf('};\n');
end