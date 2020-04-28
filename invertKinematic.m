function [q1_res, q2_res, q3_res] = invertKinematic(x, y, z)
    L = 0.524;
    l = 1.244;
    sB = 0.567;
    sP = 0.076;
%     x = 0.3;
%     y = 0.5;
%     z = -1.1;
%     x = 0.0;
%     y = -0.0;
%     z = -1;
    clf;
    plot3(0,0,0);
    
    %define
    x_ = 1;
    y_ = 2;
    z_ = 3;
    n1 = 1;
    n2 = 2;
    M_PI = 3.14159265358979323846;
    L_ = 0.1;
    %start solving
    P = [x, y, z];
    
    uB = sB * sqrt(3.0) / 3.0;
    wB = sB * sqrt(3.0) / 6.0;
    uP = sP * sqrt(3.0) / 3.0;
    wP = sP * sqrt(3.0) / 6.0;

    OB1(x_) = 0.0;
    OB1(y_) = -wB;
    OB1(z_) = 0.0;

    OB2(x_) = sqrt(3.0) * wB / 2.0;
    OB2(y_) = wB / 2.0;
    OB2(z_) = 0.0;

    OB3(x_) = -sqrt(3.0) * wB / 2.0;
    OB3(y_) = wB / 2.0;
    OB3(z_) = 0.0;

    OP1(x_) = x + 0.0;
    OP1(y_) = y + -uP;
    OP1(z_) = z + 0.0;

    OP2(x_) = x + sP / 2.0;
    OP2(y_) = y + wP;
    OP2(z_) = z + 0.0;

    OP3(x_) = x + -sP / 2.0;
    OP3(y_) = y + wP;
    OP3(z_) = z + 0.0;

    Ob1(x_) = sB / 2.0;
    Ob1(y_) = -wB;
    Ob1(z_) = 0;

    Ob2(x_) = 0.0;
    Ob2(y_) = uB;
    Ob2(z_) = 0.0;

    Ob3(x_) = -sB / 2.0;
    Ob3(y_) = -wB;
    Ob3(z_) = 0.0;
    
%     OA1(x_) = 0.0;
%     OA1(y_) = L - wB;
%     OA1(z_) = 0.0;
%     OA2(x_) = (sqrt(3.0) / 2.0) * (wB - L);
%     OA2(y_) = (1 / 2) * (wB - L);
%     OA2(z_) = 0.0;
%     OA3(x_) = (sqrt(3.0) / 2.0) * (L - wB);
%     OA3(y_) = (1.0 / 2.0) * (L + wB);
%     OA3(z_) = 0.0;
    
    a = -uP + wB;
    b = (sP / 2.0) - (wB * sqrt(3) / 2);
    c = wP - (wB / 2.0);
    
    E1 = (2.0 * L * (y + a));
    F1 = 2.0 * z * L;
    G1 = (x * x) + (y * y) + (z * z) + (a * a) + (L * L) + (2 * y * a) - (l * l);
    E2 = -L * ((x + b) * sqrt(3.0) + y + c);
    F2 = 2.0 * z * L;
    G2 = (x * x) + (y * y) + (z * z) + (b * b) + (c * c) + (L * L) + 2 * ((x * b) + (y * c)) - (l * l);
    E3 = L * ((x - b) * sqrt(3.0) - y - c);
    F3 = 2.0 * z * L;
    G3 = (x * x) + (y * y) + (z * z) + (b * b) + (c * c) + (L * L) + 2 * (-x * b + y * c) - l * l;
    
    t(1,n1) = ((-F1 - sqrt((E1 * E1) + (F1 * F1) - (G1 * G1))) / (G1 - E1));
    q(1,n1) = 2*atan(t(1,n1));
    q(1,n1) = q(1,n1) * 180.0 / M_PI;
    t(1,n2) = ((-F1 + sqrt((E1 * E1) + (F1 * F1) - (G1 * G1))) / (G1 - E1));
    q(1,n2) = 2*atan(t(1,n2));
    q(1,n2) = q(1,n2) * 180.0 / M_PI;
    if (abs(q(1,n1)) < abs(q(1,n2)))
        q1_res = q(1,n1);
    else
        q1_res = q(1,n2);
    end

    t(2,n1) = ((-F2 - sqrt((E2 * E2) + (F2 * F2) - (G2 * G2))) / (G2 - E2));
    q(2,n1) = 2*atan(t(2,n1));
    q(2,n1) = q(2,n1) * 180.0 / M_PI;
    t(2,n2) = ((-F2 + sqrt((E2 * E2) + (F2 * F2) - (G2 * G2))) / (G2 - E2));
    q(2,n2) = 2*atan(t(2,n2));
    q(2,n2) = q(2,n2) * 180.0 / M_PI;
    if (abs(q(2,n1)) < abs(q(2,n2)))
        q2_res = q(2,n1);
    else
        q2_res = q(2,n2);
    end

    t(3,n1) = ((-F3 - sqrt((E3 * E3) + (F3 * F3) - (G3 * G3))) / (G3 - E3));
    q(3,n1) = 2*atan(t(3,n1));
    q(3,n1) = q(3,n1) * 180.0 / M_PI;
    t(3,n2) = ((-F3 + sqrt((E3 * E3) + (F3 * F3) - (G3 * G3))) / (G3 - E3));
    q(3,n2) = 2*atan(t(3,n2));
    q(3,n2) = q(3,n2) * 180.0 / M_PI;
    if (abs(q(3,n1)) < abs(q(3,n2)))
        q3_res = q(3,n1);
    else
        q3_res = q(3,n2);
    end

%     figure;
    b1b2 = [Ob2;Ob1];
    b2b3 = [Ob3;Ob2];
    b3b1 = [Ob1;Ob3];
    plot3(b1b2(:,1),b1b2(:,2),b1b2(:,3),'r','LineWidth',2);
    hold on
    plot3(b2b3(:,1),b2b3(:,2),b2b3(:,3),'r','LineWidth',2);
    hold on
    plot3(b3b1(:,1),b3b1(:,2),b3b1(:,3),'r','LineWidth',2);
    hold on
    
    P1P2 = [OP1;OP2];
    P2P3 = [OP2;OP3];
    P3P1 = [OP3;OP1];
    plot3(P1P2(:,1),P1P2(:,2),P1P2(:,3),'r','LineWidth',2);
    hold on
    plot3(P2P3(:,1),P2P3(:,2),P2P3(:,3),'r','LineWidth',2);
    hold on
    plot3(P3P1(:,1),P3P1(:,2),P3P1(:,3),'r','LineWidth',2);
    hold on
    
    b1B3 = [Ob1;OB3];
    b2B1 = [Ob2;OB1];
    b3B2 = [Ob3;OB2];
    plot3(b1B3(:,1),b1B3(:,2),b1B3(:,3),'r');
    hold on
    plot3(b2B1(:,1),b2B1(:,2),b2B1(:,3),'r');
    hold on
    plot3(b3B2(:,1),b3B2(:,2),b3B2(:,3),'r');
    hold on
    
    PP1 = [P; OP1];
    PP2 = [P; OP2];
    PP3 = [P; OP3];
    plot3(PP1(:,1),PP1(:,2),PP1(:,3),'r');
    hold on
    plot3(PP2(:,1),PP2(:,2),PP2(:,3),'r');
    hold on
    plot3(PP3(:,1),PP3(:,2),PP3(:,3),'r');
    hold on
    
    OA1_ = OB1 + [0, -L*cosd(q1_res), -L*sind(q1_res)];
    OA2_ = OB2 + [L*sqrt(3.0)*cosd(q2_res)/2.0, L*cosd(q2_res)/2.0, -L*sind(q2_res)];
    OA3_ = OB3 + [-L*sqrt(3.0)*cosd(q3_res)/2.0, L*cosd(q3_res)/2.0, -L*sind(q3_res)];
    
%     OA3_1 = [OA3_(x_)+L_*cosd(60), OA3_(y_)+L*cosd(30), OA3_(z_)];
%     OA3_2 = [OA3_(x_)-L_*cosd(60), OA3_(y_)-L*cosd(30), OA3_(z_)];
%     OA3__ = [OA3_1;OA3_2];
%     plot3(OA3__(:,1), OA3__(:,2), OA3__(:,3),'g');
%     hold on
%     
%     OA2_1 = [OA2_(x_)+L_*cosd(60), OA2_(y_)-L*cosd(30), OA2_(z_)];
%     OA2_2 = [OA2_(x_)-L_*cosd(60), OA2_(y_)+L*cosd(30), OA2_(z_)];
%     OA2__ = [OA2_1;OA2_2];
%     plot3(OA2__(:,1), OA2__(:,2), OA2__(:,3),'g');
%     hold on
%     
%     OA1_1 = [OA1_(x_)-L_, OA1_(y_), OA1_(z_)];
%     OA1_2 = [OA1_(x_)+L_, OA1_(y_), OA1_(z_)];
%     OA1__ = [OA1_1;OA1_2];
%     plot3(OA1__(:,1), OA1__(:,2), OA1__(:,3),'g');
%     hold on
    
    B1A1 = [OB1;OA1_];
    B2A2 = [OB2;OA2_];
    B3A3 = [OB3;OA3_];

    plot3(B1A1(:,1),B1A1(:,2),B1A1(:,3),'b');
    text(OA1_(1), OA1_(2), OA1_(3), 'A1');
    hold on
    plot3(B2A2(:,1),B2A2(:,2),B2A2(:,3),'b');
    text(OA2_(1), OA2_(2), OA2_(3), 'A2');
    hold on
    plot3(B3A3(:,1),B3A3(:,2),B3A3(:,3),'b');
    text(OA3_(1), OA3_(2), OA3_(3), 'A3');
    hold on
    
    A1P1 = [OA1_;OP1];
    A2P2 = [OA2_;OP2];
    A3P3 = [OA3_;OP3];
    plot3(A1P1(:,1),A1P1(:,2),A1P1(:,3),'g');
    hold on
    plot3(A2P2(:,1),A2P2(:,2),A2P2(:,3),'g');
    hold on
    plot3(A3P3(:,1),A3P3(:,2),A3P3(:,3),'g');
    hold on
    length = [sqrt((OA1_(1)-OB1(1))^2+(OA1_(2)-OB1(2))^2+(OA1_(3)-OB1(3))^2),...
              sqrt((OA2_(1)-OB2(1))^2+(OA2_(2)-OB2(2))^2+(OA2_(3)-OB2(3))^2),...
              sqrt((OA3_(1)-OB3(1))^2+(OA3_(2)-OB3(2))^2+(OA3_(3)-OB3(3))^2),...
              sqrt((OA1_(1)-OP1(1))^2+(OA1_(2)-OP1(2))^2+(OA1_(3)-OP1(3))^2),...
              sqrt((OA2_(1)-OP2(1))^2+(OA2_(2)-OP2(2))^2+(OA2_(3)-OP2(3))^2),...
              sqrt((OA3_(1)-OP3(1))^2+(OA3_(2)-OP3(2))^2+(OA3_(3)-OP3(3))^2)]
          
    OZ = [[0,0,0];[0,0,z]];
%     plot3(OZ(:,1),OZ(:,2),OZ(:,3),'r');
    hold on
    plot3(0,0,-1.5);
    hold on
    plot3(0,0,1.5);
    grid on;
    plot3(0.7, 0.7, -1.5);
    grid on;
    plot3(0.7, -0.7, -1.5);
    grid on;
    plot3(-0.7, 0.7, -1.5);
    grid on;
    plot3(-0.7, -0.7, -1.5);
    grid on;
% end