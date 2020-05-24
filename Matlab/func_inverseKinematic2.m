function [q1_res,q2_res,q3_res] = func_inverseKinematic2(X,Y,Z)

%     L = 0.2;
%     l = 0.28;
%     sB  = 0.35;
%     sP = 0.107;
%     sP2  = sP*2;
    L = 0.292;
    l = 0.372;
    sB = 0.133/(sqrt(3.0) / 6.0);
    sP = 0.095/(sqrt(3.0) / 3.0);
    sP2 = sP*2;

    u1=X;
    u2=Y;
    u3=Z;
    x=X;
    y=Y;
    z=Z;
    P = [x,y,z];

    clf;
    plot3(0,0,0);

    %define
    x_ = 1;
    y_ = 2;
    z_ = 3;

    sqrt3 = sqrt(3.0);
    pi     = 3.14159265358979323846;
    sin120 = sqrt3 / 2.0;
    cos120 = -0.5;
    % tan60  = sqrt3;
    % sin30  = 0.5;
    % tan30  = 1.0 / sqrt3;
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
    
%..................................................... Inverse Kinematics
    x0 =u1;
    y0 =u2;
    z0 =u3;

    y1 = -0.5*0.57735*sB ;
    y0 = y0-0.5*0.57735*sP2; % shift center to edge
    
    a = (x0.*x0 + y0.*y0 + z0.*z0 + L.*L - l.*l - y1.*y1) ./ (2.0.*z0);
    b = (y1-y0) ./ z0;

    % discriminant
    d = -(a + b.*y1).*(a + b.*y1) + L.*(b.*b.*L + L);
   
   
    yj = (y1 - a.*b - sqrt(d)) ./ (b.*b + 1); 
    zj = a + b.*yj;
    if yj>y1 
        p=180;
    else
        p=0;
    end
    theta = atan(-zj ./ (y1-yj)) .* 180.0 / pi + p;
     
    q1_res=theta;
    
    if d<0 
        q1_res=0; % non-existing povar. 
    end  
    %.......................................................
    x0 =u1*cos120 + u2*sin120;
    y0 =u2*cos120-u1*sin120;
    z0 =u3;

    y1 = -0.5*0.57735*sB ;
    y0 = y0-0.5*0.57735*sP2; % shift center to edge
    % z = a + b*y
    a = (x0.*x0 + y0.*y0 + z0.*z0 + L.*L - l.*l - y1.*y1) ./ (2.0.*z0);
    b = (y1-y0) ./ z0;

    % discriminant
    d = -(a + b.*y1).*(a + b.*y1) + L.*(b.*b.*L + L);
  
    yj = (y1 - a.*b - sqrt(d)) ./ (b.*b + 1); % choosing outer povar
    zj = a + b.*yj;
    if yj>y1 
        p=180;
    else
        p=0;
    end
    theta = atan(-zj ./ (y1-yj)) .* 180.0 / pi + p;
    q2_res=theta;
   
    if d<0 
        q2_res=0; % non-existing povar.
    end 
    %.......................................................
    x0 =u1*cos120 - u2*sin120;
    y0 =u2*cos120+u1*sin120;
    z0 =u3;

    y1 = -0.5*0.57735*sB ;
    y0 = y0-0.5*0.57735*sP2; % shift center to edge
    % z = a + b*y
    a = (x0.*x0 + y0.*y0 + z0.*z0 + L.*L - l.*l - y1.*y1) ./ (2.0.*z0);
    b = (y1-y0) ./ z0;

    % discriminant
    d = -(a + b.*y1).*(a + b.*y1) + L.*(b.*b.*L + L);
  
    yj = (y1 - a.*b - sqrt(d)) ./ (b.*b + 1); % choosing outer povar
    zj = a + b.*yj;
    if yj>y1 
        p=180;
    else
        p=0;
    end
    theta = atan(-zj ./ (y1-yj)) .* 180.0 / pi + p;
    
    q3_res=theta; % return error, theta
   
    if d<0 
        q3_res=0; % non-existing povar.  
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
%     length = [sqrt((OA1_(1)-OB1(1))^2+(OA1_(2)-OB1(2))^2+(OA1_(3)-OB1(3))^2),...
%               sqrt((OA2_(1)-OB2(1))^2+(OA2_(2)-OB2(2))^2+(OA2_(3)-OB2(3))^2),...
%               sqrt((OA3_(1)-OB3(1))^2+(OA3_(2)-OB3(2))^2+(OA3_(3)-OB3(3))^2),...
%               sqrt((OA1_(1)-OP1(1))^2+(OA1_(2)-OP1(2))^2+(OA1_(3)-OP1(3))^2),...
%               sqrt((OA2_(1)-OP2(1))^2+(OA2_(2)-OP2(2))^2+(OA2_(3)-OP2(3))^2),...
%               sqrt((OA3_(1)-OP3(1))^2+(OA3_(2)-OP3(2))^2+(OA3_(3)-OP3(3))^2)]
    theta_ = [q1_res, q2_res, q3_res]
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
    
end

