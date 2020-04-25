import math
import snap7
import cv2
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fig = plt.figure()
ax = plt.axes(projection='3d')
point1 = [0,0,0];
point2 = [1,2,3];
vector_x = [point1[0],point2[0]]
vector_y = [point1[1],point2[1]]
vector_z = [point1[2],point2[2]]

def connect2Points(pointA, pointB):
    xValue = [pointA[0], pointB[0]]
    yValue = [pointA[1], pointB[1]]
    zValue = [pointA[2], pointB[2]]
    plt.plot(xValue, yValue, zValue)
    # plt.show()

def printPointValue(point):
    print(point[0])
    print(point[1])
    print(point[2])
    print(" ")

def invertKinematic(x, y, z):
    L = 0.524;
    l = 1.244;
    sB = 0.567;
    sP = 0.076;

    # x = 0.0;
    # y = -0.0;
    # z = -1;

    #%define
    x_ = 0;
    y_ = 1;
    z_ = 2;
    n1 = 0;
    n2 = 1;
    M_PI = 3.14159265358979323846;
    L_ = 0.1;
    #%start solving
    P = [x, y, z];
    
    uB = sB * math.sqrt(3.0) / 3.0;
    wB = sB * math.sqrt(3.0) / 6.0;
    uP = sP * math.sqrt(3.0) / 3.0;
    wP = sP * math.sqrt(3.0) / 6.0;

    OB1 = [0,0,0];
    OB1[x_] = 0.0;
    OB1[y_] = -wB;
    OB1[z_] = 0.0;
    print("OB1")
    printPointValue(OB1)
    OB2 = [0,0,0];
    OB2[x_] = math.sqrt(3.0) * wB / 2.0;
    OB2[y_] = wB / 2.0;
    OB2[z_] = 0.0;
    print("OB2")
    printPointValue(OB2)
    OB3 = [0,0,0];
    OB3[x_] = -math.sqrt(3.0) * wB / 2.0;
    OB3[y_] = wB / 2.0;
    OB3[z_] = 0.0;
    print("OB3")
    printPointValue(OB3)
    OP1 = [0,0,0];
    OP1[x_] = x + 0.0;
    OP1[y_] = y + -uP;
    OP1[z_] = z + 0.0;
    print("OP1")
    printPointValue(OP1)
    OP2 = [0,0,0];
    OP2[x_] = x + sP / 2.0;
    OP2[y_] = y + wP;
    OP2[z_] = z + 0.0;
    print("OP2")
    printPointValue(OP2)
    OP3 = [0,0,0];
    OP3[x_] = x + -sP / 2.0;
    OP3[y_] = y + wP;
    OP3[z_] = z + 0.0;
    print("OP3")
    printPointValue(OP3)
    Ob1 = [0,0,0];
    Ob1[x_] = sB / 2.0;
    Ob1[y_] = -wB;
    Ob1[z_] = 0;
    print("Ob1")
    printPointValue(Ob1)
    Ob2 = [0,0,0];
    Ob2[x_] = 0.0;
    Ob2[y_] = uB;
    Ob2[z_] = 0.0;
    print("Ob2")
    printPointValue(Ob2)
    Ob3 = [0,0,0];
    Ob3[x_] = -sB / 2.0;
    Ob3[y_] = -wB;
    Ob3[z_] = 0.0;
    print("Ob3")
    printPointValue(Ob3)
    a = -uP + wB;
    print("a")
    print(a)
    print("")
    b = (sP / 2.0) - (wB * math.sqrt(3) / 2);
    print("b")
    print(b)
    print("")
    c = wP - (wB / 2.0);
    print("c")
    print(c)
    print("")
    E1 = (2.0 * L * (y + a));
    print("E1")
    print(E1)
    print("")
    F1 = 2.0 * z * L;
    print("F1")
    print(F1)
    print("")
    G1 = (x * x) + (y * y) + (z * z) + (a * a) + (L * L) + (2 * y * a) - (l * l);
    print("G1")
    print(G1)
    print("")
    E2 = -L * ((x + b) * math.sqrt(3.0) + y + c);
    print("E2")
    print(E2)
    print("")
    F2 = 2.0 * z * L;
    print("F2")
    print(F2)
    print("")
    G2 = (x * x) + (y * y) + (z * z) + (b * b) + (c * c) + (L * L) + 2 * ((x * b) + (y * c)) - (l * l);
    print("G2")
    print(G2)
    print("")
    E3 = L * ((x - b) * math.sqrt(3.0) - y - c);
    print("E3")
    print(E3)
    print("")
    F3 = 2.0 * z * L;
    print("F3")
    print(F3)
    print("")
    G3 = (x * x) + (y * y) + (z * z) + (b * b) + (c * c) + (L * L) + 2 * (-x * b + y * c) - l * l;
    print("G3")
    print(G3)
    print("")

    t = [[0,0],[0,0],[0,0]]
    q = [[0,0],[0,0],[0,0]]

    t[0][n1] = ((-F1 - math.sqrt((E1 * E1) + (F1 * F1) - (G1 * G1))) / (G1 - E1));
    q[0][n1] = 2*math.atan(t[0][n1]);
    # q[0][n1] = q[0][n1] * 180.0 / M_PI;
    t[0][n2] = ((-F1 + math.sqrt((E1 * E1) + (F1 * F1) - (G1 * G1))) / (G1 - E1));
    q[0][n2] = 2*math.atan(t[0][n2]);
    # q[0][n2] = q[0][n2] * 180.0 / M_PI;
    if (abs(q[0][n1]) < abs(q[0][n2])):
        q1_res = q[0][n1];
    else:
        q1_res = q[0][n2];

    t[1][n1] = ((-F2 - math.sqrt((E2 * E2) + (F2 * F2) - (G2 * G2))) / (G2 - E2));
    q[1][n1] = 2*math.atan(t[1][n1]);
    # q[1][n1] = q[1][n1] * 180.0 / M_PI;
    t[1][n2] = ((-F2 + math.sqrt((E2 * E2) + (F2 * F2) - (G2 * G2))) / (G2 - E2));
    q[1][n2] = 2*math.atan(t[1][n2]);
    # q[1][n2] = q[1][n2] * 180.0 / M_PI;
    if (abs(q[1][n1]) < abs(q[1][n2])):
        q2_res = q[1][n1];
    else:
        q2_res = q[1][n2];

    t[2][n1] = ((-F3 - math.sqrt((E3 * E3) + (F3 * F3) - (G3 * G3))) / (G3 - E3));
    q[2][n1] = 2*math.atan(t[2][n1]);
    # q[2][n1] = q[2][n1] * 180.0 / M_PI;
    t[2][n2] = ((-F3 + math.sqrt((E3 * E3) + (F3 * F3) - (G3 * G3))) / (G3 - E3));
    q[2][n2] = 2*math.atan(t[2][n2]);
    # q[2][n2] = q[2][n2] * 180.0 / M_PI;
    if (abs(q[2][n1]) < abs(q[2][n2])):
        q3_res = q[2][n1];
    else:
        q3_res = q[3][n2];

    OA1 = [0,0,0];
    OA1[x_] = 0.0;
    OA1[y_] = -wB - L *math.cos(q1_res);
    OA1[z_] = 0.0 - L *math.sin(q1_res);
    OA2 = [0,0,0];
    OA2[x_] = wB *math.sqrt(3.0) /2.0 + L *math.cos(q2_res);
    OA2[y_] = wB /2.0 + L*math.cos(q2_res) /2.0;
    OA2[z_] = 0.0 - L *math.sin(q2_res);
    OA3 = [0,0,0];
    OA3[x_] = -wB *math.sqrt(3.0) /2.0 - L *math.sqrt(3.0) *math.cos(q3_res);
    OA3[y_] = wB /2.0 + L *math.cos(q3_res) /2.0;
    OA3[z_] = 0.0 - L *math.sin(q3_res);

    print(q1_res);
    print(q2_res);
    print(q3_res);

    connect2Points(Ob1, Ob2)
    connect2Points(Ob2, Ob3)
    connect2Points(Ob3, Ob1)
    connect2Points(OB1, OA1)
    connect2Points(OB2, OA2)
    connect2Points(OB3, OA3)
    connect2Points(OA1, OP1)
    connect2Points(OA2, OP2)
    connect2Points(OA3, OP3)
    connect2Points(OP1, OP2)
    connect2Points(OP2, OP3)
    connect2Points(OP3, OP1)
    plt.show()

invertKinematic(0,0,-1)
# connect2Points(point1, point2)

input("Press Enter to continue...")

