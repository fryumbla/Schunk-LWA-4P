#include <eigen3/Eigen/Dense>
using Eigen::MatrixXd;
using Eigen::Vector3d;

#include <iostream>
using namespace std;
using namespace Eigen;
#define EPS 2.22e-16

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <unistd.h>

// DH parameters Matrix
MatrixXd TF_Matrix(double alpha,double a,double d,double q)
{
    MatrixXd TF(4,4);
    TF <<   cos(q),-sin(q)*cos(alpha),  sin(q)*sin(alpha),      a*cos(q),
            sin(q), cos(q)*cos(alpha), -cos(q)*sin(alpha),      a*sin(q),
                 0,        sin(alpha),         cos(alpha),             d,
                 0,                 0,                  0,             1;
    
    // TF <<           cos(q),                      -sin(q),           0,             a,
    //                 sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d,
    //                 sin(q)*sin(alpha), cos(q)*sin(alpha), cos(alpha) ,  cos(alpha)*d,
    //                 0,              0,                 0,                          1;

    return TF;
}

double subproblema0(Vector3d p,Vector3d q,Vector3d k)
{
    Vector3d pp=p/p.norm();
    Vector3d qp=q/q.norm();
    
    Vector3d n1=pp-qp;
    Vector3d n2=pp+qp;

    double theta=2*atan2(n1.norm(),n2.norm());
  
    if (k.transpose()*(p.cross(q))<0)
    {
        theta=-theta;
    }
    return theta;
}

double subproblema1(Vector3d k,Vector3d p,Vector3d q)
{
    Vector3d kf= k/k.norm();
    Vector3d pp=p-((p.transpose()*kf)*kf);
    Vector3d qp=q-((q.transpose()*kf)*kf);

    Vector3d epp=pp/pp.norm();
    Vector3d eqp=qp/qp.norm();

    double theta=subproblema0(epp,eqp,kf);

    return theta;
}

Vector4d subproblema2(Vector3d k1,Vector3d k2,Vector3d p,Vector3d q)
{
    Vector4d theta;

    double k12=k1.transpose()*k2;
    double pk=p.transpose()*k2;
    double qk=q.transpose()*k1;

    // if ((k12*k12)-1<EPS)
    // {
    //     std::cout<< "no solution one" << std::endl;
    //     theta << NAN,NAN,NAN,NAN;
    //     return theta;
    // }

    Matrix2d temp; temp<< k12,-1,-1,k12;
    Vector2d temp1; temp1<< pk,qk;
    Vector2d a=(temp*temp1)/((k12*k12)-1);

    double bb=(p.norm()*p.norm())-(a.norm()*a.norm())-(2*a(0)*a(1)*k12);
    
    if (abs(bb)<EPS)
    {
        bb=0;
    }
    // if (bb<0)
    // {
    //     std::cout<< "no solution 2" << std::endl;
    //     theta << NAN,NAN,NAN,NAN;
    //     return theta;  // ojo no colocar
    // }

    Vector3d temp3= k1.cross(k2);
    double gamma = sqrt(bb)/temp3.norm();

    Matrix3d temp4; temp4 << k1(0),k2(0),temp3(0),
                             k1(1),k2(1),temp3(1),                   
                             k1(2),k2(2),temp3(2);
    Vector3d temp5; temp5 << a(0),a(1),gamma;

    if(abs(gamma)<EPS)
    {
        VectorXd c1=temp4*temp5;
        theta(0)= -subproblema1(k1,q,c1);
        theta(1)= NAN;
        theta(2)= subproblema1(k2,p,c1);
        theta(3)= NAN;
        return theta; 
    } 

    VectorXd c1=temp4*temp5;

    temp5; temp5 << a(0),a(1),-gamma;  
    VectorXd c2; c2=temp4*temp5;
    
    theta(0)= -subproblema1(k1,q,c1);
    theta(2)= subproblema1(k2,p,c1);

    theta(1)= -subproblema1(k1,q,c2);
    theta(3)= subproblema1(k2,p,c2);

    return theta;
}

VectorXd input()
{
    double x,y,z,alpha,beta,gamma;
    VectorXd goal(6);
    std::cout << "The position in mm" << std::endl;
    std::cout <<"X position: ";
    cin >> x;
        std::cout <<"Y position: ";
    cin >> y;
    std::cout <<"Z position: ";
    cin >> z;
    std::cout << "The angles in degrees" << std::endl;
    std::cout <<"alpha angle: ";
    cin >> alpha;
    if(alpha==0)
        alpha=0.00001;
    std::cout <<"beta angle: ";
    cin >> beta;
    if(beta==0)
        beta=0.00001;
    std::cout <<"gamma angle: ";
    cin >> gamma;
    if(gamma==0)
        gamma=0.00001;
    goal << x,y,z,gamma*M_PI/180,beta*M_PI/180,alpha*M_PI/180;

    return goal;
}

MatrixXd Euler(VectorXd goal)
{
    MatrixXd TF(4,4);
    TF <<   cos(goal(3))*cos(goal(4)), cos(goal(3))*sin(goal(4))*sin(goal(5))-sin(goal(3))*cos(goal(5)), cos(goal(3))*sin(goal(4))*cos(goal(5))+sin(goal(3))*sin(goal(5)),      goal(0),
            sin(goal(3))*cos(goal(4)), sin(goal(3))*sin(goal(4))*sin(goal(5))+cos(goal(3))*cos(goal(5)), sin(goal(3))*sin(goal(4))*cos(goal(5))-cos(goal(3))*sin(goal(5)),      goal(1),
                        -sin(goal(4)),                                        cos(goal(4))*sin(goal(5)),                                        cos(goal(4))*cos(goal(5)),      goal(2),
                                    0,                                                                0,                                                                0,            1;
    return TF;
}



MatrixXd TF_Derivate(double alpha,double a,double d,double q)
{
    MatrixXd TF(4,4);
    TF <<   -sin(q), sin(q)*sin(alpha)-cos(q)*cos(alpha),  cos(q)*sin(alpha)+sin(q)*cos(alpha),     -a*sin(q),
             cos(q),-sin(q)*cos(alpha)-cos(q)*sin(alpha),  sin(q)*sin(alpha)-cos(q)*cos(alpha),      a*cos(q),
                  0,                          cos(alpha),         -sin(alpha),                              0,
                  0,                                   0,                  0,                               1;
    
    // TF <<   cos(q),-sin(q)*cos(alpha),  sin(q)*sin(alpha),      a*cos(q),
    //         sin(q), cos(q)*cos(alpha), -cos(q)*sin(alpha),      a*sin(q),
    //              0,        sin(alpha),         cos(alpha),             d,
    //              0,                 0,                  0,             1;

    return TF;
}


MatrixXd Jacobian(VectorXd goal)
{
    double q1, q2, q3, q4, q5, q6;
    double a0, a1, a2, a3, a4, a5;
    double d1, d2, d3, d4, d5, d6;
    double alpha0, alpha1, alpha2, alpha3, alpha4, alpha5;

    MatrixXd T_0(4,4);
    T_0 <<   1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    //  Example in cm
    double myNum[24]={alpha0=-M_PI/2.,    a0=   0,    d1=  205,   q1=goal(0),
                      alpha1=    M_PI,    a1= 350,    d2=     0,  q2=goal(1),
                      alpha2=-M_PI/2.,    a2=   0,    d3=     0,  q3=goal(2),
                      alpha3= M_PI/2.,    a3=   0,    d4=   305,  q4=goal(3),
                      alpha4=-M_PI/2.,    a4=   0,    d5=     0,  q5=goal(4),
                      alpha5=       0,    a5=   0,    d6=    75,  q6=goal(5)};

    MatrixXd T_1(4,4); T_1= TF_Matrix(alpha0, a0, d1, q1);
    MatrixXd T_2(4,4); T_2= TF_Matrix(alpha1, a1, d2, q2);
    MatrixXd T_3(4,4); T_3= TF_Matrix(alpha2, a2, d3, q3);
    MatrixXd T_4(4,4); T_4= TF_Matrix(alpha3, a3, d4, q4);
    MatrixXd T_5(4,4); T_5= TF_Matrix(alpha4, a4, d5, q5);
    MatrixXd T_6(4,4); T_6= TF_Matrix(alpha5, a5, d6, q6);

    MatrixXd T0_6= T_1 * T_2 * T_3 * T_4 * T_5 * T_6;

    MatrixXd t00 = T_0*T_0;
    MatrixXd t01 = T_0*T_1;
    MatrixXd t02 = T_0*T_2;
    MatrixXd t03 = T_0*T_3;
    MatrixXd t04 = T_0*T_4;
    MatrixXd t05 = T_0*T_5;
    MatrixXd t06 = T_0*T_6;

    MatrixXd Rd_1(4,4); Rd_1= TF_Derivate(alpha0, a0, d1, q1);
    MatrixXd Rd_2(4,4); Rd_2= TF_Derivate(alpha1, a1, d2, q2);
    MatrixXd Rd_3(4,4); Rd_3= TF_Derivate(alpha2, a2, d3, q3);
    MatrixXd Rd_4(4,4); Rd_4= TF_Derivate(alpha3, a3, d4, q4);
    MatrixXd Rd_5(4,4); Rd_5= TF_Derivate(alpha4, a4, d5, q5);
    MatrixXd Rd_6(4,4); Rd_6= TF_Derivate(alpha5, a5, d6, q6);


    MatrixXd td1 = T_6*T_5*T_4*T_3*T_2*Rd_1;
    MatrixXd td2 = T_6*T_5*T_4*T_3*Rd_2*T_1;
    MatrixXd td3 = T_6*T_5*T_4*Rd_3*T_2*T_1;
    MatrixXd td4 = T_6*T_5*Rd_4*T_3*T_2*T_1;
    MatrixXd td5 = T_6*Rd_5*T_4*T_3*T_2*T_1;
    MatrixXd td6 = Rd_6*T_5*T_4*T_3*T_2*T_1;

    MatrixXd A01 = T_1;
    MatrixXd A02 = T_1*T_2;
    MatrixXd A03 = T_1*T_2*T_3;
    MatrixXd A04 = T_1*T_2*T_3*T_4;
    MatrixXd A05 = T_1*T_2*T_3*T_4*T_5;
    MatrixXd A06 = T_1*T_2*T_3*T_4*T_5*T_6;

    MatrixXd Jacobian(6,7);

    Vector3d z0, p0, pe;
    z0  << 0, 0, 1;
    p0  << 0, 0, 0;
    pe = A06.block<3,1>(0,3);

    // Ji linear velocity
    Jacobian.block<3,1>(0,0) = z0.cross(pe-p0);                                  
    Jacobian.block<3,1>(0,1) = A01.block<3,1>(0,2).cross(pe-A01.block<3,1>(0,3));
    Jacobian.block<3,1>(0,2) = A02.block<3,1>(0,2).cross(pe-A02.block<3,1>(0,3));
    Jacobian.block<3,1>(0,3) = A03.block<3,1>(0,2).cross(pe-A03.block<3,1>(0,3));
    Jacobian.block<3,1>(0,4) = A04.block<3,1>(0,2).cross(pe-A04.block<3,1>(0,3));
    Jacobian.block<3,1>(0,5) = A05.block<3,1>(0,2).cross(pe-A05.block<3,1>(0,3));
    Jacobian.block<3,1>(0,6) = A06.block<3,1>(0,2).cross(pe-A06.block<3,1>(0,3));

    // Ji angular velocity
    Jacobian.block<3,1>(3,0) = z0;                 
    Jacobian.block<3,1>(3,1) = A01.block<3,1>(0,2); 
    Jacobian.block<3,1>(3,2) = A02.block<3,1>(0,2); 
    Jacobian.block<3,1>(3,3) = A03.block<3,1>(0,2); 
    Jacobian.block<3,1>(3,4) = A04.block<3,1>(0,2); 
    Jacobian.block<3,1>(3,5) = A05.block<3,1>(0,2); 
    Jacobian.block<3,1>(3,6) = A06.block<3,1>(0,2); 
    
    
    std::cout << Jacobian << std::endl;


    MatrixXd TF(4,4);
    TF <<   cos(goal(3))*cos(goal(4)), cos(goal(3))*sin(goal(4))*sin(goal(5))-sin(goal(3))*cos(goal(5)), cos(goal(3))*sin(goal(4))*cos(goal(5))+sin(goal(3))*sin(goal(5)),      goal(0),
            sin(goal(3))*cos(goal(4)), sin(goal(3))*sin(goal(4))*sin(goal(5))+cos(goal(3))*cos(goal(5)), sin(goal(3))*sin(goal(4))*cos(goal(5))-cos(goal(3))*sin(goal(5)),      goal(1),
                        -sin(goal(4)),                                        cos(goal(4))*sin(goal(5)),                                        cos(goal(4))*cos(goal(5)),      goal(2),
                                    0,                                                                0,                                                                0,            1;
    return TF;
}


    
int main(int argc, char **argv)
{
        //Initializen the ros
    ros::init(argc, argv, "joint_state_publisher");

    //Declare the node handle
    ros::NodeHandle node;

    //Decleare a joint state publisher
    ros::Publisher joint_pub = node.advertise<sensor_msgs::JointState>("/joint_states",1);
    ros::Rate loop_rate(100);
    
    //while(ros::ok()){

    std::cout << std::endl;   
    std::cout << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    std::cout << "-Inverse Kinematics Schunk Powerball LWA 4 .6-" << std::endl;
    std::cout << "----------------------------------------------" << std::endl;
    
    VectorXd goal=input();
    MatrixXd euler=Euler(goal);
    MatrixXd T0_EE= euler;

    double q1, q2, q3, q4, q5, q6;
    double a0, a1, a2, a3, a4, a5;
    double d1, d2, d3, d4, d5, d6;
    double alpha0, alpha1, alpha2, alpha3, alpha4, alpha5;

    //  Example in cm
    double myNum[24]={alpha0=-M_PI/2.,    a0=   0,    d1=  205,  q1=-1.950,
                      alpha1=    M_PI,    a1= 350,    d2=     0,  q2=-0.717+(-M_PI/2.),
                      alpha2=-M_PI/2.,    a2=   0,    d3=     0,  q3=-2.081+(-M_PI/2.),
                      alpha3= M_PI/2.,    a3=   0,    d4=   305,  q4=2.575,
                      alpha4=-M_PI/2.,    a4=   0,    d5=     0,  q5=1.634,
                      alpha5=       0,    a5=   0,    d6=    75,  q6=0.938};

    MatrixXd T0_1(4,4); T0_1 = TF_Matrix(alpha0, a0, d1, q1);
    MatrixXd T1_2(4,4); T1_2= TF_Matrix(alpha1, a1, d2, q2);
    MatrixXd T2_3(4,4); T2_3= TF_Matrix(alpha2, a2, d3, q3);
    MatrixXd T3_4(4,4); T3_4= TF_Matrix(alpha3, a3, d4, q4);
    MatrixXd T4_5(4,4); T4_5= TF_Matrix(alpha4, a4, d5, q5);
    MatrixXd T5_EE(4,4); T5_EE= TF_Matrix(alpha5, a5, d6, q6);


    Vector3d z0, p0, pe;
    z0  << 0, 0, 350;
    p0  << 0, 0, 0;
    pe = T5_EE.block<3,1>(0,3);




    // MatrixXd T0_EE= T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_EE;

    std::cout << endl;
    std::cout << "Transfer funciont goal:" << std::endl;
    std::cout << T0_EE << std::endl;

    // Matrix solution with the 6 angles joints and 8 solutions
    MatrixXd th_ik(6,8);

    double l_a1=0, l_a2=350, l_a3 = 0; // 0, upper arm, 0
    double l_d1= 205, l_d4=305, l_d6 =75; // base. forearm, hand

    Vector3d t2; t2 << 0,0,l_d6;
    Vector3d dx= (T0_EE.block<3,3>(0,0))*t2; // Vector from sphericall wrist to tool tip

    Vector3d t4; t4 << 0,0,l_d1;
    Vector3d d_elbow = (T0_EE.block<3,1>(0,3))-dx-t4; // Vector with the tool tip distance and base distance remove
  
    double theta3= M_PI-acos(((l_a2*l_a2)+(l_d4*l_d4)-(d_elbow.norm()*d_elbow.norm()))/(2*l_a2*l_d4)); // Angle of elbow

    for (int i=0;i<=3; ++i)
    {
        th_ik(2,i)=theta3; // Elbow up
        th_ik(2,7-i)=-theta3; // Elbow down
    }

    Vector3d k1,k2,p,q;
    k1<< 0,0,1;
    k2<< 0,1,0;
    p << -l_d4*sin(th_ik(2,0)), 0 , l_a2+l_d4*cos(th_ik(2,0));// ojo se debe poner en el sin y cos multiplicar por *180/M_PI o no
    q = d_elbow;

    Vector4d theta12v1;
    theta12v1= subproblema2(k1,k2,p,q);// 2 solutions for theta3 Elbow up

    p << -l_d4*sin(th_ik(2,4)), 0 , l_a2+l_d4*cos(th_ik(2,4));
    Vector4d theta12v2;
    theta12v2=subproblema2(k1,k2,p,q);// 2 solutions for theta3 Elbow down

    th_ik(0,0)=theta12v1(0);
    th_ik(0,1)=theta12v1(1);
    th_ik(0,2)=theta12v1(0);
    th_ik(0,3)=theta12v1(1);
    th_ik(0,4)=theta12v2(0);
    th_ik(0,5)=theta12v2(1);
    th_ik(0,6)=theta12v2(0);
    th_ik(0,7)=theta12v2(1);

    th_ik(1,0)=theta12v1(2);
    th_ik(1,1)=theta12v1(3);
    th_ik(1,2)=theta12v1(2);
    th_ik(1,3)=theta12v1(3);
    th_ik(1,4)=theta12v2(2);
    th_ik(1,5)=theta12v2(3);
    th_ik(1,6)=theta12v2(2);
    th_ik(1,7)=theta12v2(3);

    // find theta 4 5 and 6
    for(int i=0; i<=1; ++i)
    {
        double th_1=th_ik(0,i);
        double th_2=th_ik(1,i);
        double th_3=th_ik(2,i);
        
        MatrixXd T01(4,4); T01= TF_Matrix(    -M_PI/2.,    0, l_d1, th_1);
        MatrixXd T12(4,4); T12= TF_Matrix(        M_PI, l_a2,    0, th_2-M_PI/2.);
        MatrixXd T23(4,4); T23= TF_Matrix(    -M_PI/2.,    0,    0, th_3-M_PI/2.);

        MatrixXd T03 = T01 * T12 * T23;
  
        // extrae 3 filas por una columna desde la posicion de fila 0 columna 3
        // Matrix3d Mtemp= T0_EE.block<3,1>(0,3);
        Vector3d Mt=-T03.block<3,3>(0,0).transpose()*T03.block<3,1>(0,3);

        Matrix4d R46;
        R46 <<  T03(0,0),T03(1,0),T03(2,0),Mt(0),
                T03(0,1),T03(1,1),T03(2,1),Mt(1),
                T03(0,2),T03(1,2),T03(2,2),Mt(2),
                       0,         0,       0,    1;

        Matrix4d Twrist=R46*T0_EE;

        th_ik(3,i)=atan2(-Twrist(1,2),-Twrist(0,2));
        th_ik(4,i)=acos(Twrist(2,2)); 
        th_ik(5,i)=atan2(-Twrist(2,1),Twrist(2,0));

        th_ik(3,i+2)=atan2(Twrist(1,2),Twrist(0,2));
        th_ik(4,i+2)=-acos(Twrist(2,2));
        th_ik(5,i+2)=atan2(Twrist(2,1),-Twrist(2,0));
    }

    for(int i=4; i<=5; ++i)
    {
        double th_1=th_ik(0,i);
        double th_2=th_ik(1,i);
        double th_3=th_ik(2,i);
        
        MatrixXd T01(4,4); T01= TF_Matrix(    -M_PI/2.,    0, l_d1, th_1);
        MatrixXd T12(4,4); T12= TF_Matrix(        M_PI, l_a2,    0, th_2-M_PI/2.);
        MatrixXd T23(4,4); T23= TF_Matrix(    -M_PI/2.,    0,    0, th_3-M_PI/2.);

        MatrixXd T03 = T01 * T12 * T23;
  
        // extrae 3 filas por una columna desde la posicion de fila 0 columna 3
        // Matrix3d Mtemp= T0_EE.block<3,1>(0,3);
        Vector3d Mt=-T03.block<3,3>(0,0).transpose()*T03.block<3,1>(0,3);

        Matrix4d R46;
        R46 <<  T03(0,0),T03(1,0),T03(2,0),Mt(0),
                T03(0,1),T03(1,1),T03(2,1),Mt(1),
                T03(0,2),T03(1,2),T03(2,2),Mt(2),
                       0,         0,       0,    1;

        Matrix4d Twrist=R46*T0_EE;

        th_ik(3,i)=atan2(-Twrist(1,2),-Twrist(0,2));
        th_ik(4,i)=acos(Twrist(2,2)); 
        th_ik(5,i)=atan2(-Twrist(2,1),Twrist(2,0));

        th_ik(3,i+2)=atan2(Twrist(1,2),Twrist(0,2));
        th_ik(4,i+2)=-acos(Twrist(2,2));
        th_ik(5,i+2)=atan2(Twrist(2,1),-Twrist(2,0));
    }

    std::cout << "Jacobian: " << std::endl;
    Jacobian(th_ik.block<6,1>(0,0));


    std::cout << std::endl;
    std::cout << "Joints solutions table without limits: " << std::endl;
    std::cout << "|| Solution1 | Solution2 | Solution3 | Solution4 | Solution5 | Solution6 | Solution7 | Solution8 ||" << std::endl;
    std::cout << th_ik << std::endl;
        
    // std::cout << "theta limits: " << std::endl;
    VectorXd th_l(6); th_l << 170,110,155.5,170,140,170;
    VectorXd th_limit=th_l*M_PI/180;


    int counter=0;
    MatrixXd temp_out(6,8);

    for(int i=0; i<=7; ++i)
    {
        int temp=0;
        for(int j = 0; j <=5; j++)
        {        
            if(abs(th_ik(j,i))<th_limit(j))
            {
                temp=temp+1;
            }
        }
        if(temp==6)
        {
            
            for(int k = 0; k <=5; k++)
            {
                temp_out(k,counter)=th_ik(k,i);
            }
            counter=counter+1;
        }           
    }

    MatrixXd temp_out_lim=temp_out.leftCols(counter);


    if(temp_out_lim.size()/6==0)
    {
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << "-----------------" << std::endl;
        std::cout << "-NO IK SOLUTIONS-" << std::endl;
        std::cout << "-----------------" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
        std::cout << "----------------------------" << std::endl;
        std::cout << "-   !!!FINISH PROGRAM!!!   -" << std::endl;
        std::cout << "----------------------------" << std::endl;
        std::cout << std::endl;
        std::cout << std::endl;
    }
    else
    {
        //Define the joint state
        sensor_msgs::JointState joint_state;

        joint_state.name.push_back("arm_1_joint");
        joint_state.name.push_back("arm_2_joint");
        joint_state.name.push_back("arm_3_joint");
        joint_state.name.push_back("arm_4_joint");
        joint_state.name.push_back("arm_5_joint");
        joint_state.name.push_back("arm_6_joint");

        std::cout << std::endl;
        std::cout << "----------------------------" << std::endl;
        std::cout << "-   SOLUTIONS WITH LIMITS  -" << std::endl;
        std::cout << "----------------------------" << std::endl;
        std::cout << std::endl;

        joint_state.position.resize(6);

            for(int j = 0; j < counter; j++)
            {
                std::cout << std::endl;
                std::cout << "-----------------" << std::endl;
                std::cout << "-SOLUTION: " << j+1 <<"    -" << std::endl;
                std::cout << "-----------------" << std::endl;
                for(int i = 0; i < 6; i++)
                {
                    joint_state.position[i]=temp_out_lim(i,j);  
                }
                // std::cout<<joint_state<<std::endl;

                joint_state.header.stamp= ros::Time::now();
                //command the robot to move
                joint_pub.publish(joint_state);
                
                sleep(3);
            }
            // ros::spinOnce();
            // loop_rate.sleep();
        }
    // }

    std::cout << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << "-   !!!FINISH PROGRAM!!!   -" << std::endl;
    std::cout << "----------------------------" << std::endl;
    std::cout << std::endl;
    
    return 0;

}

