#include "ros/ros.h"
#include "math.h"
#include "geometry_msgs/Vector3.h"
#include "macaron/erp42_write.h"
#include "macaron/erp42_read.h"

#define PI acos(-1)

/*
속도제어 필요
원자계 필요
s,l 성공 theta 제어 보정필요

로컬에서 차량 각도구하기 : v*tan(kw)/W
로컬에서 좌우 이동거리 : v*sin(theta) 적분
로컬에서 전면 이동거리 : v*cos(tetha) 적분
*/

//-------------------write value--------------------------//
bool write_E_stop;
int write_gear=0;
int write_speed=0;
int write_brake=1;
int write_steer=0;
//-------------------input value--------------------------//
double macaron_look_ahead=2;
double macaron_offset=-0.5;
double macaron_orient=0*PI/180;
//from erp_reader
double velocity,accel,s,l,yaw;
int AroM,brake,steer,speed,E_stop,gear,ENC,angle;

//----------------macaron property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property

void erp42_reader(const macaron::erp42_read::ConstPtr& msg)
{
    AroM=msg->read_AorM;
    brake=msg->read_brake;
    steer=msg->read_steer;
    speed=msg->read_speed;
    E_stop=msg->read_E_stop;
    gear=msg->read_gear;
    ENC=msg->read_ENC;
    velocity=msg->read_velocity;
    accel=msg->read_accel;
    s=msg->read_s;
    l=msg->read_l;
    yaw=msg->read_yaw;
}

void sltCallback(const geometry_msgs::Vector3::ConstPtr& s_l_theta)
{
    macaron_look_ahead = s_l_theta->x;
    macaron_offset = s_l_theta->y;
    macaron_orient= s_l_theta->z;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"vel_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher control;
    control = nh.advertise<macaron::erp42_write>("erp_write", 1);
    macaron::erp42_write erp42;
    ros::Subscriber s_l_theta_sub = nh.subscribe("s_l_theta",1, sltCallback);
    //EKF에서 나오는 헤딩값/속도 가속도값 받는 섭스크라이버 작성필요

    while(ros::ok)
    {
        ros::spinOnce();
        //s, l, theta에서 speed, steer로 변환, PID제어 추가 필요
        double a1=(macaron_look_ahead* tan(macaron_orient) - 2 * (macaron_offset)) / (macaron_look_ahead * macaron_look_ahead *macaron_look_ahead);
        double a2 = (macaron_offset)/ (macaron_look_ahead * macaron_look_ahead);

        double offset = (s - macaron_look_ahead)*(s - macaron_look_ahead) * (a1*s - a2) + macaron_offset; // offset   
        double T = 180/PI*atan((s- macaron_look_ahead)*(3*a1*s-2*a2-a1*macaron_look_ahead));
        if(s>macaron_look_ahead) T=0;

        double a=0;
        double K=2*sin(T*PI/180)/macaron_look_ahead;

        write_steer=71*atan(2*wheel_base*sin(double(T)*PI/180)/macaron_look_ahead)*180/PI; //pure pursuit 적용 모델이구나!
        if(write_steer>2000) write_steer=2000;
        else if(write_steer<-2000) write_steer=-2000;


        erp42.write_speed=20;
        if(s>macaron_look_ahead) erp42.write_speed=0;

        erp42.write_steer=write_steer;
        erp42.write_gear=0;
        erp42.write_brake=1;
        control.publish(erp42);     
        loop_rate.sleep();
    }
    return 0;
}
