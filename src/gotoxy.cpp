#include "ros/ros.h"
#include <iostream>
#include "math.h"
#include "macaron/erp42_write.h"
//cmake파일이랑 메시지 파일 아직 안만듬

//pure pursuit//
//delta를 alpha에 맞춰야한다.!!!
//코드의 최종 목표는 speed steer값을 시리얼익잼플 노드에 갖다주는것.
//speed 는 안전빵으로 10으로 설정
//steer 는 delta 즉 시간에 대한 함수로 설정
//if(엔코더값 = (gx,gy)~(0,0)사이 거리값과 일치한다면 그만하세요!)
//
//짐벌락 피하기 위해 쿼터니언을 사용할수있다,
// 쿼터니언  q = d + ai + bj +ck = (d,(a,b,c))
// 즉 4자유도..
// 쿼터니언으로 회전 표현하기.....


//epsilon 값 찾는게 관건으로 보인다...다른 pure pursuit자료 찾아보기.

//(-2,5) 를 설정한다고 하자.
//그럼 뒷바퀴는 (0,0)으로 설정..이것도 맞춰줘야하나?
//
//ld, alpha(sin 형으로 적용),R,delta, time, epsilon

//epsilon 찾는게 관건인데.. epsi


//ld를 따라간다.. 함수를 따라가는데 사실은 점ㅁ을 따라간다.
// 축간거리 L은 1040 mm // 1.04 m
//alpha는 현재 앞바퀴가 보는 각도 delta ? 차이점.
// alpha를 delta 에 맞추려하는것
#define pi acos(-1)
int speed=0;
int steer=0;
bool write_E_stop;
int write_gear=0;
int write_speed=10;
int write_break=1;
int write_steer=0;

int goal_x = -3;
int goal_y = 4;

int rear_x = 0;
int rear_y = 0;

double L = 1.04;
double ld = 5; //ld >> meter 단위로 설정
double eld = 3;

double alpha = asin(eld/ld);

double delta = atan((2*1.04*sin(alpha)/ld));
//최대 28도 까지 돌아간다..따라서 실제 조향각 1도당 71씩 곱해주면 된다(28*71 = 2000)


int main(int argc, char **argv)
{
ros::init(argc, argv, "gotoxy");
ros::NodeHandle nh;

ros::Publisher gogoxy_pub = nh.advertise<macaron::erp42_write>("erp_write_msg", 1);
ros::Rate loop_rate(10);

macaron::erp42_write msg;


while (ros::ok())
{
write_steer = (int) 71*((delta * 180)/pi);  

msg.write_E_stop = write_E_stop;
msg.write_gear = write_gear;
msg.write_speed = write_speed;
msg.write_brake = write_break;
msg.write_steer = write_steer;

gogoxy_pub.publish(msg);

loop_rate.sleep();

}

return 0;

}

