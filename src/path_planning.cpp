#include "ros/ros.h"
#include <iostream>
#include "math.h"

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "macaron/Floats.h"
#include "macaron/base_frame.h"
#include "macaron/erp42_write.h"


#define PI acos(-1)

//-------------------input value--------------------------//\


//from lanetraker
double chasun=0;
double deviation = 0;

//from lidar........??
double scan_data[2][180];
double carte_data[3][360];
double lane_data[2][23];
double cost[4][57];

//from gps_txt reader
double localized_p[2] = {0,0};   //현재 내 위치에서 가장 가까운 base path
double localized_p_before[2];   
double localized_vec;            //도북기준 각도, rad
double destination_p[2]; 
double destination_vec;          //도북기준 각도, rad
double ld_p[2];
double ld_vec;                   //도북기준 각도, rad
double qi;                       //초기 오프셋
double qf = 0;                   //목표 오프셋
double qf_before;                //이전 목표 오프셋

//from gps_RTK
double x_tm;
double y_tm;

//from IMU
double yaw; //도북기준 yaw, rad

//-------------------write value--------------------------//
bool write_E_stop;
int write_gear=0;
int write_speed=0;
int write_brake=1;
int write_steer=0;

//----------------control property--------------------------//
double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
int candidate_num = 7;           // 중심 제외 단방향으로의 후보경로
double search_range  = 2;        //단방향 후보경로 탐색범위(m) ,q방향
double ld = 2;                   //ld (m), 정수로 설정할 것
double candidate_path_leng = 10; //후보경로 s길이 (m), 정수로 설정할 것

double w_offset      = 0.5;
double w_safety      = 5.0; 
double w_consistency = 0.1;

double alpha = 0;                // 현재 방향과 ld포인트 사이의 각도 차이 , rad
macaron::erp42_write erp42;


double gx(int i,double sigma)
{
    double gausian_factor = exp(-0.01*double(i)*double(i)/2/sigma/sigma)/sqrt(2*PI*sigma);
    return gausian_factor;
}


double radians(double deg)
{
    return deg / double(180) * PI; 
}


struct Quaternion
{
    double w, x, y, z;
};
struct EulerAngles
{
    double roll, pitch, yaw;
};
EulerAngles ToEulerAngles(Quaternion q)
{
    EulerAngles angles;
    // roll (x-axis rotation)
    double sinr_cosp = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);
    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        angles.pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = asin(sinp);
    // yaw (z-axis rotation)
    double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);  
    angles.yaw = atan2(siny_cosp, cosy_cosp);
    return angles;
}


void laser_scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    for(int i=0; i < 811; i++)
    {
        scan_data[0][i]=msg->ranges[i];
        scan_data[1][i]=0.3333*i-135;
        if(isinff(scan_data[0][i])) 
            scan_data[0][i] = 25;
    }
}


void laneCallBack(const macaron::Floats::ConstPtr& lane)
{
    lane->mid_point_vector;
    lane->testset.data;
    lane->testset.layout;
    deviation=lane->deviation;
    float dstride0 = lane->testset.layout.dim[0].stride;
    float dstride1 = lane->testset.layout.dim[1].stride;
    float h = lane->testset.layout.dim[0].size;
    float w = lane->testset.layout.dim[1].size;
    double l_cost=1;
    for (int i=0; i<h; i++)
    {
        for (int j=0; j<w; j++)
        {
            lane_data[i][j]=lane->testset.data[dstride1*i+j];
        }
    }

    double a=0;
    for (int j=0; j<23; j++) 
    {
        a+=lane_data[1][j];
    }

    //캘리브레이션 코드가 절실하다 빠른시일내에 작성하도록
}


void imuCallBack(const sensor_msgs::Imu::ConstPtr& msg) // 지금은 도북기준이 아니라 시작위치 기준이다. 도북기준으로 바꿀 것
{
    Quaternion q;
    q.x = msg->orientation.x;
    q.y = msg->orientation.y;
    q.z = msg->orientation.z;   
    q.w = msg->orientation.w;
    EulerAngles e = ToEulerAngles(q);
    yaw = e.yaw; //rad
}


void gpsCallBack(const sensor_msgs::NavSatFix::ConstPtr& fix) // lon, lat -> tm, gpstxt reader에서 받아도 될듯. 굳이 두번 계산?
{
    double lon =  fix->longitude;
    double lat =  fix->latitude;

    double e_2,e,C,T,A,N,M,M_0;    
    double st_lat = 38.0; // standard latitude
    double st_lon = 127.0; // standard longitude
    double k_0 =1.0;
    double a = 6378137.0;
    double b = 6356752.31;
    double f =(a-b)/a;
    double d_y = 200000.0;
    double d_x = 600000.0;

    e = (a*a - b*b) / (a*a);
    e_2 = (a*a - b*b) / (b*b);
    M = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*pow(e,6.0)/256.0)*radians(lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)*sin(2*radians(lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)*sin(4.0*radians(lat)) - 35.0*e*e*e/3072.0*sin(6.0*radians(lat)));
    C = (e / (1-e))*cos(radians(lat));
    T = pow(tan(radians(lat)),2);
    A = (radians(lon - st_lon))*cos(radians(lat));
    N = a/sqrt(1-(e)*(sin(radians(lat))*sin(radians(lat))));
    M_0 = a * ((1 - e/4.0 - 3.0*e*e/64.0 - 5.0*e*e*e/256.00)*radians(st_lat) - (3.0*e/8.0 + 3.0*e*e/32.0 + 45.0*e*e*e/1024.0)*sin(2.0*radians(st_lat)) + (15.0*e*e/256.0 + 45.0*e*e*e/1024.0)*sin(4.0*radians(st_lat)) - 35.0*e*e*e/3072.0*sin(6.0*radians(st_lat)));
    y_tm = (d_y+k_0*N*(A + (A*A*A/6.0)*(1-T+C) + (A*A*A*A*A/120.0) * (5.0 - 18.0*T + T*T + 72.0*C - 58.0*e_2)));
    x_tm = (d_x + k_0*(M - M_0 + N*tan(radians(lat))*(A*A/2.0 + (A*A*A*A/24.0)*(5.0-T+9.0*C+4.0*C*C)+(A*A*A*A*A*A/720.0)*(61.0-58.0*T+T*T+600.0*C-330.0*e_2))));
}


void pathCallBack(const macaron::base_frame::ConstPtr& path) 
{
    localized_p_before[0] = localized_p[0];
    localized_p_before[1] = localized_p[1];
    localized_p[0]        = path->s_x[0];
    localized_p[1]        = path->s_y[0];
    localized_vec         = path->s_a[0];
    destination_p[0]      = path->s_x[int(candidate_path_leng)];
    destination_p[1]      = path->s_y[int(candidate_path_leng)];
    destination_vec       = path->s_a[int(candidate_path_leng)];
    ld_p[0]               = path->s_x[int(ld)];
    ld_p[1]               = path->s_y[int(ld)];
    ld_vec                = path->s_a[int(ld)];
    qi                    = path->distance;
}


void generate_candidate_path(int index)
{
    qf_before = qf;
    qf = search_range / double(candidate_num) * double(index);
    double theta = yaw - destination_vec;
    double ds = candidate_path_leng;
    double a1 = (ds * tan(theta) - 2 * (qf - qi)) / (ds * ds * ds);
    double a2 = (qf - qi) / (ds * ds);
    

    int s = ld;
    double offset = (s - ds)*(s - ds) * (a1*s - a2) + qf;
    double x = ld_p[0] + offset * cos(ld_vec + PI/2);
    double y = ld_p[1] + offset * sin(ld_vec + PI/2);
    alpha = yaw - atan2(y_tm - y, x_tm - x); //시계반대가+ (오른손나사), rad
}


double path_cost()
{
    //offset cost
	double cost_offset = fabs(qf);

	//safty cost
    double cost_safety = 0;

	//consistency cost, 시작시 consistency cost 무시
	double common_s = candidate_path_leng - sqrt(pow(localized_p[0] - localized_p_before[0], 2) + pow(localized_p[1] - localized_p_before[1], 2));
	double cost_consistency = fabs(qf - qf_before) * common_s * 0.5;

	return w_offset*cost_offset + w_safety*cost_safety + w_consistency*cost_consistency;
}


void toERP42()
{
    write_steer = 71 * atan2(2*wheel_base*sin(-alpha), ld)*180/PI + 0.5; //conversion with rounding
    if(write_steer > 2000)
        write_steer = 2000;
    else if(write_steer < -2000)
        write_steer = -2000;

    erp42.write_speed = 30;
    erp42.write_steer = write_steer;
    erp42.write_gear  = 0;
    erp42.write_brake = 1;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"path_planning");
    ros::NodeHandle nh;
    ros::Rate loop_rate(50);

    ros::Publisher control = nh.advertise<macaron::erp42_write>("/erp_write", 1);
    ros::Subscriber lidar_sub   = nh.subscribe("/scan",100, laser_scan_Callback);
    ros::Subscriber lane_sub    = nh.subscribe("/lane",10, laneCallBack);
    ros::Subscriber pp_gps_sub  = nh.subscribe("/fix",10, gpsCallBack);
    ros::Subscriber pp_imu_sub  = nh.subscribe("/imu_rpy",1,imuCallBack);
    ros::Subscriber gpspath_sub = nh.subscribe("/base_frame",10, pathCallBack);

    while(ros::ok)
    {
        ros::spinOnce();
        
        int proper_index;
        double TEMP = 100; //임의의 큰 수
        for(int index = -candidate_num; index < candidate_num; index++)
        {
            generate_candidate_path(index);
            if(path_cost() < TEMP)
            {
                TEMP = path_cost();
                proper_index = index;
            }
        }
        generate_candidate_path(proper_index);
        toERP42();
        control.publish(erp42);     
        loop_rate.sleep();
    }
    return 0;
}
