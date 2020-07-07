
/***
 20헤르츠 버그 수정필요
 왼쪽 엔코더 편향상태 보정공식 적용필요
 */

#include <ros/ros.h>
#include <serial/serial.h>
#include <macaron/erp42_read.h>
#include <macaron/erp42_write.h>
#include "key_zzino/wasd.h"

#define PI acos(-1)
#define MAX 18

serial::Serial ser;

//imu value
double yaw_imu = 0, yaw0_imu = 0;
int firstrun = 1;

//write value
int steer=0;
bool E_stop;
uint8_t steer1,steer2,speed,brake,gear;

double wheel_base = 1.040, tread = 0.985, width = 1.160; //macaron property
double ENC_saver[4];
double vel_saver[4];
double dt=0;
void write_callback(const macaron::erp42_write::ConstPtr& write){

    E_stop=write->write_E_stop;
    gear=write->write_gear;
    steer= -write->write_steer; //실수인데.. 저희 차가 z축이 아래방향이다. 
    steer1=(steer/256); //몫
    steer2=(steer%256); // 나머지 // 무슨소리?
    if(steer<0) steer1=steer1-1; //보수 만드는 과정
    speed= write->write_speed;
    brake= write->write_brake;
}
//  씨리얼 통씬... 
//2삐트까 나오면 이쪠 씨짞이꾸나. 
//우린 씨리얼 통신을  stx 쫘라라라라라락  첫번째는 뭐고 두번째는 스피드고..시리얼통신?
// steer -2000~2000 ....2바이트로 즉 2의 16승으로 보내갰다.
//한번에 1바티으밖에 못보내니까 몫은 앞 바이트 나머지는 뒤 바이트
// 음수 란걸 알ㄹ때 보수란걸 사용.
//2진수 보수사용. 1에서 빼주는걸 보수라고 한다.
//


int main (int argc, char** argv){
    ros::init(argc, argv, "serial_example_node");
    ros::NodeHandle nh;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Subscriber write_sub = nh.subscribe("erp_write", 1, write_callback);
    ros::Subscriber keycode_sub=nh.subscribe("key_write", 1, write_callback);
    ros::Publisher read_pub = nh.advertise<macaron::erp42_read>("erp_read", 1);
    macaron::erp42_read erp42_state;
    try
    {
        ser.setPort("/dev/erp42");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
        }

    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
        return -1;
    }

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");

    }else{
        return -1;
    }

    ros::Rate loop_rate(50);
        uint8_t answer[17]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
        uint8_t answer_quere[MAX]={0,};
        uint8_t answer_tester[1]={0x00};

     if(ser.available()){
           // ROS_INFO_STREAM("Reading from serial port");

            while(answer_tester[0]!=0x53){
                ser.read(answer_tester,1); 
//                printf("%x ",answer_tester[0]);
            }
            answer_tester[0]={0x00};
            ser.read(answer,17);

            if(answer[0]==0x54 && answer[1]==0x58)     
            {
                if(answer[15]==0x0D && answer[16]==0x0A)
            { //  for(int i=0;i<17;i++)         printf("%x ",answer[i]);
              //  printf("\n");
                    erp42_state.read_AorM=bool(answer[2]);
                    erp42_state.read_E_stop=bool(answer[3]);
                    erp42_state.read_gear=answer[4];
                    erp42_state.read_speed=int(answer[5]);
                    erp42_state.read_brake=int(answer[9]);
                    erp42_state.read_ENC=int(answer[13])*256*256*256+int(answer[12])*256*256+int(answer[11])*256+int(answer[10]); //컴퓨터가 보내는 엔코더값을 우리가 알아들으려고 2에 8승 곱해주는것
                    ENC_saver[0]=erp42_state.read_ENC; //차량으로부터 엔코더값을 읽어오는거겟죠?
                    ENC_saver[1]=erp42_state.read_ENC; // write는 이렇게 굴러가라 이것아! 하고 요청하는것
                    ENC_saver[2]=erp42_state.read_ENC;
                    ENC_saver[3]=erp42_state.read_ENC;

            }
        }
     }


    while(ros::ok()){
    ros::spinOnce();
    last_time = current_time;
    current_time = ros::Time::now();
    dt = (current_time - last_time).toSec(); //저번루프와 이번루프사이의 시간차이, tosec은 로스내장

    uint8_t a;
    a++;
	
    uint8_t ask[14]={0x53,0x54,0x58,0x01,0x00,gear,0x00,speed,steer1,steer2,brake,a,0x0D,0x0A};
    ser.write(ask,14);//S  0x53

        if(ser.available()){ //시리얼 값이 들어온다면!
            ser.read(answer_quere,18);
            
            for(int i=0; i<MAX;i++){
                printf("%x ",answer_quere[i]);
            }
            printf("%f \n",dt);
        }

    if(answer_quere[0]!=0x53 || answer_quere[1]!=0x54 || answer_quere[2]!=0x58 || answer_quere[16]!=0x0D || answer_quere[17]!=0x0A)
    {
        ser.flushOutput();
        for(int i=0; i<MAX;i++){
            answer_quere[i]={0x00};
        }
        while(answer_tester[0]!=0x0A){
            ser.read(answer_tester,1);
            ROS_INFO("DORMAMU %x",answer_tester[0]);
        } //시리얼 플러쉬 방지...데이터 처리속도 가 못따라가서 처음거받고 버린다.
    }

    else{
        erp42_state.read_AorM=bool(answer_quere[3]);
        erp42_state.read_E_stop=bool(answer_quere[4]);
        erp42_state.read_gear=answer_quere[5];
        erp42_state.read_speed=int(answer_quere[6]);
        erp42_state.read_brake=int(answer_quere[10]);
        erp42_state.read_ENC=int(answer_quere[14])*256*256*256+int(answer_quere[13])*256*256+int(answer_quere[12])*256+int(answer_quere[11]);
	//4개의 회전수 저장해놓고 ..원하는건 회전수가 아니라 속도.    
        for(int i=2; i>=0; i--)
        {
            ENC_saver[i+1]=ENC_saver[i]; // 바이트단위라서 한개씩 미는건가??
        }
        ENC_saver[0]=erp42_state.read_ENC; // 왜 
        erp42_state.read_velocity=0.01651*((ENC_saver[0]-ENC_saver[2]))/2/dt; //왜 곱하지? dt 로 나눈거
  	//속도니까....결국 저게 회전수차이?..0에서 2빼본건가 그냥? // 2로 나눈건 그냥 반지름
        for(int i=2; i>=0; i--)
        {
        vel_saver[i+1]=vel_saver[i];
        }
        vel_saver[0]=erp42_state.read_velocity;

        erp42_state.read_accel=((vel_saver[0]-vel_saver[1]))/dt;
        erp42_state.read_steer=int(answer_quere[9])*256+int(answer_quere[8]);

        if(erp42_state.read_steer>32768) erp42_state.read_steer=erp42_state.read_steer-65536+1;
 	//위에 이프문..보수이다!
        erp42_state.read_yaw=erp42_state.read_yaw+0.01652*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI; //회전수차이..
        erp42_state.read_s=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*cos( erp42_state.read_yaw/180/PI); //사실 y임
        erp42_state.read_l=erp42_state.read_s+0.01652*(ENC_saver[0]-ENC_saver[1])*sin( erp42_state.read_yaw/180/PI); //사실 x임
        read_pub.publish(erp42_state);
        ser.flush();
        loop_rate.sleep();

    }
                  //엔코더로 yaw를 구하는 식. 보정필요, IMU로 대체.
                    //erp42_state.read_yaw=erp42_state.read_yaw+0.01651*(ENC_saver[0]-ENC_saver[1])*tan(double(erp42_state.read_steer)/71*PI/180)/wheel_base*180/PI;
 }
}            
        
