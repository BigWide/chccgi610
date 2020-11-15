#include<ros/ros.h>
#include<iostream>
#include<serial/serial.h>
#include<sensor_msgs/Imu.h>
#include<chccgi610/chccgi610nav.h>

//$GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyro z,acc x,acc y,acc z,Lattitude,Longitude,Altitude,Ve,Vn,Vu,Baseline,NSV1,NSV2,Status,Age,Warming,Cs<CR><LF>
//$GPCHC,wwww,ssssss.ss,hhh.hh,-pp.pp,-rrr.rr,-ggg.gg,-ggg.gg,-ggg.gg,-a.aaaa,-a.aaaa,-a.aaaa,-ll.lllllll,-ll.lllllll,-aaaa.aa,-eee.eee,-nnn.nnn,-uuu.uuu,-uuu.uuu,nn,nn,ss,aa,ww,*hh<CR><LF>
//$GPCHC,2063,11273.94,179.78,-0.21,-1.75,-0.15,-0.01,0.02,0.0304,-0.0036,0.9980,31.0246688,121.4355940,17.46,0.002,0.035,-0.012,0.036,22,20,90,1,
//$GPCHC每帧的数据长度最大为183位字符，包括头尾字符

struct FormatGPCHC
{
	int   	GPSWeek;//自1980-1-6至当前的星期数（格林尼治时间）
	double 	GPSTime;//自本周日0:00:00至当前的秒数（格林尼治时间）
	double  Heading;//偏航角，0 - 359.99
	double  Pitch;//俯仰角，-90 - 90
	double 	Roll;//横滚角，-180 - 180
    double  gyro_x;//x轴角速度deg/s
    double  gyro_y;//y轴角速度deg/s
    double  gyro_z;//z轴角速度deg/s
    double  acc_x;//x轴加速度g
    double  acc_y;//y轴加速度g
    double  acc_z;//z轴加速度g
	double  Lat;//纬度，-90 - 90
	double 	Lon;//经度，-180 - 180
	double 	Alt;//高度，m
	double 	Ve;//东向速度，m/s
	double 	Vn;//北向速度，m/s
	double 	Vu;//天向速度，m/s
    double 	V;//车辆速度，m/s
	int		NSV1;//天线1卫星数
	int		NSV2;//天线2卫星树
	//系统状态，
	//低半字节：0-初始化;1-卫导模式;2-组合导航模式;3-纯惯导模式;
	//高半字节：0-不定位不定向;1-单点定位定向;2-伪距差分定位定向；3-组合推算;4-RTK稳定解定位定向;5-RTK浮点解定位定向;6-单点解定位不定向;7-伪距差分定位不定向;8-RTK稳定解定位不定向;9-RTK浮点解定位不定向
	int		Status;
    int     Age;//差分延时
    int     Warming;
};

//全局变量
serial::Serial ser;//声明串口对象 
int StateParser = 0;//解析处理状态机状态
int CntByte = 0;//用于记录OneFrame中的实际数据长度
int PosDelimiter[24] = {0};//用于记录分隔符位置
int field_len[23];//字符串长度
int CntDelimiter = 0;//分隔符计数
unsigned char rbuf[500];//接收缓冲区，要足够大，需要通过测试得出
char OneFrame[250];//存放一帧数据，长度大于115即可，这里取200
double gpstime_pre;//上一个gps时间
double delta_t;
char str[3];
unsigned int tmpint = 0;
int cscomputed;//计算得到的校验，除去$*hh<CR><LF>共6个字符
int csreceived;//接收到的校验
char strtemp[3];
char temp_field[30] = {0};
sensor_msgs::Imu chccgi610_imu;
chccgi610::chccgi610nav msg_chccgi610_nav;//自定义消息

ros::Publisher msg_nav_pub;
ros::Publisher msg_imu_pub;
/*****************************
  功能：计算校验，字符串中所有字符的异或
  返回：返回一个无符号整数
  输入参数：<1>字符串指针，<2>字符串长度（指有效长度，不包括字符串结束标志）
  输出参数：校验结果
******************************/
unsigned int GetXorChecksum(const char *pch, int len) 
{
    unsigned int cs = 0;
    int i;

    for(i=0; i<len; i++)
        cs ^= pch[i];

    return cs;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "chccgi610");
    ros::NodeHandle nh;
	std::string port_name;
	ros::param::get("~port_name", port_name);

	msg_nav_pub = nh.advertise<chccgi610::chccgi610nav>("/chccgi610_nav",1000,true);
	msg_imu_pub = nh.advertise<sensor_msgs::Imu>("/chccgi610_imu",1000,true);
	try 
    { 
    	//设置串口属性，并打开串口 
        ser.setPort(port_name); 
        ser.setBaudrate(460800); 
        serial::Timeout to = serial::Timeout::simpleTimeout(1000); //超时定义，单位：ms
        ser.setTimeout(to); 
        ser.open(); 
    } 
    catch (serial::IOException& e) 
    { 
		std::cout<<port_name+" open failed，please check the permission of port ,run command \"sudo chmod 777 "+port_name+"\" and try again！"<<std::endl;
		getchar(); 
		return -1;
	}

	 //检测串口是否已经打开，并给出提示信息 
    if(ser.isOpen()) 
    {
		std::cout<<port_name+" open successed！"<<std::endl;
    } 
    else 
    { 
		std::cout<<port_name+" open failed！"<<std::endl;
		getchar(); 
		return -1;
	} 

	ros::Rate loop_rate(100);//设置循环频率为100Hz
    ser.flushInput();//在开始正式接收数据前先清除串口的接收缓冲区 	
	memset(OneFrame, 0, sizeof(OneFrame));//清空gps字符串
	int framecnt = 0;
	CntByte = 0;//指向OneFrame的第一个位置
	while (ros::ok())
	{
		int i, j;
		int start;//当前位置
		int pos;//下一个分隔符的位置
		int numinbuf;
		int numgetted;
		
		try
		{
			numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
			//std::cout<<"串口缓冲区的数据有"<<numinbuf<<"个"<<std::endl;
			//initrd.img.oldCLEAR();
			//printf("bytes in buf = %d\n",numinbuf);
		}
		catch (serial::IOException& e)
		{
			std::cout<<"Port crashed！ Please check cable!"<<std::endl;
			getchar(); 
			return -1;
		}

		if(numinbuf > 0)//串口缓冲区有数据
		{ 
            numgetted = ser.read(rbuf, numinbuf);//串口缓冲区数据读到rbuf中
			if(numgetted == numinbuf)//取回的数据个数与缓冲区中有的数据个数相同，说明读串口成功
			{
				for(int i=0; i<numgetted; i++)//对收到的字符逐个处理
				{
					//在一帧数据的接收过程中，只要遇到非$GPCHC帧头就重新开始
					//此处理具有最高优先级，会重置状态机
					if(rbuf[i]=='$' &&rbuf[i+3] != 'C'&&rbuf[i+4] != 'H'&&rbuf[i+5] != 'C')
					{
						memset(OneFrame, 0, sizeof(OneFrame));
						StateParser = 0;

						break;//中断循环
					}

					//正常处理过程
					switch (StateParser)
					{
						//等待语句开始标志'$'
						case 0:
							if(rbuf[i] == '$'&&rbuf[i+3] == 'C'&&rbuf[i+4] == 'H'&&rbuf[i+5] == 'C')//收到语句开始标志
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								OneFrame[0] = '$';
								CntByte = 1;//开始对帧长度的计数
								StateParser = 1;
							}
							break;
						
						//寻找帧头"$GPCHC,"
						case 1:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
						
							if(rbuf[i]==',')
							{
								if(strncmp(OneFrame, "$GPCHC,", 7) == 0)
								{
									CntDelimiter = 0;//分隔符计数从0开始
									PosDelimiter[0] = CntByte - 1;//记录分隔符在OneFrame中的位置
									//std::cout<<"PosDelimiter[0]"<<PosDelimiter[0]<<std::endl;
									StateParser = 2;
									//std::cout<<"寻找帧头$GPCHC完成"<<std::endl;
								}	
								else//帧头错误
								{
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
								}
							}
							break;

						//接收各数据域
						case 2:
							//std::cout<<"开始接受各个数据域"<<std::endl;
							OneFrame[CntByte] = rbuf[i];
							//std::cout<<"接受字符"<<rbuf[i]<<std::endl;
							CntByte++;//指向下一个空位

							if(rbuf[i]==','||rbuf[i]=='*')
							{
								CntDelimiter++;//分隔符计数
								//std::cout<<"分隔符个数："<<CntDelimiter<<std::endl;
								PosDelimiter[CntDelimiter] = CntByte - 1;//记下分隔符位置
								//std::cout<<"PosDelimiter["<<CntDelimiter<<"]"<<PosDelimiter[CntDelimiter]<<std::endl;
								field_len[CntDelimiter-1] = PosDelimiter[CntDelimiter] - PosDelimiter[CntDelimiter-1] - 1;
								//std::cout<<"第"<<CntDelimiter<<"段数据长"<<field_len[CntDelimiter]<<std::endl;
								if(CntDelimiter == 23)//0-23，共24个分隔符，开始数据解析
								{
									//计算出每个字段的长度
									for(int j=0; j<=22; j++)//0-22，23个字段
									{
										field_len[j] = PosDelimiter[j+1] - PosDelimiter[j] - 1;
										//std::cout<<"第"<<j<<"段数据长"<<field_len[j]<<std::endl;
									}
									
									if(field_len[0] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[0]+1], field_len[0]);
										msg_chccgi610_nav.gpsweek = atoi(temp_field);
										
									}

									if(field_len[1] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[1]+1], field_len[1]);
										msg_chccgi610_nav.gpstime = atof(temp_field);
										
									}

									if(field_len[2] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[2]+1], field_len[2]);
										msg_chccgi610_nav.heading = atof(temp_field);
										
									}

									if(field_len[3] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[3]+1], field_len[3]);
										msg_chccgi610_nav.pitch = atof(temp_field);
										
									}

									if(field_len[4] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[4]+1], field_len[4]);
										msg_chccgi610_nav.roll = atof(temp_field);
										
									}
									
									if(field_len[5] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[5]+1], field_len[5]);
										msg_chccgi610_nav.wx = atof(temp_field);
										
									}

									if(field_len[6] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[6]+1], field_len[6]);
										msg_chccgi610_nav.wy = atof(temp_field);
										
									}

									if(field_len[7] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[7]+1], field_len[7]);
										msg_chccgi610_nav.wz = atof(temp_field);
										
									}

									if(field_len[8] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[8]+1], field_len[8]);
										msg_chccgi610_nav.ax = atof(temp_field);
										
									}

									if(field_len[9] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[9]+1], field_len[9]);
										msg_chccgi610_nav.ay = atof(temp_field);
										
									}

									if(field_len[10] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[10]+1], field_len[10]);
										msg_chccgi610_nav.az = atof(temp_field);
										
									}

									if(field_len[11] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[11]+1], field_len[11]);
										msg_chccgi610_nav.lat = atof(temp_field);
										
									}

									if(field_len[12] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field)); 
										strncpy(temp_field, &OneFrame[PosDelimiter[12]+1], field_len[12]);
										msg_chccgi610_nav.lon = atof(temp_field);
										
									}

									if(field_len[13] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[13]+1], field_len[13]);
										msg_chccgi610_nav.alt = atof(temp_field);
										
									}

									if(field_len[14] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[14]+1], field_len[14]);
										msg_chccgi610_nav.ve = atof(temp_field);
										
									}

									if(field_len[15] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[15]+1], field_len[15]);
										msg_chccgi610_nav.vn = atof(temp_field);
										
									}

									if(field_len[16] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[16]+1], field_len[16]);
										msg_chccgi610_nav.vu = atof(temp_field);
										
									}

									if(field_len[17] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[17]+1], field_len[17]);
										msg_chccgi610_nav.v = atof(temp_field);
										
									}

									if(field_len[18] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[18]+1], field_len[18]);
										msg_chccgi610_nav.nsv1 = atoi(temp_field);
										
									}

									if(field_len[19] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[19]+1], field_len[19]);
										msg_chccgi610_nav.nsv2 = atoi(temp_field);
										
									}

									if(field_len[20] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[20]+1], field_len[20]);
										msg_chccgi610_nav.status = atoi(temp_field);
										
									}

									if(field_len[21] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[21]+1], field_len[21]);
										msg_chccgi610_nav.age = atoi(temp_field);
										
									}

									if(field_len[22] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[22]+1], field_len[22]);
										msg_chccgi610_nav.warning = atoi(temp_field);
										
									}

									StateParser = 3;
								}
							}
							break;

							//校验和第一个字符
						case 3:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if(rbuf[i-1]=='*'&&((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F')))//校验和字节应是一个十六进制数
							{
								StateParser = 4;
							}
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							break;

							//校验和第二个字符	
						case 4:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位

							if((rbuf[i]>='0' && rbuf[i]<='9') || (rbuf[i]>='A' && rbuf[i]<='F'))//校验和字节应是一个十六进制数
							{
								//检查校验
								cscomputed = GetXorChecksum((char*)(OneFrame+1), CntByte-4);//计算得到的校验，除去$*hh<CR><LF>共6个字符
								csreceived = 0;//接收到的校验
								strtemp[0] = OneFrame[CntByte-2];
								strtemp[1] = OneFrame[CntByte-1];
								strtemp[2] = '\0';//字符串结束标志
								sscanf(strtemp, "%x", &csreceived);//字符串strtemp转换为16进制数
											
								//检查校验是否正确
								if(cscomputed != csreceived)//校验和不匹配
								{
									memset(OneFrame, 0, sizeof(OneFrame));
									StateParser = 0;
								}
								else//校验和匹配
								{		
									StateParser = 5;
								}	
							}//校验和字节是hex
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							
							break;

							//等待结束标志<CR>=0x0d
						case 5:
							OneFrame[CntByte] = rbuf[i];
							CntByte++;//指向下一个空位
							if(rbuf[i] == '\r')
							{
								StateParser = 6;
							}
							else
							{
								memset(OneFrame, 0, sizeof(OneFrame));
								StateParser = 0;
							}
							break;
						
						//等待结束标志<LF>=0x0a
						case 6:
							OneFrame[CntByte] = rbuf[i];
							if(rbuf[i]=='\n')
							{
								delta_t = msg_chccgi610_nav.gpstime - gpstime_pre;//前后两帧之间的时间差
								gpstime_pre = msg_chccgi610_nav.gpstime;
							}
							
							msg_chccgi610_nav.header.stamp = ros::Time::now();//ros时刻
							chccgi610_imu.header.stamp = ros::Time::now();
							chccgi610_imu.header.frame_id = "imu";
							msg_nav_pub.publish(msg_chccgi610_nav);//发布nav消息
							msg_imu_pub.publish(chccgi610_imu);//发布IMU消息
							//std::cout<<"发布成功"<<std::endl;

							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
							break;

						default:
							memset(OneFrame, 0, sizeof(OneFrame));
							StateParser = 0;
						 	break;
					}//switch(StateParser)
				}//for(int i=0; i<numgetted; i++)
			}//if(numgetted == numinbuf)
		}
		ros::spinOnce();//执行等待队列中所有回调函数
		loop_rate.sleep();
	}//while
    return 0;
}