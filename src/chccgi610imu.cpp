#include<ros/ros.h>
#include<iostream>
#include<serial/serial.h>
#include<sensor_msgs/Imu.h>


//$GPCHC,GPSWeek,GPSTime,Heading,Pitch,Roll,gyro x,gyro y,gyro z,acc x,acc y,acc z,Lattitude,Longitude,Altitude,Ve,Vn,Vu,Baseline,NSV1,NSV2,Status,Age,Warming,Cs<CR><LF>
//$GPCHC,wwww,ssssss.ss,hhh.hh,-pp.pp,-rrr.rr,-ggg.gg,-ggg.gg,-ggg.gg,-a.aaaa,-a.aaaa,-a.aaaa,-ll.lllllll,-ll.lllllll,-aaaa.aa,-eee.eee,-nnn.nnn,-uuu.uuu,-uuu.uuu,nn,nn,ss,aa,ww,*hh<CR><LF>
//$GPCHC,2063,11273.94,179.78,-0.21,-1.75,-0.15,-0.01,0.02,0.0304,-0.0036,0.9980,31.0246688,121.4355940,17.46,0.002,0.035,-0.012,0.036,22,20,90,1,
//$GPCHC每帧的数据长度最大为183位字符，包括头尾字符


//全局变量
serial::Serial ser;//声明串口对象 
int StateParser = 0;//解析处理状态机状态
int CntByte = 0;//用于记录OneFrame中的实际数据长度
int PosDelimiter[24] = {0};//用于记录分隔符位置
int field_len[23];//字符串长度
int CntDelimiter = 0;//分隔符计数
unsigned char rbuf[500];//接收缓冲区，要足够大，需要通过测试得出
char OneFrame[250];//存放一帧数据，长度大于183即可，这里取250
double gpstime_pre;//上一个gps时间
double delta_t;
char str[3];
unsigned int tmpint = 0;
int cscomputed;//计算得到的校验，除去$*hh<CR><LF>共6个字符
int csreceived;//接收到的校验
char strtemp[3];
char temp_field[30] = {0};
sensor_msgs::Imu msg_chccgi610_imu;

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
    ros::init(argc, argv, "chccgi610imu");
    ros::NodeHandle nh;
	std::string port_name;
	ros::param::get("~port_name", port_name);
	msg_imu_pub=nh.advertise<sensor_msgs::Imu>("/chccgi610_imu",1000,true);

	try 
    { 
    	//设置串口属性，并打开串口 
        ser.setPort(port_name); 
        ser.setBaudrate(230400); 
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
		int numinbuf;
		int numgetted;
		
		try
		{
			numinbuf = ser.available();//available()返回从串口缓冲区读回的字符数
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
								//printf("%s\n",OneFrame);
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
									for(int j=5; j<=10; j++)//0-22，23个字段
									{
										field_len[j] = PosDelimiter[j+1] - PosDelimiter[j] - 1;
										//std::cout<<"第"<<j<<"段数据长"<<field_len[j]<<std::endl;
									}
									
									if(field_len[5] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[5]+1], field_len[5]);
										msg_chccgi610_imu.angular_velocity.x = atof(temp_field);
										
									}

									if(field_len[6] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[6]+1], field_len[6]);
										msg_chccgi610_imu.angular_velocity.y = atof(temp_field);
										
									}

									if(field_len[7] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[7]+1], field_len[7]);
										msg_chccgi610_imu.angular_velocity.z = atof(temp_field);
										
									}

									if(field_len[8] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[8]+1], field_len[8]);
										msg_chccgi610_imu.linear_acceleration.x = atof(temp_field);
										
									}

									if(field_len[9] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[9]+1], field_len[9]);
										msg_chccgi610_imu.linear_acceleration.y = atof(temp_field);
										
									}

									if(field_len[10] > 0)
									{
										memset(temp_field, 0, sizeof(temp_field));
										strncpy(temp_field, &OneFrame[PosDelimiter[10]+1], field_len[10]);
										msg_chccgi610_imu.linear_acceleration.z = atof(temp_field);
										
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
							
							msg_chccgi610_imu.header.stamp = ros::Time::now();//ros时刻
							msg_chccgi610_imu.header.frame_id="imu";
							msg_imu_pub.publish(msg_chccgi610_imu);//发布imu消息
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