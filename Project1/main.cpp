#include <opencv2/opencv.hpp>
#include "tld_utils.h"
#include <iostream>
#include <sstream>
#include "TLD.h"
#include <cstdio>
#include <time.h>
#include "Ardrone.h"
using namespace cv; //Use open cv 
using namespace std;
//Global variables    // Can't change
Rect box;
bool drawing_box = false;
bool gotBB = false;   
bool tl = true;
//FRAME  //work every frame 
bool Global_Pro_LT; 
cv::Rect init_Fr, cur_Fr;
const char winName[10] = "DXY";
//bounding box mouse callback
void mouseHandler(int event, int x, int y, int flags, void *param);
cv::Mat getFlow(cv::Mat img, int threshold);
// *** *** ***
SPEED speed;
void Control(ARDrone &ardrone, char key); //控制飞机
int L_SL = 1200;
int R_SL = 3000;
const int TTX = 640;
const int TTY = 360;
int XXL = 80, XXR = 570;
int YYL = 60, YYR = 290;
const int POP = 12;
int Make_dec(ARDrone &ardrone);
void PRINT_info(ARDrone &ardrone,cv::Mat &frame);
void SetRange();
int main(int argc, char * argv[]){
	FileStorage fs;
	//Read options
	fs.open("parameters.yml", FileStorage::READ);
	ARDrone ardrone;
	//Init camera
	if (!ardrone.Init()){  cout << "Ardrone Init failed !" << endl; return 0; }
	//Register mouse callback to draw the bounding box
	cvNamedWindow(winName, CV_WINDOW_AUTOSIZE);
	cvSetMouseCallback(winName, mouseHandler, NULL);
	//TLD framework
	TLD tld;
	//Read parameters file
	tld.read(fs.getFirstTopLevelNode());

	Mat frame, last_gray,PPP; //
	VideoWriter outputVideo("output.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(640, 360));

	///Initialization
    
GETBOUNDINGBOX:
	while(!gotBB){
        frame = ardrone.getFrame();
		cvtColor(frame, last_gray, CV_RGB2GRAY);
		//last_gray = frame;
		drawBox(frame,box); //输出至控制端
		init_Fr = box;
		imshow(winName, frame);
		char key = cvWaitKey(33); //为什么是33
		if (key == 'q')
			return 0;
	}
	//box 选定的追踪方框
	if (min(box.width,box.height)<(int)fs.getFirstTopLevelNode()["min_win"]){
		cout << "Bounding box too small, try again." << endl;
		gotBB = false;
		goto GETBOUNDINGBOX;//重新再去读取方框
	}
	//Remove callback
	cvSetMouseCallback(winName, NULL, NULL);//点 拖 弹 
	//printf("Initial Bounding Box = x:%d y:%d h:%d w:%d\n",box.x,box.y,box.width,box.height);
	//Output file
	FILE *bb_file = fopen("bounding_boxes.txt","w");//这是什么 读入一个跟踪阵列
	//TLD initialization
	tld.init(last_gray,box,bb_file);//TLD算法入门

	SetRange(); //设置区间

	///Run-time
	Mat current_gray; //当前灰度
	BoundingBox pbox;  
	vector<Point2f> pts1; //可变长的数组
	vector<Point2f> pts2;
	bool status=true; 
	printf("Current Box:(%d,%d)\n", init_Fr.height, init_Fr.width);
	bool Lock = false;
	SPEED pre_speed;
	for (int Lost = 0, count = 0; true; count++){ //控制进程
        frame = ardrone.getFrame();
		outputVideo << frame; //把每帧输出到桌面上
		cvtColor(frame, current_gray, CV_RGB2GRAY); //把它彩色的图变成灰色的图
		//Process Frame
		tld.processFrame(last_gray,current_gray,pts1,pts2,pbox,status,tl,bb_file); //通过上一帧和下一帧的灰度 
		//Draw Points
		if (status){
			drawBox(frame,pbox); //Pbox 是啥
			cur_Fr = pbox; 
			Lost = 0;
		}else{
			Lost++;
		}
		PRINT_info(ardrone, frame);
		//Display
		imshow(winName, frame);
		// 输出
		
		//swap points and images
		swap(last_gray,current_gray);
		pts1.clear();
		pts2.clear();
		
		char key = cvWaitKey(33);//33 ！ 
		if (key == 27){ //27  空格
			break;
		}else if (key == 'b'){
			Lock = !Lock;
			cout << "Lock:" << (Lock ? "true" : "false") << endl; //是否经过锁定
		}
		speed.ZERO();//为什么是ZERO
		bool Coot = false;
		if ((key >= 'a' && key <= 'z') || key == ' '){ 
			Control(ardrone, key);  //启动控制
			Coot = true;
		}

		if (!Lock && !Coot){
			speed.Update( Make_dec( ardrone ), Global_Pro_LT );//这是控制台的控制
		}
			
		if (!Lock && Lost > 0){ 
			speed.ZERO(); //这个到底是在哪定义的
			if(Lost<3)cout << "LOST LOST LOST LOST LOST !!!" << endl; //为什么是Lost<3?
		}

		if (speed.isEmpty() && pre_speed == speed)continue; //这应该是重新
		
		SPEED cur = speed.GotSlow(); //为什么把当前的变小?
		if (count % POP >= POP/5 ){ 
			cur = speed; //这个...不知道为什么除以5
		}else{
			cur = speed; cur.ZERO(); 
		}
		cur.Lock(4);
		ardrone.move3D(cur);
		pre_speed = speed;

	}
    ardrone.Close();
	fclose(bb_file); 
	return 0;
}
const int maxThsod = 80;
SPEED GotSlow(SPEED now){
	int CX = cur_Fr.x + cur_Fr.width / 2;
	int CY = cur_Fr.y + cur_Fr.height / 2;
	double Ky = min(abs(CX - XXL), abs(CX - XXR))*1.0 / maxThsod;
	double Kz = min(abs(CY - YYL), abs(CY - YYR))*1.0 / maxThsod;
	Ky = sqrt(Ky + 0.4);
	Kz = sqrt(Kz + 0.4);
	if (Ky < 0.5) Ky = 0.5;
	if (Kz < 0.5) Kz = 0.5;
	double Kx = 1.0;
	//int IniS = init_Fr.height*init_Fr.width;
	int CurS = cur_Fr.height*cur_Fr.width;
	double PP = 0;
	if (CurS > R_SL){
		//Back
		PP = 1.0*(CurS - R_SL) / maxThsod;
	}
	else if (CurS < L_SL){
		//Front
		PP = 1.0*(L_SL - CurS) / maxThsod;
	}
	if (PP != 0)PP += 0.2;
	Kx = sqrt(PP);
	SPEED ans;
	ans.Update(now.vx*Kx, now.vy*Ky, now.vz*Kz);//这里更新速度
	return ans;
}

void SetRange(){ //这个应该是设置区间的吧
	int s = init_Fr.height * init_Fr.width;
	if (s < 1000){
		L_SL = (int)(0.6*s);
		R_SL = (int)(s*1.7);
		XXL = 170;
	}else if(s<1400){
		L_SL = (int)(0.7*s);
		R_SL = (int)(s*1.7);
		XXL = 170;
	}else if (s < 2000){
		L_SL = (int)(0.8*s);
		R_SL = (int)(s*1.5);
		XXL = 170;
		YYL = 80;
	}else if (s < 2500){
		L_SL = (int)(0.8*s);
		R_SL = (int)(s*1.4);
		XXL = 200;
		YYL = 80;
	}else{
		L_SL = (int)(0.75*s);
		R_SL = (int)(s*1.23);
		XXL = 200;
		YYL = 80;
	}
	XXL = 220; YYL = 110;
	XXR = TTX - XXL;
	YYR = TTY-YYL-20;
}
void Control(ARDrone &ardrone, char key){
	// Take off / Landing
	speed.ZERO();  //悬浮
	int bits = 0; //没看懂这是啥
	if (key == ' ') {
		if (ardrone.onGround()) ardrone.takeoff();  //这是对于起飞的控制
		else					ardrone.landing();
	}else if (key == 'i')   { //Front //向前飞 然而bits还是不懂
		speed.Update(1 << 0,false);
		//speed.Up_FB(true);
		bits += 1 << 0;
	}else if (key == 'k'){   //Back
		//speed.Update(1 << 1);
		//speed.Up_FB(false);
		bits += 1 << 1;
	}else if (key == 'u'){   //Roll L
		//speed.Roll(2.0);
		speed.Roll(1.5);
	}else if (key == 'o'){    //Roll R
		//speed.Roll(-2.0);
		speed.Roll(-1.5);
	}else if (key == 'j'){
		//speed.Update(1 << 2);
		//speed.Up_LR(true);
		bits += 1 << 2;
	}else if (key == 'l'){
		//speed.Update(1 << 3);
		//speed.Up_LR(false);
		bits += 1 << 3;
	}else if (key == 'q'){
		//speed.Update(1 << 4);
		//speed.vz = speed.UD;
		bits += 1 << 4;
	}else if (key == 'a'){
		//speed.Update(1 << 5);
		//speed.vz = -speed.UD;
		bits += 1 << 5;
	}else if (key == 'z'){
		ardrone.setFlatTrim();
		cout << "FlatTrim" << endl;
	}else if (key == 'e'){
		ardrone.emergency();
		cout << "Emergency !!!" << endl;
	}else if (key == 'm'){
		ardrone.setCalibration();
		cout << "Calibration" << endl;
	}else if (key == 'r'){
		init_Fr = cur_Fr;
		SetRange();
    }else if(key=='g'){
        // Change Shift to Turn or reverse.
        Global_Pro_LT = !Global_Pro_LT;
		cout << (Global_Pro_LT ? "Change to Turn" : "C to LR");
    }
	//speed.Print();
	//ardrone.move3D(speed);
	//return 0;
	if (speed.vr == 0)speed.Update(bits,false); // L-R
}
int Make_dec(ARDrone &ardrone){ //这是判断人走的方向 然后输出到控制台的
	//int IniS = init_Fr.height*init_Fr.width;
	int CurS = cur_Fr.height*cur_Fr.width; //当前的Screen 的高度和宽度
	int bits = 0;//Bits 是控制的方式 不同的位数代表不同指令
	if (CurS > R_SL){
		//Back
		//Control(ardrone, 'k');
		bits |= 1 << 1;
		//cout << "Go Back:  " << CurS << " -> " << R_SL << endl;
	}else if (CurS < L_SL ){
		//Front
		//Control(ardrone, 'i');
		bits |= 1 << 0;
		//cout << "Front:  " << CurS << " -> " << L_SL << endl;
	}
	int mid = (cur_Fr.x + cur_Fr.width / 2);
	if (mid < XXL){
		//Control(ardrone, 'j');
		bits |= 1 << 2;
		//cout << "Left: " << cur_Fr.x << "->" << XXL << endl;
	}else if (mid > XXR){
		//Control(ardrone, 'l');
		bits |= 1 << 3;
		//cout << "Right: " << cur_Fr.x << "->" << TTX - XXL - cur_Fr.width / 3 << endl;
	}
	mid = (cur_Fr.y + cur_Fr.height / 2);
	if (mid < YYL){
		//Control(ardrone, 'q');
		bits |= 1 << 4;
		//cout << "UP" << endl;
	}else if (mid > YYR){//|| cur_Fr.y+cur_Fr.height>TTY*0.95){
		//Control(ardrone, 'a');
		bits |= 1 << 5;
		//cout << "Down" << endl;
	}
	return bits;
}
void PRINT_info(ARDrone &ardrone,cv::Mat &frame){
	char showMsg[40] = { 0 };
	//字体初始化
    CvFont cvfont;
	cvInitFont(&cvfont, CV_FONT_HERSHEY_PLAIN | CV_FONT_ITALIC, 1, 1, 0, 1);
	double VX, VY, VZ;
	sprintf(showMsg, "Battery:%d%%", ardrone.getBatteryPercentage());
    cv::putText(frame, showMsg, cv::Point(0,10),CV_FONT_HERSHEY_PLAIN , 0.9, CV_RGB(0, 0, 0));
	//cvPutText(src, showMsg, cvPoint(0, 10), &cvfont, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Altitude:%.3lf", ardrone.getAltitude());
	//cvPutText(src, showMsg, cvPoint(0, 25), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 25), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "WIFI:%d", ardrone.getWIFI_S());
	//cvPutText(src, showMsg, cvPoint(0, 40), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 40), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Speed:%.3lf", ardrone.getVelocity(&VX, &VY, &VZ));
	//cvPutText(src, showMsg, cvPoint(0, 55), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 55), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "X:%.2lf , Y:%.2lf, Z:%.2lf", VX, VY, VZ);
	//cvPutText(src, showMsg, cvPoint(0, 70), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 70), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Pitch:%.2lf ", ardrone.getPitch());;
	//cvPutText(src, showMsg, cvPoint(0, 85), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 85), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Yaw:%.2lf ", ardrone.getYaw());
	//cvPutText(src, showMsg, cvPoint(0, 100), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 100), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Rol:%.2lf ", ardrone.getRoll());
	//cvPutText(src, showMsg, cvPoint(0, 115), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 115), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "POS:(%d,%d)", cur_Fr.x,cur_Fr.y);
	//cvPutText(src, showMsg, cvPoint(0, 130), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 130), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "Size:(%d,%d)", cur_Fr.height, cur_Fr.width);
	//cvPutText(src, showMsg, cvPoint(0, 145), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 145), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	sprintf(showMsg, "VVV:(%.2lf,%.2lf,%.2lf)", speed.vx, speed.vy,speed.vz);
	//cvPutText(src, showMsg, cvPoint(0, 160), &cvfont, CV_RGB(0, 0, 0));
	cv::putText(frame, showMsg, cv::Point(0, 160), CV_FONT_HERSHEY_PLAIN, 0.9, CV_RGB(0, 0, 0));
	// 画矩形
    cv::line(frame, Point(XXL, YYL), Point(XXR, YYL), CV_RGB(0, 0, 0), 1, CV_AA);
    cv::line(frame, Point(XXL, YYR), Point(XXR, YYR), CV_RGB(0, 0, 0), 1, CV_AA);
    cv::circle(frame, Point(cur_Fr.x + cur_Fr.width / 2, cur_Fr.y + cur_Fr.height / 2), 2, CV_RGB(0, 255, 255), -1);
}
void mouseHandler(int event, int x, int y, int flags, void *param){
	switch (event){
	case CV_EVENT_MOUSEMOVE:
		if (drawing_box){
			box.width = x - box.x;
			box.height = y - box.y;
		}
		break;
	case CV_EVENT_LBUTTONDOWN:
		drawing_box = true;
		box = Rect(x, y, 0, 0);
		break;
	case CV_EVENT_LBUTTONUP:
		drawing_box = false;
		if (box.width < 0){
			box.x += box.width;
			box.width *= -1;
		}
		if (box.height < 0){
			box.y += box.height;
			box.height *= -1;
		}
		gotBB = true;
		break;
	}
}
