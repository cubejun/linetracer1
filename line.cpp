#include "opencv2/opencv.hpp"//opencv 헤더파일 포함
#include "dxl.hpp"//다이나믹셀 헤더파일 추가
#include <iostream>//iostream 헤더파일 추가
#include <queue>//큐 헤더파일 추가
#include <ctime>
#include <unistd.h>
#include <signal.h>
using namespace cv;
using namespace std;
bool ctrl_c_pressed;
void ctrlc(int)
{
	ctrl_c_pressed = true;
}
int main()
{
	string src = "nvarguscamerasrc sensor-id=0 ! \
    video/x-raw(memory:NVMM), width=(int)640, height=(int)360, \
    format=(string)NV12 ! nvvidconv flip-method=0 ! video/x-raw, \
    width=(int)640, height=(int)360, format=(string)BGRx ! \
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";//카메라로부터 영상 입력 받음
    VideoCapture source(src, CAP_GSTREAMER);
	// VideoCapture source("8_lt_cw_100rpm_in.mp4");
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }//카메라에러처리


	string dst1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.156 port=8001 sync=false";
	VideoWriter writer1(dst1, 0, (double)30, Size(640, 360), true);
	if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	string dst2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! \
    nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! \
    h264parse ! rtph264pay pt=96 ! \
    udpsink host=203.234.58.156 port=8002 sync=false";
	VideoWriter writer2(dst2, 0, (double)30, Size(640, 90), true);
	if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	Dxl mx;
	struct timeval start, end1;//시간 저장하는 변수
	double diff1;//코드 동작 시간 저장하는 변수
	int lvel = 0, rvel = 0, err;

	Point cameracentroids(320, 45);//영상의 중심좌표

	Mat frame, ROI, gray, bin, color, meangray;//매트변수

	Mat labels, stats, centroids;//레이블링 변수

	bool spress = 0;

	//왼쪽
	int cntmin = 0;
	// , nearx = 0, neary = 0;
	int cx = 0, cy = 0, cw = 0, ch = 0;
	double centermin = 0, centerqbacksave, centerysave = 45, centerymin = 0;
	queue<double> centerq, centeryq;
	centerq.push(320);//영상의 중심 x좌표 큐에 푸쉬
	centeryq.push(45);//영상의 중심 y좌표 큐에 푸쉬

	double gain = 0.3;
	//50rpm아웃은 0.13 인은 0.10
    //100rpm아웃은 0.14, 0.15
    //200rpm은 0.3, 0.32

	signal(SIGINT, ctrlc);              //시그널 핸들러 지정
	if (!mx.open()) { cout << "dxl open error" << endl; return -1; }//다이나믹셀 오픈 에러처리
	while (true) {//무한반복문 실행
		gettimeofday(&start, NULL);//반복문 안 코드 실행 시간 측정
		if (mx.kbhit())//키보드 입력 검사
		{
			char c = mx.getch();//키보드에 입력된 문자 저장
			switch (c)//c에 입력된 단어 케이스 별로 처리
			{
				case 's': spress = 1; break;//s가 입력되면 모터 동작 시작
			}
		}
		if (ctrl_c_pressed) break;//ctrl+c  입력되면 반복문 탈출

		source >> frame;//카메라 영상을 Mat 객체 frame 에 저장
		if (frame.empty()) { cerr << "frame empty!" << endl; break; }//프레임 에러 처리
		ROI = frame(Rect(0, 270, 640, 90));//관심영역지정
		cvtColor(ROI, gray, COLOR_BGR2GRAY);//컬러영상 흑백영상 변환
		meangray = gray + (Scalar(100) - mean(gray));//영상 평균값 보정
		threshold(meangray, bin, 120, 255, THRESH_BINARY);//영상 이진화
		//라인 160
		//레인 불 켬 120, 130
		int cnt = connectedComponentsWithStats(bin, labels, stats, centroids);//이진화한 관심영역 레이블링
		cvtColor(bin, color, COLOR_GRAY2BGR);//이진화한 영상 컬러 영상으로 변환

		centermin = abs(centerq.back() - centroids.at<double>(1, 0));//레이블링되어 검출된 첫번째 객체의 x 좌표와 이전에 검출된 x좌표의 차이 계산
		centerymin = abs(centeryq.back() - centroids.at<double>(1, 1));//레이블링되어 검출된 첫번째 객체의 y 좌표와 이전에 검출된 y좌표의 차이 계산

		for (int i = 1; i < cnt; i++) {//레이블링 해서 검출된 객체의 개수만큼 반복
			int* p = stats.ptr<int>(i);//스텟의 내용을 포인터 변수에 저장
			double* c = centroids.ptr<double>(i);//무게중심의 내용을 포인터 변수에 저장

			if (p[4] > 100) {//객체의 크기가 100 이상일때만 동작

				rectangle(color, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);//검출된 객체를 파란색으로 사각형 그림
				circle(color, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2);//검출된 객체의 무게중심에 파란색 원 그림

				if ((centermin >= abs(centerq.back() - c[0])) || (centerymin >= abs(centeryq.back() - c[1]))) {//검출된 객체의 좌표들 중 이전의 라인 무게중심좌표와 가장 가까운 좌표를 검출
					if ((abs(centerq.back() - c[0]) < 200) && (abs(centeryq.back() - c[1]) < 50)) {//이전 라인 좌표와 현재 검출된 객체의 무게중심 좌표의 차이가 특정 값 이하일때만 동작
						cntmin = i;//이전 라인 무게중심과 객체의 무게중심차이가 차이가 작을때의 인덱스를 cmtmin에 저장
						centermin = abs(centerq.back() - c[0]);//최소값 알고리즘
						centerymin = abs(centeryq.back() - c[1]);//최소값 알고리즘
						// nearx = centroids.at<double>(cntmin, 0);
						// neary = centroids.at<double>(cntmin, 1);
						cx = stats.at<int>(cntmin, 0);
						cy = stats.at<int>(cntmin, 1);
						cw = stats.at<int>(cntmin, 2);
						ch = stats.at<int>(cntmin, 3);
					}
				}
			}
		}
		//1
		centerq.push(centroids.at<double>(cntmin, 0));//이전 라인과 무게중심 차이가 가장 작은 객체의 x좌표를 큐에 푸쉬
		centeryq.push(centroids.at<double>(cntmin, 1));//이전 라인과 무게중심 차이가 가장 작은 객체의 y좌표를 큐에 푸쉬
		if (abs(centerq.back() - centerq.front()) > 200) {//이전 라인과 현재 라인의 x좌표 차이가 200이상이 되면 
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
			//이전 라인 무게중심을 현재 라인 무게중심으로 저장
		}
		if (abs(centeryq.back() - centeryq.front()) > 60) {//이전 라인과 현재 라인의  y좌표 차이가 60이상이 되면 
			centeryq.push(centeryq.front());
			centeryq.pop();
			centeryq.push(centeryq.back());
			centeryq.pop();
			//이전 라인 무게중심을 현재 라인 무게중심으로 저장
		}
		if(abs(err - (cameracentroids.x - centerq.back()))>100){//이전 에러와 현재 에러의 차이가 100 이상이면 
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
			//이전 라인 무게중심을 현재 라인 무게중심으로 저장
		}
		centerqbacksave = centerq.back();//검출된 현재 라인 무게중심값 변수에 저장
		centerysave = centeryq.back();//검출된 현재 라인 무게중심값 변수에 저장

		circle(color, Point(centerqbacksave, centerysave), 2, Scalar(0, 0, 255), 2);//검출된 라인의 무게중심 빨간색 원으로 그림
		rectangle(color, Rect(cx, cy, cw, ch), Scalar(0, 0, 255), 2);//검출된 라인에 빨간색 사각형 그림
		
		err = cameracentroids.x - centerq.back();//카메라의 x좌표 중심값과 현재 검출된 무게중심의 x좌표를 빼서 에러값 저장
		lvel = 200 - gain * err;//왼쪽 바퀴 속도
		rvel = -(200 + gain * err);//오른쪽 바퀴 속도

		centerq.pop();//이전 라인 x좌표값 제거
		centeryq.pop();//이전 라인 y좌표값 제거

		writer1 << frame;//영상 출력
		writer2 << color;//영상처리한 영상 출력
		if (spress)//s버튼이 입력되면 모터 구동
		{
			if (!mx.setVelocity(lvel, rvel)) { cout << "setVelocity error" << endl; return -1; }
		}
		usleep(20*1000);//20ms 쉼
		
		gettimeofday(&end1, NULL);//코드 동작 시간 측정
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;//코드 동작 시간 변수에 저장
		cout << "err : " << err;//에러값 출력
		cout << ", lvel : " << lvel;//왼쪽 바퀴 속도 출력
		cout << ", rvel :" << rvel;//오른쪽 바퀴 속도 출력
		cout << ", time : " << diff1 << endl;//코드 동작 시간 출력
	}
    mx.close();//다이나믹셀 종료
	return 0;//0 반환
}