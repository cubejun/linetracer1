#include "opencv2/opencv.hpp"
#include "dxl.hpp"
#include <iostream>
#include <queue>
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
    videoconvert ! video/x-raw, format=(string)BGR ! appsink";
    VideoCapture source(src, CAP_GSTREAMER);
	// VideoCapture source("8_lt_cw_100rpm_in.mp4");
	if (!source.isOpened()) { cout << "Camera error" << endl; return -1; }


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
	struct timeval start, end1;
	double diff1;
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
	centerq.push(320);
	centeryq.push(45);
	//오른쪽 변수
	
	double gain = 0.3;
	//50rpm아웃은 0.13 인은 0.10
    //100rpm아웃은 0.14, 0.15
    //200rpm은 0.3, 0.32

	signal(SIGINT, ctrlc);              //시그널 핸들러 지정
	if (!mx.open()) { cout << "dxl open error" << endl; return -1; }
	while (true) {
		gettimeofday(&start, NULL);
		if (mx.kbhit())
		{
			char c = mx.getch();
			switch (c)
			{
			case 's': spress = 1; break;
			}
		}
		if (ctrl_c_pressed) break;

		source >> frame;
		if (frame.empty()) { cerr << "frame empty!" << endl; break; }
		ROI = frame(Rect(0, 270, 640, 90));
		cvtColor(ROI, gray, COLOR_BGR2GRAY);
		meangray = gray + (Scalar(100) - mean(gray));
		threshold(meangray, bin, 120, 255, THRESH_BINARY);
		//라인 160
		//레인 불 켬 120, 130
		int cnt = connectedComponentsWithStats(bin, labels, stats, centroids);
		cvtColor(bin, color, COLOR_GRAY2BGR);

		centermin = abs(centerq.back() - centroids.at<double>(1, 0));
		centerymin = abs(centeryq.back() - centroids.at<double>(1, 1));

		for (int i = 1; i < cnt; i++) {
			int* p = stats.ptr<int>(i);
			double* c = centroids.ptr<double>(i);

			if (p[4] > 100) {

				rectangle(color, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);
				circle(color, Point(c[0], c[1]), 2, Scalar(255, 0, 0), 2);

				if ((centermin >= abs(centerq.back() - c[0])) || (centerymin >= abs(centeryq.back() - c[1]))) {
					if ((abs(centerq.back() - c[0]) < 200) && (abs(centeryq.back() - c[1]) < 50)) {
						cntmin = i;
						centermin = abs(centerq.back() - c[0]);
						centerymin = abs(centeryq.back() - c[1]);
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
		centerq.push(centroids.at<double>(cntmin, 0));
		centeryq.push(centroids.at<double>(cntmin, 1));
		if (abs(centerq.back() - centerq.front()) > 200) {
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
			//cx = 0; cy = 0; cw = 0; ch = 0;
		}
		if (abs(centeryq.back() - centeryq.front()) > 60) {
			centeryq.push(centeryq.front());
			centeryq.pop();
			centeryq.push(centeryq.back());
			centeryq.pop();
		}
		if(abs(err - (cameracentroids.x - centerq.back()))>100){
			centerq.push(centerq.front());
			centerq.pop();
			centerq.push(centerq.back());
			centerq.pop();
		}
		centerqbacksave = centerq.back();
		centerysave = centeryq.back();

		circle(color, Point(centerqbacksave, centerysave), 2, Scalar(0, 0, 255), 2);
		rectangle(color, Rect(cx, cy, cw, ch), Scalar(0, 0, 255), 2);
		
		err = cameracentroids.x - centerq.back();
		lvel = 200 - gain * err;
		rvel = -(200 + gain * err);

		centerq.pop();
		centeryq.pop();

		writer1 << frame;
		writer2 << color;
		if (spress)
		{
			if (!mx.setVelocity(lvel, rvel)) { cout << "setVelocity error" << endl; return -1; }
		}
		usleep(20*1000);
		
		gettimeofday(&end1, NULL);
		diff1 = end1.tv_sec + end1.tv_usec / 1000000.0 - start.tv_sec - start.tv_usec / 1000000.0;
		cout << "err : " << err;
		cout << ", lvel : " << lvel;
		cout << ", rvel :" << rvel;
		cout << ", time : " << diff1 << endl;
	}
    mx.close();
	return 0;
}