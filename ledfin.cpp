#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"

using namespace cv;
using namespace std;

int rmin,rmax,bmin,bmax,gmin,gmax,a,b;
void cir(Mat);
double distance(Point );

Mat img=imread("led3.jpg",1);
Point pt_min,pt_max;
int rad_min,rad_max;

int initialise()
{
	rmin=103;
	rmax=255;
	bmin=249;
	bmax=255;
	gmin=127;
    gmax=255;
}

int main()
{
	int i,j;

    a=img.rows;
    b=img.cols;
    Mat led(a,b,CV_8UC1,Scalar(0));
    initialise();
    for(i=0;i<a;i++)
    {
        for(j=0;j<b;j++)
        {
            if((img.at<Vec3b>(i,j)[0]>=bmin)&&(img.at<Vec3b>(i,j)[0]<=bmax)&&(img.at<Vec3b>(i,j)[1]>=gmin)&&(img.at<Vec3b>(i,j)[1]<=gmax)&&(img.at<Vec3b>(i,j)[2]>=rmin)&&(img.at<Vec3b>(i,j)[2]<=rmax))
            led.at<uchar>(i,j)=255;
        }
    }
    imshow("win1",img);
    imshow("My_Window",led);
    cir(led);
    waitKey(0);
	return 0;
}
void cir(Mat led)
{
  Mat led1;

  int dist,min_x=10000,max_x=0,l_led=0,r_led=0;
  vector<vector<Point> >contours;
  vector<Vec4i>hierarchy;

  Mat drawing(a,b,CV_8UC3,Scalar(0,0,0));

  Canny(led,led1,10,30,3,1);
  imshow("canny",led1);

  dilate(led1,led1, Mat(), Point(-1,-1),0,1,1);
  erode(led1,led1, Mat(), Point(-1,-1),0,1,1);

  imshow("ed",led1);

  findContours(led1,contours,hierarchy,CV_RETR_TREE,CV_CHAIN_APPROX_SIMPLE,Point(0, 0));

  /// Approximate contours to polygons + circles
  vector<vector<Point> >contours_poly(contours.size());
  
  vector<Point2f>center(contours.size());
  vector<float>radius(contours.size());
  
  for(int i=0;i<contours.size();i++)
  { 
     approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
     minEnclosingCircle((Mat)contours_poly[i],center[i],radius[i]);
  }

  for( int i = 0; i< contours.size(); i++ )
  {
     Scalar color = Scalar( 0, 0, 255);
    // drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
     circle( drawing, center[i], (int)radius[i], color, 2, 8, 0 );
  }

  for( int i = 0; i< contours.size(); i++ )
  {
     int x1=center[i].x;
     if(x1<min_x)
     {
     	min_x=x1;
     	pt_min=(center[i]);
     	rad_min=(int)radius[i];
     }
     if(x1>max_x)
     {
     	max_x=x1;
     	pt_max=(center[i]);
     	rad_max=(int)radius[i];
     }
  }
  if(pt_min.x==0)
     		l_led=1;
  if(pt_max.x==img.cols)
     		r_led=1;
  cout<<min_x<<endl;
  cout<<max_x<<endl;
  if(l_led==1)
  	cout<<"LEFT LED BLINK"<<endl;
  else if(r_led==1)
  	cout<<"RIGHT LED BLINK"<<endl;
  else
  	cout<<"DON'T BLINK"<<endl;
  l_led=0;
  r_led=0;
  circle( drawing, pt_min, rad_min, Scalar(255,0,0), 2, 8, 0 );
  circle( drawing, pt_max, rad_max, Scalar(0,255,0), 2, 8, 0 );
  imshow("Contours",drawing);
}

double distance(Point a)
{
    return (sqrt(pow(a.x-(img.cols/2),2)+pow((a.y-img.rows),2)));
}