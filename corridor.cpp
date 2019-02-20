#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <bits/stdc++.h>
#include <iostream>
#include <fcntl.h>
#include <termios.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core/core.hpp"
#include <errno.h> /* ERROR Number Definitions */

using namespace cv;
using namespace std;

int rmin,rmax,bmin,bmax,gmin,gmax,dil,ero;
int fd;
Mat img;
const char *prev;
RNG rng(12345);
int blink;
struct timespec start, finish;
int l;


void initialise()
{
   rmin=76;
   rmax=255;
   bmin=0;
   bmax=255;
   gmin=0;
   gmax=255;
   dil=8;
   ero=2; 
}

void init_var()
{
  blink=0;
  finish.tv_sec =0;
  start.tv_sec=0;
}

double ycoord(int ,double ,Point );
double xcoord(int ,double ,Point );
double angle_between(Point , Point , Point );
char turn(double ,int ,int );
//void cir(Mat);
void settings(const char *abc)
{
      fd = open(abc,O_RDWR | O_NOCTTY); /* ttyUSB0 is the FT232 based USB2SERIAL Converter   */
      usleep(3500000);
                                    /* O_RDWR Read/Write access to serial port           */
                                    /* O_NOCTTY - No terminal will control the process   */
                                    /* O_NDELAY -Non Blocking Mode,Does not care about-  */
                                    /* -the status of DCD line,Open() returns immediatly */                                        
                                    
            if(fd == -1)                        /* Error Checking */
                   printf("\n  Error! in Opening ttyUSB0  ");
            else
                   printf("\n  ttyUSB0 Opened Successfully ");
       struct termios toptions;         /* get current serial port settings */
       tcgetattr(fd, &toptions);        /* set 9600 baud both ways */
       cfsetispeed(&toptions, B9600);
       cfsetospeed(&toptions, B9600);   /* 8 bits, no parity, no stop bits */
       toptions.c_cflag &= ~PARENB;
       toptions.c_cflag &= ~CSTOPB;
       toptions.c_cflag &= ~CSIZE;
       toptions.c_cflag |= CS8;         /* Canonical mode */
       toptions.c_lflag |= ICANON;       /* commit the serial port settings */
       tcsetattr(fd, TCSANOW, &toptions);
}
void sendCommand(const char *abc)
{
   write(fd, abc, 1);
}

int main(int argc, const char **argv)
{
   int left=0,right=0;
   settings(argv[1]);
   init_var();
   VideoCapture cap(1);
   while(1)
   {
   cout<<blink<<endl;
   cap >> img;
   Mat img2,color_dst,color;
   Mat im(img.rows,img.cols,CV_8UC1,Scalar(0));   

   int i,j,x,y,a=0,col=0,b=0,c=0,d=0,e=0,flag=0,tur=-10;
   double angle;

   Point p1,p2,lmin,lmax,rtmin,rtmax,midmin,midmax;

   lmin=Point(img.cols+1,img.rows+1);
   lmax=Point(-1,-1);
   rtmin=Point(img.cols+1,img.rows+1);
   rtmax=Point(-1,-1);
   midmin=Point(img.cols+1,img.rows+1);
   midmax=Point(-1,-1);

   namedWindow("Original",WINDOW_NORMAL);
   imshow("Original",img);
   resizeWindow("Original",600,600);
   
   initialise();

   for(i=0;i<img.rows;i++)
   {
        for(j=0;j<img.cols;j++)
        {
            if((img.at<Vec3b>(i,j)[0]>=bmin)&&(img.at<Vec3b>(i,j)[0]<=bmax)&&(img.at<Vec3b>(i,j)[1]>=gmin)&&(img.at<Vec3b>(i,j)[1]<=gmax)&&(img.at<Vec3b>(i,j)[2]>=rmin)&&(img.at<Vec3b>(i,j)[2]<=rmax))
            im.at<uchar>(i,j)=255;
        }
   }
   
   dilate(im, im, Mat(), Point(-1,-1),dil,1,1);
   erode(im, im, Mat(), Point(-1,-1),ero,1,1);

   namedWindow("Processed",WINDOW_NORMAL);
   imshow("Processed",im);
   resizeWindow("Processed",600,600);

   Canny(im,img2,10,30,3,1);

   vector<vector<Point> >contours;
   int area[contours.size()];
   int length[contours.size()];
  vector<Vec4i>hierarchy;

   findContours(img2,contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  int minlength=1000000000;
  int min;
  Mat img1 = Mat::zeros( img2.size(), CV_8UC1 );
  Mat led = Mat::zeros( img2.size(), CV_8UC1 );
  cout<<"S"<<endl;
  int rem=0;
  for( int i = 0; (i< contours.size()) && (contours.size()<10); i++ )
     {
      Scalar color = Scalar( 255,255,255 );
       area[i]=contourArea(contours[i]);
       length[i]=arcLength(contours[i],false);
       //cout<<area[i]<<" "<<length[i]<<endl;

       if((minlength>length[i])&&length[i]>30)
       {
           minlength=length[i];
           min=i;
       }
       if(length[i]<=50)
       {
        rem++;
       }
       else
       {
           drawContours( img1,contours, i, color, 2, 8, hierarchy, 0, Point() );
       } 
     }
     cout<<min<<endl;
     if(contours.size()>1&&((contours.size()-rem)>1))
     {
     drawContours( img1,contours, min, Scalar(0,0,0), 2, 8, hierarchy, 0, Point() );
     drawContours( led,contours, min, Scalar(255,255,255), 2, 8, hierarchy, 0, Point() );
     }
     cout<<"E"<<endl;

 imshow("New",img1);
 imshow("New1",led);
 
 cout<<blink<<endl;
 

 resizeWindow("New",600,600);
 cvtColor(img1,color_dst,CV_GRAY2BGR);
   
 vector<Vec4i> lin;
 HoughLinesP( img1, lin, 1, CV_PI/180,50,50,30);
 int lines[lin.size()][4];
  double slope[lin.size()];

    for( int i = 0; i < lin.size(); i++ )
    {
        p1=Point(lin[i][0], lin[i][1]);
        p2=Point(lin[i][2], lin[i][3]);
        float y=(lin[i][2]-lin[i][0]);
        if(y!=0)
        {

            double sl = (p2.y - p1.y) / (double)(p2.x - p1.x);
            if(abs(sl)<=6.0)
            {
                //line( color_dst, p1 , p2, Scalar(0,0,255), 3, 8 );
                while(col<4)
                {
                    lines[a][col]=lin[i][col];
                    col++;   
                }
                if(abs(sl)>=0.20)
                    {
                      if(sl<0)
                      {
                         if(c==0)
                          c++;
                         if(lines[a][0]<lines[a][2] && lines[a][0]<lmin.x)
                         {
                          lmin.x=lines[a][0];
                          lmin.y=lines[a][1];
                         }
                         else if(lines[a][2]<lines[a][0] && lines[a][2]<lmin.x)
                         {
                          lmin.x=lines[a][2];
                          lmin.y=lines[a][3];
                         }
                         if(lines[a][0]>lines[a][2] && lines[a][0]>lmax.x)
                         {
                          lmax.x=lines[a][0];
                          lmax.y=lines[a][1];
                         }
                         else if(lines[a][2]>lines[a][0] && lines[a][2]>lmax.x)
                         {
                          lmax.x=lines[a][2];
                          lmax.y=lines[a][3];
                         }
                      }
                      else
                      {
                         if(d==0)
                          d++;
                         if(lines[a][0]<lines[a][2] && lines[a][0]<rtmin.x)
                         {
                          rtmin.x=lines[a][0];
                          rtmin.y=lines[a][1];
                         }
                         else if(lines[a][2]<lines[a][0] && lines[a][2]<rtmin.x)
                         {
                          rtmin.x=lines[a][2];
                          rtmin.y=lines[a][3];
                         }
                         if(lines[a][0]>lines[a][2] && lines[a][0]>rtmax.x)
                         {
                          rtmax.x=lines[a][0];
                          rtmax.y=lines[a][1];
                         }
                         else if(lines[a][2]>lines[a][0] && lines[a][2]>rtmax.x)
                         {
                          rtmax.x=lines[a][2];
                          rtmax.y=lines[a][3];
                         }
                      }
                    }
                  else if(abs(sl)<0.1)
                  {
                    if(e==0)
                      e++;
                    if(lines[a][0]<lines[a][2] && lines[a][0]<midmin.x)
                         {
                          midmin.x=lines[a][0];
                          midmin.y=lines[a][1];
                         }
                         else if(lines[a][2]<lines[a][0] && lines[a][2]<midmin.x)
                         {
                          midmin.x=lines[a][2];
                          midmin.y=lines[a][3];
                         }
                         if(lines[a][0]>lines[a][2] && lines[a][0]>midmax.x)
                         {
                          midmax.x=lines[a][0];
                          midmax.y=lines[a][1];
                         }
                         else if(lines[a][2]>lines[a][0] && lines[a][2]>midmax.x)
                         {
                          midmax.x=lines[a][2];
                          midmax.y=lines[a][3];
                         }
                  }    
                slope[a]=sl;
                a++;
                col=0;
            }
        }
    }
    cout<<lmin<<";"<<endl;
    cout<<lmax<<";"<<endl;
    cout<<rtmin<<";"<<endl;
    cout<<rtmax<<";"<<endl;
    cout<<midmin<<";"<<endl;
    cout<<midmax<<";"<<endl;

    b=c+d+e;
    cout<<b<<endl;
    line( color_dst, Point(0,img.rows/2), Point(img.cols,img.rows/2), Scalar(0,255,0), 3, 8 );
    line( color_dst, Point(img.cols/2,0), Point(img.cols/2,img.rows), Scalar(0,255,0), 3, 8 );

    if(b==3)
    {
        x=0;
        int x1,x2;
        x1=xcoord(img.rows/2,double(lmin.y-lmax.y)/(lmin.x-lmax.x),lmin);
        x2=xcoord(img.rows/2,double(rtmin.y-rtmax.y)/(rtmin.x-rtmax.x),rtmin);
        x=(x1+x2)/2;
        line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0), 3, 8 );
        line( color_dst, lmin , lmax, Scalar(0,0,255), 3, 8 );
        line( color_dst, rtmin , rtmax, Scalar(0,0,255), 3, 8 );
        line( color_dst, midmin , midmax, Scalar(0,0,255), 3, 8 );
        if(lmax.y<rtmin.y)
        {
          left=0;
          right=1;
        }
        else
        {
          right=0;
          left=1;
        }
        if((contours.size()-rem)>1)
        {
          if(blink==0)
          {
            cout<<blink<<endl;
            cout<<"YO"<<endl;
            blink=1;
            clock_gettime(CLOCK_MONOTONIC, &start);
          }
        }
    }

    else if(b==2)
    {
        x=0;
        int x1;
        if(c==1)
        {
          x1=xcoord(img.rows/2,double(lmin.y-lmax.y)/(lmin.x-lmax.x),lmin);
          x = (x1+img.cols)/2;
          line( color_dst, lmin , lmax, Scalar(0,0,255), 3, 8 );
        }
        else if(d==1)
        {
          x1=xcoord(img.rows/2,double(rtmin.y-rtmax.y)/(rtmin.x-rtmax.x),rtmin);
          x = x1/2;
          line( color_dst, rtmin , rtmax, Scalar(0,0,255), 3, 8 );
        }
        line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0),3,8 );
        line( color_dst, midmin , midmax, Scalar(0,0,255), 3, 8 );
    }
    else if(b==1)
    {
      int x1;
      x=0;
        for(i=-1;i<1;i++)
        {
        for(j=-1;j<1;j++)
           {
               if(im.at<uchar>((img.rows/2)+i,((img.cols/2)+j)) != 255)
               {
                  tur=1;
                  break;
               }
            }
        }
        if(c==1)
        {
          x1=xcoord(img.rows/2,double(lmin.y-lmax.y)/(lmin.x-lmax.x),lmin);
          x = (x1+img.cols)/2;
          line( color_dst, lmin , lmax, Scalar(0,0,255), 3, 8 );
        }
        else if(d==1)
        {
          x1=xcoord(img.rows/2,double(rtmin.y-rtmax.y)/(rtmin.x-rtmax.x),rtmin);
          x = x1/2;
          line( color_dst, rtmin , rtmax, Scalar(0,0,255), 3, 8 );
        }
        else if(e==1)
        {
          if(tur==1 && right==1)
          {
            x=img.cols;
            line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0));
            //sendCommand("R");
          }
          else if(tur==1 && left ==1)
          {
            x=0;
            line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0));
            //sendCommand("L");
          }    
          else 
          {
            x=img.cols/2;
            line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0));
          }
        }
        line( color_dst, Point(img.cols/2,img.rows), Point(x,img.rows/2), Scalar(255,0,0),3,8 ); 
        tur=0;
    }

    angle = angle_between(Point(img.cols/2,0),Point(img.cols/2,img.rows),Point(x,img.rows/2));

       turn(angle,right,left);

    cout<<angle<<endl;

    clock_gettime(CLOCK_MONOTONIC, &finish);

    double elapsed = (finish.tv_sec - start.tv_sec);
    cout<<elapsed<<endl;

    if(elapsed>3 && elapsed<15 && blink==1)
    {
      blink=0;
      cout<<"blink blink"<<endl;
      cout<<blink<<endl;
      sendCommand("B");
    }

    cout<<blink<<endl;

   // elapsed=0;

    namedWindow("Source",WINDOW_NORMAL);
    imshow("Source",img1 );
    resizeWindow("Source",600,600);
    namedWindow("Detected Lines",WINDOW_NORMAL);
    imshow("Detected Lines",color_dst);
    resizeWindow("Detected Lines",600,600);
    waitKey(50);
}
}

double ycoord(int x,double slope,Point p)
{
    double y=(slope*x)+(p.y-(slope*(p.x)));
    return y;
}

double xcoord(int y,double slope,Point p)
{
    double x=double(y-(p.y-(slope*(p.x))))/slope;
    return x;
}

double angle_between(Point a, Point b, Point c)
{
    double inter_angle;
    //double slope1 = (double) (a.y - b.y) / (a.x - b.x);
    if((c.x-b.x)!=0)
    {
        double slope2 = (double) (c.y - b.y) / (c.x - b.x);

        //double inter_angle = (double) (atan((slope1 - slope2) / (1 + (slope1 * slope2)))) * 180 / 3.14;
        inter_angle = (double) (atan(slope2)) * 180 / 3.14;

        if(inter_angle<0)
            inter_angle = -(90+inter_angle);
        else
            inter_angle=90-inter_angle;
    }
    else
        inter_angle=0;

    return inter_angle;
}


double distance(Point a)
{
    return (sqrt(pow(a.x-(img.cols/2),2)+pow((a.y-img.rows),2)));
}

char turn(double a,int r,int l)
{
    if(a<0 && abs(a)>15)
    {
        cout<<"TURN RIGHT"<<endl;
        if(r==1)
           cout<<"APPROACHING RIGHT TURN"<<endl;
        else if(l==1)
           cout<<"APPROACHING LEFT TURN"<<endl;
         prev="D";
         sendCommand("D");
    }
    else if(a>0 && abs(a)>15)
    {
        cout<<"TURN LEFT"<<endl;
        if(r==1)
           cout<<"APPROACHING RIGHT TURN"<<endl;
        else if(l==1)
           cout<<"APPROACHING LEFT TURN"<<endl;
         prev="A";
         sendCommand("A");
    }
    else
    {
        cout<<"MOVE STRAIGHT"<<endl;
        if(r==1)
           cout<<"APPROACHING RIGHT TURN"<<endl;
        else if(l==1)
           cout<<"APPROACHING LEFT TURN"<<endl;
         prev="W";
         sendCommand("W");
    }
}