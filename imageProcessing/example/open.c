#include "opencv2/opencv.hpp"
#include <iostream>
#include <math.h>

#define SEARCH_RANGE 30
#define Sampling 5
#define vector_size 15
#define Hue_impo 0.8
#define Sat_impo 0.1
#define Val_impo 0.1

IplImage *img, *img2;

CvPoint LU_Point;
CvPoint RD_Point;
CvPoint Center;

int Rectangle_width;
int Rectangle_height;
bool IsRectSet = false;
bool IsModelHistogram = false;
bool IsPointSet = false;
bool back_img = false;

double histogram[16] = { 0, };
double next_frame_histogram[16] = { 0, };

void reset_next_frame_histogram();
void on_mouse(int event, int x, int y, int flags, void* param);
void make_model_histogram(CvPoint LU, CvPoint RD);
//void make_next_frame_histogram(CvPoint LU, CvPoint RD);
int get_index(int sum);
double get_model_histogram_value(int x, int y);
//double get_next_frame_histogram_value(int x, int y);
double get_w(int x, int y);

int min(int x, int y)
{
   return x > y ? y : x;
}
int max(int x, int y)
{
   return x > y ? x : y;
}

int main()
{
   
   CvCapture *video;
   CvPoint LU, RD;
   int key, height, width;
   double total_w = 0, w;

   cvNamedWindow("video", CV_WINDOW_AUTOSIZE);
   video = cvCaptureFromAVI("video2.avi");
   img = cvQueryFrame(video);


   do
   {
      img2 = cvCreateImage(cvGetSize(img), IPL_DEPTH_8U, 3);
      cvCvtColor(img, img2, CV_BGR2HSV);
      
      if(!IsRectSet)
         cv::setMouseCallback("video", on_mouse, NULL);
      
      if (IsRectSet)
      {
         if (!IsPointSet)
         {
            LU = LU_Point;
            RD = RD_Point;
            Center.x = (LU.x + RD.x) / 2;
            Center.y = (LU.y + RD.y) / 2;
            std::cout << Rectangle_width << ' ' << Rectangle_height << std::endl;
            IsPointSet = true;
         }
         cvRectangle(img2, LU, RD, CV_RGB(255, 255, 255));
         if(!IsModelHistogram)
         {
            make_model_histogram(LU, RD);
            IsModelHistogram = true;
         }
      }

      cvShowImage("video", img);
      cvShowImage("video2", img2);

      cv::Mat Mat_img = cv::cvarrToMat(img2,true);
      cv::Vec3b * pixel = NULL;
      if (IsRectSet)
      {
         uchar hue;
         total_w = 0;

         CvPoint search_LU = { max(LU.x - SEARCH_RANGE, 0) , max(LU.y - SEARCH_RANGE, 0) };
         CvPoint search_RD = { min(RD.x + SEARCH_RANGE, Mat_img.rows) , min(RD.y + SEARCH_RANGE, Mat_img.cols) };

         double next_x = 0, next_y = 0;
         int abs_x = 0;
         int abs_y = 0;

         
         if (!back_img)
         {
            //mean shift
            do {
               next_x = 0, next_y = 0;
               search_LU = { max(LU.x - SEARCH_RANGE, 0) , max(LU.y - SEARCH_RANGE, 0) };
               search_RD = { min(RD.x + SEARCH_RANGE, Mat_img.rows) , min(RD.y + SEARCH_RANGE, Mat_img.cols) };

               for (int i = search_LU.x; i < search_RD.x; i+=Sampling)
               {
                  for (int j = search_LU.y; j < search_RD.y; j+=Sampling)
                  {
                        
                     w = get_w(i,j);   
                     total_w += w;
                     next_x += w*i;
                     next_y += w*j;
                  }
               }
               
               next_x = next_x / total_w;
               next_y = next_y / total_w;

               abs_x = abs(Center.x - next_x);
               abs_y = abs(Center.y - next_y);
               Center.x = next_x;
               Center.y = next_y;
               LU.x = max(Center.x - Rectangle_width / 2, 0);
               LU.y = max(Center.y - Rectangle_height / 2, 0);
               RD.x = min(LU.x + Rectangle_width, Mat_img.rows);
               RD.y = min(LU.y + Rectangle_height, Mat_img.cols);
               
            } while (sqrt(std::pow(abs_x,2) + std::pow(abs_y, 2))>vector_size); 

            std::cout << Center.x << ' ' << Center.y << std::endl;


            
            //backprojection image
            for (int i = max(LU.x - Rectangle_width / 2, 0); i <min(RD.x + Rectangle_width / 2, img2->width); i+=5)//for (int i = search_LU.x; i < search_RD.x; i += 5)
            {
               pixel = Mat_img.ptr<cv::Vec3b>(i);
               for (int j = max(LU.y - Rectangle_width / 2, 0); j < min(RD.y + Rectangle_width / 2, img2->height); j+=5)//for (int j = search_LU.y; j < search_RD.y; j += 5)
               {
                  w = get_w(i, j);
                     //w = sqrt(1 * get_model_histogram_value(j, i));
                     //w = get_model_histogram_value(j, i);
                     //w = sqrt(get_model_histogram_value(j, i) * get_next_frame_histogram_value(j, i));
                     //w = sqrt(get_model_histogram_value(j, i) / get_next_frame_histogram_value(j, i));
                  // 새 픽셀의 color histogram은 hue가 속하는 범위만 1이고 나머지는 0, 
                  //따라서 1*model color histogram(pixel)해주면 됨, 앞의 255는 흑백영상이 0~255값이라 상대값을 구하기 위해서임.
                  pixel[j][0] = (uchar)255 * w;
                  pixel[j][1] = (uchar)255 * w;
                  pixel[j][2] = (uchar)255 * w;
                  /*for (int x = max(i - Rectangle_width / 2, 0); x < min(i + Rectangle_width / 2, img2->width); x++)
                  {
                     
                     for (int y = max(j - Rectangle_width / 2, 0); y < min(j + Rectangle_width / 2, img2->height); y++)
                     {
                        
                     }
                  }*/
               }

            } //backprojection
         }

         
         /*
         if (back_img)
         {
            for (int i = 0; i < Mat_img.rows; i++)
            {
               pixel = Mat_img.ptr<cv::Vec3b>(i);
               for (int j = 0; j < Mat_img.cols; j++)
               {
                  
                  if (get_next_frame_histogram_value(j, i) == 0)
                     w = 0;
                  else
                     w = get_model_histogram_value(j, i);
                     //w = sqrt(get_model_histogram_value(j, i) * get_next_frame_histogram_value(j, i));
                     //w = sqrt(get_model_histogram_value(j, i) / get_next_frame_histogram_value(j, i));
                  pixel[j][0] = (uchar)255*w; // 새 픽셀의 color histogram은 hue가 속하는 범위만 1이고 나머지는 0, 
                                                                       //따라서 1*model color histogram(pixel)해주면 됨, 앞의 255는 흑백영상이 0~255값이라 상대값을 구하기 위해서임.
                  pixel[j][1] = (uchar)255*w;
                  pixel[j][2] = (uchar)255*w;
                  
               }

            } //backprojection
         }
         */
         
      }

      cv::imshow("mat", Mat_img);
      

      key = cvWaitKey(30);
      if (key == 27) break;//ESC key
      if (IsRectSet)
      img = cvQueryFrame(video);
   } while (img);

   return 0;
}

void reset_next_frame_histogram()
{
   for (int i = 0; i < 16; i++)
      next_frame_histogram[i] = 0;
}

void on_mouse(int event, int x, int y, int flags, void* param) {   
   if (event == CV_EVENT_LBUTTONDOWN) {
      LU_Point.x = x;
      LU_Point.y = y;
      //std::cout << LU_Point.x << ' ' << LU_Point.y << std::endl;
   }
   if (event == CV_EVENT_LBUTTONUP)
   {
      RD_Point.x = x;
      RD_Point.y = y;
      //Center.x = (LU_Point.x + RD_Point.x) / 2;
      //Center.y = (LU_Point.y + RD_Point.y) / 2;
      Rectangle_width = (RD_Point.x - LU_Point.x);
      Rectangle_height = (RD_Point.y - LU_Point.y);
      //std::cout << RD_Point.x << ' ' << RD_Point.y << std::endl;
      IsRectSet = true;
   }
}

void make_model_histogram(CvPoint LU, CvPoint RD) //히스토그램 만들기
{
   int total_pixel=0;
   for (int i = LU.x; i < RD.x; i++)
   {
      for (int j = LU.y; j < RD.y; j++)
      {
         total_pixel++;
         CvScalar s = cvGet2D(img2, j, i);
         int hue = (int)s.val[0];
         int saturation = (int)s.val[1];
         int value = (int)s.val[2];
         int sum = (hue*Hue_impo +saturation*Sat_impo + value*Val_impo);
         histogram[sum / 16]++;
      }
   }
   int total = 0;
   for (int i = 0; i < 16; i++)
   {
      total += histogram[i] * histogram[i];// = next_frame_histogram[i] / total_pixel;
   }
   total = sqrt(total);
   /*for (int i = 0; i < 16; i++)
      histogram[i] = histogram[i] / total_pixel;*/
   for (int i = 0; i < 16; i++)
   {
      histogram[i] /= total;
   }
}
/*
void make_next_frame_histogram(CvPoint LU, CvPoint RD)
{
   int total_pixel = 0;
   for (int i = LU.x; i < RD.x; i++)
   {
      for (int j = LU.y; j < RD.y; j++)
      {
         total_pixel++;
         CvScalar s = cvGet2D(img2, j, i);
         int hue = (int)s.val[0];
         int saturation = (int)s.val[1];
         int value = (int)s.val[2];
         int sum = (hue*Hue_impo);// +saturation*Sat_impo + value*Val_impo);
         next_frame_histogram[sum / 16]++;
      }
   }

   int total=0;
   for (int i = 0; i < 16; i++)
   {
      total += next_frame_histogram[i] * next_frame_histogram[i];// = next_frame_histogram[i] / total_pixel;
   }
   total = sqrt(total);
   for (int i = 0; i < 16; i++)
   {
      next_frame_histogram[i] /= total;
   }
}
*/
int get_index(int sum)
{
   return sum / 16;
}

double get_model_histogram_value(int x, int y)
{
   CvScalar s = cvGet2D(img2, y, x);
   //std::cout << x << ' ' << y << std::endl;
   int hue = (int)s.val[0];
   int saturation = (int)s.val[1];
   int value = (int)s.val[2];
   int sum = (hue*Hue_impo +saturation*Sat_impo + value*Val_impo);
   return histogram[get_index(sum)];
}
/*
double get_next_frame_histogram_value(int x, int y)
{
   CvScalar s = cvGet2D(img2, y, x);
   //std::cout << x << ' ' << y << std::endl;
   int hue = (int)s.val[0];
   int saturation = (int)s.val[1];
   int value = (int)s.val[2];
   int sum = (hue*Hue_impo);// +saturation*Sat_impo + value*Val_impo);
   return next_frame_histogram[get_index(sum)];
}
*/
double get_w(int x, int y)//중심좌표에서 사각형 크기만큼 히스토그램을 구하고 모델 히스토그램하고의 유사도를 구함
{
   double w = 0;
   int total_pixel = 0;
   reset_next_frame_histogram();
   for (int i = max(x-Rectangle_width/2,0); i < min(x + Rectangle_width/2, img2->width); i++)
   {
      for (int j = max(y - Rectangle_width / 2, 0); j < min(y + Rectangle_width / 2, img2->height); j++)
      {
         total_pixel++;
         CvScalar s = cvGet2D(img2, j, i);
         int hue = (int)s.val[0];
         int saturation = (int)s.val[1];
         int value = (int)s.val[2];
         int sum = (hue*Hue_impo+saturation*Sat_impo + value*Val_impo);
         next_frame_histogram[sum / 16]++;
      }
   }
   int total = 0;
   for (int i = 0; i < 16; i++)
   {
      total += next_frame_histogram[i] * next_frame_histogram[i];
   }
   total = sqrt(total);
   for (int i = 0; i < 16; i++)
   {
      next_frame_histogram[i] /= total;
   }
   for (int i = 0; i < 16; i++)
   {
      w += sqrt(histogram[i] * next_frame_histogram[i]);
   }
   return w;
}