/**
 * @file lsd_opencv_example.cpp
 *
 * Test the LSD algorithm with OpenCV
 */
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
//#include <highgui.h>
//#include <cv.h>
#include <lsd.h>
#include <iostream>
using namespace cv;
using namespace std;
#define PI 3.14159265


typedef struct oneLine
{
    Point  start;
    Point  end;
    int count;
    int width;
    bool seen;
    oneLine(Point p1, Point p2, int w, int c)
    {
        start = p1;
        end = p2;
        width = w;
        count = c;
    }
}oneLine;

typedef struct linePoinits
{
    int count;
    vector<oneLine> lines;
}linePoints;


bool linkPoint(linePoinits& lines, Mat display);
bool linkPoint2(linePoinits& lines, Mat img);
int lengthTwoPoint(Point p0, Point p1);
int findMiniLength(const int length0, const int length1, const int length2, const int length3, int& state);
int findMiniLength(const int length0, const int length1, const int length2, const int length3, int& state);
float findAngle(const oneLine curr_line, const oneLine min_line);
void endToStart(Point& start, Point& end);
Point findOrient(const Point start, const Point end);
bool showLinkImg(const linePoinits lines, Mat display);

/*
bool linkPoint(linePoinits& lines, Mat display)
{
    cout<<"count"<<lines.count<<endl;
    for(int i=0; i<lines.count ; i++)
    {
        int min_distance = 10000;
        int min_count = -1;
        oneLine line1;
        line1.start = lines.start.at(i);
        line1.end = lines.end.at(i);
        for(int j=i+1; j<lines.count; j++)
        {
            oneLine line2;
            line2.start = lines.start.at(j);
            line2.end = lines.end.at(j);
            int length = sqrt((line1.end.x - line2.start.x)*(line1.end.x - line2.start.x) + (line1.end.y - line2.start.y)*(line1.end.y - line2.start.y));
            if(length < min_distance)
            {
                min_distance = length;
                min_count = j;
                //cout<<"length "<<length<<endl;
            }
        }
        if(min_distance < 20 )
        {
            cout<<" min_distance "<<min_distance<<endl;
            Point a1 = lines.end.at(i) - lines.start.at(i);
            Point a2 = lines.end.at(min_count) - lines.start.at(min_count);
            float length1 = sqrt(a1.x*a1.x + a1.y*a1.y);
            float length2 = sqrt(a2.x*a2.x + a2.y*a2.y);
            float angle = acos((float)(a1.x*a2.x + a1.y*a2.y)/(length1*length2))*180.0/PI; //
            cout<<"angle"<<angle<<endl;
            if(abs(angle - 90) < 10.0  )//vertical
            {
                cout<<"vertical line"<<endl;
                if(length1 < length2)
                    lines.end.at(i) = lines.start.at(min_count);
                else
                    lines.start.at(min_count) = lines.end.at(i) ;
            }
            else if (abs(angle ) < 10.0 )//horiental
            {
                cout<<"horiental process"<<endl;
                int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                if(length_another >= (length1 + length2 - 4))// mid point ==> link
                {
                    cout<<"horiental line :link"<<endl;
                    lines.end.at(i) = lines.end.at(min_count);
                    lines.start.at(min_count) = lines.start.at(i);

                }
                else //replace another
                {
                    cout<<"horiental line : replace"<<endl;
                    if(length1 < length2)
                    {
                        lines.start.at(i) = lines.start.at(min_count);
                        lines.end.at(i) = lines.end.at(min_count);
                    }
                    else
                    {
                        lines.start.at(min_count) = lines.start.at(i);
                        lines.end.at(min_count) = lines.end.at(i);
                    }

                }

            }
        }

    }
    for (int j = 0; j < lines.count ; j++)
    {
        cv::line(display, lines.start.at(j), lines.end.at(j), cv::Scalar(255),1,CV_AA);//lines.width.at(j)

        rectangle(display, cvPoint(lines.start.at(j).x-1,lines.start.at(j).y-1),
                  cvPoint(lines.start.at(j).x+1,lines.start.at(j).y+1),
                  CV_RGB(0,255,0),1,8);       // 画矩形点
        rectangle(display, cvPoint(lines.end.at(j).x-1,lines.end.at(j).y-1),
                  cvPoint(lines.end.at(j).x+1,lines.end.at(j).y+1),
                  CV_RGB(255,0,0),1,8);       // 画矩形点
    }
    imshow("linklines",display);
}

*/

/**
 * @brief linkPoint2
 * four status  :

 * 0: start  start
 * 1: start  end
 * 2: end start
 * 3: end end
 * @param lines
 * @param display
 * @return
 */
bool linkPoint2(linePoinits& lines, Mat img)
{
    static int replace, link;
    Mat display;
    img.copyTo(display);
    cout<<"count"<<lines.count<<endl;
    for(int i=0; i<lines.count ; i++)
    {
       // if(lines.lines.at(i).seen == true) continue;
        int min_distance = 10000;
        int min_count = -1;
        oneLine curr_line = lines.lines.at(i);
        int state = 2;
        for(int j=i+1; j<lines.count; j++)
        {
            oneLine temp_line = lines.lines.at(j);
            int length0 = lengthTwoPoint(curr_line.start,temp_line.start);
            int length1 = lengthTwoPoint(curr_line.start, temp_line.end);
            int length2 = lengthTwoPoint(curr_line.end, temp_line.start);
            int length3 = lengthTwoPoint(curr_line.end, temp_line.end);

            int length = findMiniLength(length0, length1, length2, length3, state);
            if(length < min_distance)
            {
                min_distance = length;
                min_count = j;
                lines.lines.at(min_count).seen = true;
            }
        }

        if(min_distance < 25 )
        {
            cout<<" min_distance "<<min_distance<<endl;
            oneLine min_line = lines.lines.at(min_count);

            Point  orient1 = findOrient(curr_line.start, curr_line.end);
            Point  orient2 = findOrient(min_line.start,  min_line.end);

            float curr_length = sqrt(orient1.x*orient1.x + orient1.y*orient1.y);
            float min_length = sqrt(orient2.x*orient2.x + orient2.y*orient2.y);

            float angle = acos((float)(orient1.x*orient2.x + orient1.y*orient2.y)/(curr_length*min_length))*180.0/PI; //


            //float angle = findAngle(curr_line, min_line);
            cout<<"angle"<<angle<<endl;

            if (abs(angle ) < 30.0 | abs(angle - 180) < 30.0 )//horiental
            {
                cout<<"horiental process"<<endl;
                switch(state)
                {
                case 0:
                {
                    break;
                }
                case 1:
                {
                    endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                    break;
                }
                case 2:
                {
                    endToStart(lines.lines.at(i).start, lines.lines.at(i).end);
                    //lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.end);
                    break;
                }
                case 3:
                {
                    endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                    endToStart(lines.lines.at(i).start, lines.lines.at(i).end);
                    //lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.start);
                    break;
                default :
                        break;
                    }
                }
                int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.end);
                //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                if(lengthEndPoint >= (curr_length + min_length - 6)) // middle point ==> link
                {
                    link++;
                    cout<<"horiental line :  link "<<lengthEndPoint<<endl;
                    cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),5,CV_AA);//lines.width.at(j)
                    cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),5,CV_AA);//lines.width.at(j)

                    lines.lines.at(i).start = lines.lines.at(min_count).end;
                    lines.lines.at(min_count).start = lines.lines.at(i).end;
                  //  cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                }
                else if(lengthEndPoint < curr_length | lengthEndPoint < min_length) //replace another
                {
                    replace++;
                    cout<<"horiental line : replace"<<endl;
                    if(curr_length < min_length)
                    {
                       cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                        lines.lines.at(i).start = lines.lines.at(min_count).start;
                        lines.lines.at(i).end = lines.lines.at(min_count).end;

                    }
                    else
                    {
                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                        lines.lines.at(min_count).start = lines.lines.at(i).start;
                        lines.lines.at(min_count).end = lines.lines.at(i).end;
                    }
                }
            }
        }
    }
    cout<<" link "<<link<<" replace "<<replace<< endl;
    showLinkImg(lines,display);

}

bool showLinkImg(const linePoinits lines, Mat display)
{
    for (int j = 0; j < lines.count ; j++)
    {
        cv::line(display, lines.lines.at(j).start, lines.lines.at(j).end, cv::Scalar(255),1,CV_AA);//lines.width.at(j)

        rectangle(display, cvPoint(lines.lines.at(j).start.x-1, lines.lines.at(j).start.y-1),
                  cvPoint(lines.lines.at(j).start.x+1, lines.lines.at(j).start.y+1),
                  CV_RGB(0,255,0),1,8);       // 画矩形点
        rectangle(display, cvPoint(lines.lines.at(j).end.x-1,lines.lines.at(j).end.y-1),
                  cvPoint(lines.lines.at(j).end.x+1,lines.lines.at(j).end.y+1),
                  CV_RGB(255,0,0),1,8);       // 画矩形点
    }
    imshow("linklines",display);
}
int lengthTwoPoint(Point p0, Point p1)
{
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}

int findMiniLength(const int length0, const int length1, const int length2, const int length3, int& state)
{
    int min =10000;
    if (length0 < min)
    {
        min = length0;
        state = 0;
    }
    if (length1 < min)
    {
        min = length1;
        state = 1;
    }
    if (length2 < min)
    {
        min = length2;
        state = 2;
    }

    if (length3 < min)
    {
        min = length3;
        state = 3;
    }
    return min;
}

Point findOrient(const Point start, const Point end)
{
    return  end -start;
}

float findAngle(const oneLine curr_line, const oneLine min_line)
{
    Point  a1 = findOrient(curr_line.start, curr_line.end);
    Point  a2 = findOrient(min_line.start,  min_line.end);

    float length1 = sqrt(a1.x*a1.x + a1.y*a1.y);
    float length2 = sqrt(a2.x*a2.x + a2.y*a2.y);
    float angle = acos((float)(a1.x*a2.x + a1.y*a2.y)/(length1*length2))*180.0/PI; //
}

void endToStart(Point& start, Point& end)
{
    Point temp;
    temp = start;
    start = end;
    end = temp;
}

int main(int argc, char **argv)
{
    if (argc < 2 || argc > 2)
    {
        std::cout << "Usage: lsd_opencv_example imageName" << std::endl;
        return -1;
    }
    cv::Mat src_raw = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
    Mat src;
    cv::cvtColor(src_raw, src, CV_RGB2GRAY);
//    imshow("gray_img",src);
    int row = src.rows; // number of rows
    int col = src.cols; // total number of elements per line
    for (int j=0; j<row; j++) {
        uchar* data= src.ptr<uchar>(j);
        for (int i=0; i<col; i++) {
            if(data[i] > 120 && data[i]<255)
                data[i] = 255;
            if(data[i] < 120 && data[i]>60)
                data[i] = 60;
        }
    }
 //   imshow("src_white",src);
    cv::Mat  newImg;
    cv::cvtColor(src_raw, newImg, CV_RGB2GRAY);
    for (int j=0; j<row; j++) {
        uchar* data= src.ptr<uchar>(j);
        uchar* data2= newImg.ptr<uchar>(j);
        for (int i=0; i<col; i++) {
            // if(data2[i] > 120 && data2[i]<255)
            data2[i] = data[i]*0.9 + data2[i]*0.1 ;
        }
    }


   // imshow("newImg",newImg);

    //    Mat guass_gray;
    //    GaussianBlur(src, guass_gray, Size( 3, 3 ), 0, 0 );

    Mat erode_src, dilate_src;
    cv::erode(newImg, erode_src, Mat(2,2,CV_8U), Point(-1,-1),4);
    cv::erode(erode_src, erode_src, Mat(2,2,CV_8U), Point(-1,-1),2);
    cv::erode(erode_src, erode_src, Mat(2,2,CV_8U), Point(-1,-1),2);
    cv::erode(erode_src, erode_src, Mat(2,2,CV_8U), Point(-1,-1),2);
    //cv::erode(erode_src, erode_src, Mat(2,2,CV_8U), Point(-1,-1),3);

    cv::dilate(erode_src,dilate_src , Mat(2,2,CV_8U), Point(-1,-1), 4);//膨胀
    cv::dilate(dilate_src,dilate_src , Mat(2,2,CV_8U), Point(-1,-1), 2);//膨胀
    cv::dilate(dilate_src,dilate_src , Mat(2,2,CV_8U), Point(-1,-1), 2);//膨胀
    cv::dilate(dilate_src,dilate_src , Mat(2,2,CV_8U), Point(-1,-1), 2);//膨胀

    //    int r = dilate_src.rows; // number of rows
    //    int c = dilate_src.cols; // total number of elements per line
    //    for (int j=0; j<r; j++) {
    //        uchar* data= dilate_src.ptr<uchar>(j);
    //        for (int i=0; i<c; i++) {
    //            if(data[i] > 150 && data[i]<225)
    //            data[i] = data[i]+30;
    //          }
    //    }
//    cv::namedWindow("erode_src", CV_WINDOW_AUTOSIZE);
//    cv::namedWindow("dilate_sec", CV_WINDOW_AUTOSIZE);
//    imshow("erode_src",erode_src);
//    imshow("dilate_sec",dilate_src);

    linePoinits lsd_lines;

    Mat img_64fc1;
    newImg.convertTo(img_64fc1, CV_64FC1);

    int cols  = img_64fc1.cols;
    int rows = img_64fc1.rows;

    image_double image = new_image_double(cols, rows);
    image->data = img_64fc1.ptr<double>(0);
    ntuple_list ntl = lsd(image);

    cv::Mat lsd_display = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::Mat lsd_display_point;
    src_raw.copyTo(lsd_display_point);
    cv::Point pt1, pt2;
    for (int j = 0; j != ntl->size ; ++j)
    {
        pt1.x = int(ntl->values[0 + j * ntl->dim]);
        pt1.y = int(ntl->values[1 + j * ntl->dim]);
        pt2.x = int(ntl->values[2 + j * ntl->dim]);
        pt2.y = int(ntl->values[3 + j * ntl->dim]);
        int width = int(ntl->values[4 + j * ntl->dim]);

        double length = sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y) );
        cout<<length<<endl;
        if(width >2 && length>5)
            // if(width >2 && length>15)
        {
            cv::line(lsd_display, pt1, pt2, cv::Scalar(255), width, CV_AA);
            cv::line(lsd_display_point, pt1, pt2, cv::Scalar(255), 1, CV_AA);//width

            oneLine  line(pt1, pt2, width, j);
            line.seen = false;
            lsd_lines.lines.push_back(line);
            lsd_lines.count++;

            rectangle(lsd_display_point, cvPoint(pt1.x-1,pt1.y-1),
                      cvPoint(pt1.x+1,pt1.y+1),
                      CV_RGB(0,255,0),1,8);       // 画矩形点 //green
            rectangle(lsd_display_point, cvPoint(pt2.x-1,pt2.y-1),
                      cvPoint(pt2.x+1,pt2.y+1),
                      CV_RGB(255,0,0),1,8);       // 画矩形点 //red
        }
    }
    free_ntuple_list(ntl);
    linkPoint2(lsd_lines, src_raw);

//    cv::namedWindow("src", CV_WINDOW_AUTOSIZE);
//    cv::imshow("src", src);
//    cv::namedWindow("lsd_display", CV_WINDOW_AUTOSIZE);
//    cv::imshow("lsd_display", lsd_display);
    cv::imshow("lsd_display_point", lsd_display_point);
    //    Mat img_erode;
    //    cv::dilate(lsd_display, img_erode, Mat(2,2,CV_8U), Point(-1,-1), 1);//膨胀
    // cv::erode(lsd_display, img_erode, Mat(2,2,CV_8U), Point(-1,-1),1);
    // cv::erode(img_erode, img_erode, Mat(2,2,CV_8U), Point(-1,-1),1);
    //imshow("img_erode", img_erode);
    // cv::dilate(img_erode, src, Mat(2,2,CV_8U), Point(-1,-1), 1);//膨胀
    //cv::imshow("lsd_display2", img_erode);

    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}
