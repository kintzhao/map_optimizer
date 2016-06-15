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
#include <cmath>
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
    oneLine(Point p1, Point p2)
    {
        start = p1;
        end = p2;
    }
}oneLine;

typedef struct linePoinits
{
    int count;
    vector<oneLine> lines;
}linePoints;

bool linkPoint(linePoinits& lines, Mat display);
bool linkPoint2(linePoinits& lines, Mat img);
bool linkPoint3(linePoinits& lines, Mat img);
int lengthTwoPoint(Point p0, Point p1);
int findMiniLength(const int length0, const int length1, const int length2, const int length3, int& state);
int findMiniLength(const int length0, const int length1, const int length2, const int length3, int& state);
float findAngle(const oneLine curr_line, const oneLine min_line);
void endToStart(Point& start, Point& end);
Point findOrient(const Point start, const Point end);
bool showLinkImg(const linePoinits lines, Mat display);
float distanceLineWithPoint(const oneLine line, const Point p);
void lineEquation(const oneLine line, float& A, float& B, float& C);
int distanceTwoLine(oneLine line1, oneLine line2);
string int2str(int num);
bool imgProcess(vector<rect>& lines, Mat img);
bool imgProcess2(vector<rect>& lines, Mat img);
int distanceTwoRect(rect rec1, rect rec2);
bool displayRect(const vector<rect> lines, Mat& display);



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
    static int replace, link , joint;
    replace = 0;
    link = 0;
    joint = 0;
    Mat display_test, display, result;
    img.copyTo(display_test);
    img.copyTo(display);
    img.copyTo(result);
    cout<<"count"<<lines.count<<endl;
    for(int i=0; i<lines.count ; i++)
    {
        // if(lines.lines.at(i).seen == true) continue;
        int min_distance = 10000;
        int min_count = -1;
        oneLine curr_line = lines.lines.at(i);
        int state = 2;
        for(int j=0; j<lines.count; j++)
        {
            if(i==j) continue;
            oneLine temp_line = lines.lines.at(j);
            int length0 = lengthTwoPoint(curr_line.start, temp_line.start);
            int length1 = lengthTwoPoint(curr_line.start, temp_line.end);
            int length2 = lengthTwoPoint(curr_line.end, temp_line.start);
            int length3 = lengthTwoPoint(curr_line.end, temp_line.end);

            //int line4 = distanceTwoLine(curr_line, temp_line);
            int length = findMiniLength(length0, length1, length2, length3, state);

            if(length < min_distance)
            {
                min_distance = length;
                min_count = j;
                lines.lines.at(min_count).seen = true;
            }
        }

        if(min_distance < 20 )
        {
            cout<<" min_distance "<<min_distance<<endl;
            oneLine min_line = lines.lines.at(min_count);

            Point  orient1 = findOrient(curr_line.start, curr_line.end);
            Point  orient2 = findOrient(min_line.start,  min_line.end);

            float curr_length = sqrt(orient1.x*orient1.x + orient1.y*orient1.y);
            float min_length = sqrt(orient2.x*orient2.x + orient2.y*orient2.y);

            float cost_value = (float)(orient1.x*orient2.x + orient1.y*orient2.y)/(curr_length*min_length);
            if(cost_value < -1 && cost_value > -2) cost_value = -1;
            else if(cost_value > 1 && cost_value <2)  cost_value = 1;

            float angle = acos(cost_value)*180.0/M_PI;//PI; //
            //float angle = findAngle(curr_line, min_line);
            cout<<"angle"<<angle<<endl;

            cout<<"horiental process"<<endl;
            switch(state)
            {
            case 0:
            {
                break;
            }
            case 1:
            {
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end ;
                endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                //cout<< " "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end<<endl;
                break;
            }
            case 2:
            {
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end ;
                endToStart(lines.lines.at(i).start, lines.lines.at(i).end);
                //cout<< " "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end<<endl;
                break;
            }
            case 3:
            {
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end ;
                //cout<<" "<<state<<" "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end ;

                endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                endToStart(lines.lines.at(i).start, lines.lines.at(i).end);

                //cout<< " "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end;
                //cout<< " "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end<<endl;
                break;
            default :
                    break;
                }
            }

            if (abs(angle ) < 20.0 | abs(angle - 180) < 20.0 )//horiental
            {
                int line_distance = distanceTwoLine(curr_line, min_line);
                int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.end);
                //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                if(lengthEndPoint >= (curr_length + min_length)*0.7 && min_distance < 8) // middle point ==> link
                    //                if(lengthEndPoint >=  curr_length && lengthEndPoint >= min_length ) // middle point ==> link
                {
                    link++;
                    // change two line to one line
                    //                    cout<<"horiental line :  link "<<lengthEndPoint<<endl;
                    //                    cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                    //                    cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                    //                    lines.lines.at(i).start = lines.lines.at(min_count).end;
                    //                    lines.lines.at(min_count).start = lines.lines.at(i).end;
                    //                    cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                    if(curr_length < min_length)
                    {
                        cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                        lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced

                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                    }
                    else
                    {
                        cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                        lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                    }

                }
                else if(lengthEndPoint < curr_length | lengthEndPoint < min_length | line_distance <12) //replace another
                {
                    replace++;
                    cout<<"horiental line : replace"<<endl;
                    if(curr_length < min_length)
                    {
                        cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                        lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced
                        lines.lines.at(i).end = lines.lines.at(min_count).end;

                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                    }
                    else
                    {
                        cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                        lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                        lines.lines.at(min_count).end = lines.lines.at(i).end;

                        cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                    }
                }
            }
            else if(min_distance < 10)
            {
                int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.end);
                //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                if(lengthEndPoint >=  curr_length && lengthEndPoint >= min_length ) // middle point ==> link
                {
                    joint++;
                    /*   // change two line to one line
                    cout<<"horiental line :  link "<<lengthEndPoint<<endl;
                    cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                    cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                    lines.lines.at(i).start = lines.lines.at(min_count).end;
                    lines.lines.at(min_count).start = lines.lines.at(i).end;
                    cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                    */
                    if(curr_length < min_length)
                    {
                        cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                        cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)

                        lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced
                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                        cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                    }
                    else
                    {
                        cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                        cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)

                        lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                        cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                        cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,0,255),1,CV_AA);//lines.width.at(j)
                    }
                }
            }
        }
    }
    cout<<" link "<<link<<" replace "<<replace<<" joint "<<joint<< endl;
    showLinkImg(lines, result);
    imshow("display_test",display_test);
    imwrite("./linkPoint.png", display_test);
    imwrite("./result.png", result);
    imwrite("./display.png", display);
    imshow("display",display);
    imshow("result",result);
}
/**
 * @brief imgProcess
 * img_test :--------
 * r current_i     display
 * g img_j    dele       : width
 * b img_i  ==dele  :width<=2 and length <6
 *
 *
 * @param lines
 * @param img
 * @return
 */
bool imgProcess(vector<rect>& lines, Mat img)
{
    //imshow("raw",img);
    Mat display, display_test;
    img.copyTo(display);
    img.copyTo(display_test);
    vector<rect> near_lines;
    cout<<" lines.size()  "<<lines.size()<<endl;
    for (int i = 0; i != lines.size(); ++i)
    {

        rect current_line = lines.at(i);
        cout<<" current_theta: "<<current_line.theta<<endl;
        if(current_line.width<=2 | current_line.length <10)
        {
            lines.erase(lines.begin()+i);
            cv::line(display_test, Point(current_line.x1, current_line.y1), Point(current_line.x2, current_line.y2), CV_RGB(0,0,255), 1/*current_line.width*/ , CV_AA);//lines.width.at(j)

            i=i-1;
            continue;
        }

        near_lines.clear();
        near_lines.push_back(current_line);
        for(int j=i+1; j!= lines.size(); ++j)
        {
            //if(i == j) continue ;
            rect temp_line = lines.at(j);
            int min_distance = distanceTwoRect(current_line, temp_line);
            int min_center_distance = lengthTwoPoint(Point(current_line.x, current_line.y), Point(temp_line.x, temp_line.y));
            //cout<<" min_distance  "<<min_distance<<" min_center_distance "<<min_center_distance<<endl;
            if( min_distance < (current_line.width + temp_line.width) && min_center_distance < (current_line.width + temp_line.width)*1.5 && abs(current_line.theta - temp_line.theta) < 50.0*3.14/180 )
            {
                // cout<<" min_distance  "<<min_distance<<endl;
                near_lines.push_back(temp_line);

                cv::line(display_test, Point(temp_line.x1, temp_line.y1), Point(temp_line.x2, temp_line.y2), CV_RGB(0,255,0), 1/*temp_line.width*/ , CV_AA);//lines.width.at(j)

                lines.erase(lines.begin()+j);
                j--;
            }
        }
        int min_x = 1000, min_y = 10000, max_x = 0, max_y = 0;
        Point center ;
        Point2d theta;
        double angle;
        for(int k =0; k!= near_lines.size(); ++k)
        {
            if ( near_lines.at(k).x1 > max_x ) max_x = near_lines.at(k).x1;
            if ( near_lines.at(k).x2 > max_x ) max_x = near_lines.at(k).x2;

            if ( near_lines.at(k).x1 < min_x ) min_x = near_lines.at(k).x1;
            if ( near_lines.at(k).x2 < min_x ) min_x = near_lines.at(k).x2;

            if ( near_lines.at(k).y1 > max_y ) max_y = near_lines.at(k).y1;
            if ( near_lines.at(k).y2 > max_y ) max_y = near_lines.at(k).y2;

            if ( near_lines.at(k).y1 < min_y ) min_y = near_lines.at(k).y1;
            if ( near_lines.at(k).y2 < min_y ) min_y = near_lines.at(k).y2;
            center.x += near_lines.at(k).x;
            center.y += near_lines.at(k).y;
            theta.x += near_lines.at(k).dx;
            theta.y += near_lines.at(k).dy;
            angle += near_lines.at(k).theta;
        }
        center.x /=near_lines.size();
        center.y /=near_lines.size();
        theta.x /=near_lines.size();
        theta.y /=near_lines.size();
        angle /=near_lines.size();

        double l_min = lengthTwoPoint(Point(min_x, min_y), center) ;
        double l_max = lengthTwoPoint(Point(max_x, max_y), center) ;

        //        lines.at(i).x1 = center.x + l_min * theta.x;
        //        lines.at(i).y1 = center.y + l_min * theta.y;
        //        lines.at(i).x2 = center.x + l_max * theta.x;
        //        lines.at(i).y2 = center.y + l_max * theta.y;
        //        //lines.at(i).width = w_max - w_min;
        //        lines.at(i).x = center.x;
        //        lines.at(i).y = center.y;
        //        //lines.at(i).theta = theta;
        //        lines.at(i).dx = theta.x;
        //        lines.at(i).dy = theta.y;
        //       // lines.at(i).prec = prec;
        //       // lines.at(i).p = p;
        //        lines.at(i).length =  l_max + l_min;

        cv::line(display_test, Point(lines.at(i).x1, lines.at(i).y1), Point(lines.at(i).x2, lines.at(i).y2), CV_RGB(255,0,0), 1/*temp_line.width*/ , CV_AA);//lines.width.at(j)

        if(current_line.theta <= M_PI/2.0)
        {
            int length = lengthTwoPoint(Point(min_x,min_y), Point(max_x, max_y)) ;
            lines.at(i).x1 = min_x;//-1;
            lines.at(i).x2 = max_x;//+1;
            lines.at(i).y1 = min_y;//-1;
            lines.at(i).y2 = max_y;//+1;
            lines.at(i).length = length;
        }
        else{
            int length = lengthTwoPoint(Point(min_x,max_y), Point(max_x, min_y)) ;
            lines.at(i).x1 = min_x;//-1;
            lines.at(i).x2 = max_x;//+1;
            lines.at(i).y1 = max_y;//+1;
            lines.at(i).y2 = min_y;//-1;
            lines.at(i).length = length;
        }
    }

    cout<<" lines.size()  "<<lines.size()<<endl;
    for (int j = 0; j < lines.size(); j++)
    {
        static int c = 0;
        rect temp_line = lines.at(j);
        // if(c%50 ==2)
        if(temp_line.length > 20)
        {
        cv::line(display, Point(temp_line.x1, temp_line.y1), Point(temp_line.x2, temp_line.y2), CV_RGB(0,0,255),1 /*temp_line.width*/ , CV_AA);//lines.width.at(j)

//        if(j%20 == 5)  //no order
//            cv::putText(display,int2str(j),cvPoint(temp_line.x-1,temp_line.y-1),
//                        CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0) );

        //        rectangle(display, cvPoint(temp_line.x1-1, temp_line.y1-1),
        //                  cvPoint(temp_line.x1+1, temp_line.y1+1),
        //                  CV_RGB(0,255,0),1,8);       // 画矩形点
        //        rectangle(display, cvPoint(temp_line.x2-1,temp_line.y2-1),
        //                  cvPoint(temp_line.x2+1,temp_line.y2+1),
        //                  CV_RGB(255,0,0),1,8);       // 画矩形点
        // if(c%50 ==2)
        //        rectangle(display, cvPoint(temp_line.x1,temp_line.y1),
        //                  cvPoint(temp_line.x2,temp_line.y2),
        //                  CV_RGB(255,0,0),1,8);       // 画矩形点
        c++;
        }
    }
    imshow("display",display);
    imshow("display_test",display_test);
}

bool imgProcess2(vector<rect>& lines, Mat img)
{
    Mat display, display_test;
    img.copyTo(display);
    img.copyTo(display_test);
    vector<rect> near_lines;
    cout<<" lines.size()  "<<lines.size()<<endl;
    for (int i = 0; i != lines.size(); ++i)
    {
        rect current_line = lines.at(i);
        cout<<" current_theta: "<<current_line.theta<<endl;
        if(current_line.width<=2 | current_line.length <10)
        {
            lines.erase(lines.begin()+i);
            cv::line(display_test, Point(current_line.x1, current_line.y1), Point(current_line.x2, current_line.y2), CV_RGB(0,0,255), 1/*current_line.width*/ , CV_AA);//lines.width.at(j)
            i=i-1;
            continue;
        }
        near_lines.clear();
        near_lines.push_back(current_line);
        for(int j=i+1; j!= lines.size(); ++j)
        {
            //if(i == j) continue ;
            rect temp_line = lines.at(j);
            int min_distance = distanceTwoRect(current_line, temp_line);
            int min_center_distance = lengthTwoPoint(Point(current_line.x, current_line.y), Point(temp_line.x, temp_line.y));
            //cout<<" min_distance  "<<min_distance<<" min_center_distance "<<min_center_distance<<endl;
            if( min_distance < (current_line.width + temp_line.width) && min_center_distance < (current_line.width + temp_line.width)*1.5 && abs(current_line.theta - temp_line.theta) < 50.0*3.14/180 )
            {
                // cout<<" min_distance  "<<min_distance<<endl;
                near_lines.push_back(temp_line);
                cv::line(display_test, Point(temp_line.x1, temp_line.y1), Point(temp_line.x2, temp_line.y2), CV_RGB(0,255,0), 1/*temp_line.width*/ , CV_AA);//lines.width.at(j)
                lines.erase(lines.begin()+j);
                j--;
            }
        }
        int min_x = 1000, min_y = 10000, max_x = 0, max_y = 0;
        for(int k =0; k!= near_lines.size(); ++k)
        {
            if ( near_lines.at(k).x1 > max_x ) max_x = near_lines.at(k).x1;
            if ( near_lines.at(k).x2 > max_x ) max_x = near_lines.at(k).x2;

            if ( near_lines.at(k).x1 < min_x ) min_x = near_lines.at(k).x1;
            if ( near_lines.at(k).x2 < min_x ) min_x = near_lines.at(k).x2;

            if ( near_lines.at(k).y1 > max_y ) max_y = near_lines.at(k).y1;
            if ( near_lines.at(k).y2 > max_y ) max_y = near_lines.at(k).y2;

            if ( near_lines.at(k).y1 < min_y ) min_y = near_lines.at(k).y1;
            if ( near_lines.at(k).y2 < min_y ) min_y = near_lines.at(k).y2;
        }
        cv::line(display_test, Point(lines.at(i).x1, lines.at(i).y1), Point(lines.at(i).x2, lines.at(i).y2), CV_RGB(255,0,0), 1/*temp_line.width*/ , CV_AA);//lines.width.at(j)
        if(current_line.theta <= M_PI/2.0)
        {
            int length = lengthTwoPoint(Point(min_x,min_y), Point(max_x, max_y)) ;
            lines.at(i).x1 = min_x;//-1;
            lines.at(i).x2 = max_x;//+1;
            lines.at(i).y1 = min_y;//-1;
            lines.at(i).y2 = max_y;//+1;
            lines.at(i).length = length;
        }
        else{
            int length = lengthTwoPoint(Point(min_x,max_y), Point(max_x, min_y)) ;
            lines.at(i).x1 = min_x;//-1;
            lines.at(i).x2 = max_x;//+1;
            lines.at(i).y1 = max_y;//+1;
            lines.at(i).y2 = min_y;//-1;
            lines.at(i).length = length;
        }
    }

    cout<<" lines.size()  "<<lines.size()<<endl;
    for (int j = 0; j < lines.size(); j++)
    {
        rect temp_line = lines.at(j);
        cv::line(display, Point(temp_line.x1, temp_line.y1), Point(temp_line.x2, temp_line.y2), CV_RGB(0,0,255),1 /*temp_line.width*/ , CV_AA);//lines.width.at(j)
//            cv::putText(display,int2str(j),cvPoint(temp_line.x-1,temp_line.y-1),
//                        CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0) );
        //        rectangle(display, cvPoint(temp_line.x1-1, temp_line.y1-1),
        //                  cvPoint(temp_line.x1+1, temp_line.y1+1),
        //                  CV_RGB(0,255,0),1,8);       // 画矩形点
        //        rectangle(display, cvPoint(temp_line.x2-1,temp_line.y2-1),
        //                  cvPoint(temp_line.x2+1,temp_line.y2+1),
        //                  CV_RGB(255,0,0),1,8);       // 画矩形点
    }
    imshow("display",display);
    imshow("display_test",display_test);
}


bool showLinkImg(const linePoinits lines, Mat display)
{
    for (int j = 0; j < lines.count-50 ; j++)
    {
        cv::line(display, lines.lines.at(j).start, lines.lines.at(j).end, cv::Scalar(255), lines.lines.at(j).width , CV_AA);//lines.width.at(j)

        rectangle(display, cvPoint(lines.lines.at(j).start.x-1, lines.lines.at(j).start.y-1),
                  cvPoint(lines.lines.at(j).start.x+1, lines.lines.at(j).start.y+1),
                  CV_RGB(0,255,0),1,8);       // 画矩形点
        rectangle(display, cvPoint(lines.lines.at(j).end.x-1,lines.lines.at(j).end.y-1),
                  cvPoint(lines.lines.at(j).end.x+1,lines.lines.at(j).end.y+1),
                  CV_RGB(255,0,0),1,8);       // 画矩形点
    }
    imshow("linklines",display);
}
bool displayRect(const vector<rect> lines, Mat& display)
{
    for (int j = 0; j < lines.size(); j++)
    {
        cv::line(display, Point(lines.at(j).x1, lines.at(j).y1), Point(lines.at(j).x2, lines.at(j).y2), cv::Scalar(255), 1/*lines.at(j).width*/ , CV_AA);//lines.width.at(j)

        rectangle(display, cvPoint(Point(lines.at(j).x1, lines.at(j).y1).x-1, Point(lines.at(j).x1, lines.at(j).y1).y-1),
                  cvPoint(Point(lines.at(j).x1, lines.at(j).y1).x+1, Point(lines.at(j).x1, lines.at(j).y1).y+1),
                  CV_RGB(0,255,0),1,8);       // 画矩形点
        rectangle(display, cvPoint(Point(lines.at(j).x2, lines.at(j).y2).x-1,Point(lines.at(j).x2, lines.at(j).y2).y-1),
                  cvPoint(Point(lines.at(j).x2, lines.at(j).y2).x+1,Point(lines.at(j).x2, lines.at(j).y2).y+1),
                  CV_RGB(255,0,0),1,8);       // 画矩形点
    }
}
int lengthTwoPoint(Point p0, Point p1)
{
    return sqrt((p0.x-p1.x)*(p0.x-p1.x) + (p0.y-p1.y)*(p0.y-p1.y));
}

//return max distance
int distanceTwoLine(oneLine line1, oneLine line2)
{
    int min_distance;
    int len1 = lengthTwoPoint(line1.start, line1.start);
    int len2 = lengthTwoPoint(line2.start, line2.end);

    if(len1 <= len2)
    {
        int distance1 = distanceLineWithPoint(line2, line1.start);
        int distance2 = distanceLineWithPoint(line2, line1.end);
        if(distance1 <= distance2)
        {
            return distance1;
        }
        else
            return distance2;
    }
    else
    {
        int distance1 = distanceLineWithPoint(line1, line2.start);
        int distance2 = distanceLineWithPoint(line1, line2.end);
        if(distance1 <= distance2)
        {
            return distance2;
        }
        else
            return distance1;
    }
}

int distanceTwoRect(rect rec1, rect rec2)
{
    int len1 = rec1.length;
    int len2 = rec2.length;
    oneLine line1(Point(rec1.x1, rec1.y1), Point(rec1.x2, rec1.y2));
    oneLine line2(Point(rec2.x1, rec2.y1), Point(rec2.x2, rec2.y2));
    //    if(abs(rec1.theta - rec2.theta) < 20*3.14/180)
    //    {
    //        if(len1 <= len2)
    //        {
    //            int distance1 = distanceLineWithPoint(line2, Point(rec1.x, rec1.y));
    //                return distance1;
    //        }
    //        else
    //        {
    //            int distance1 = distanceLineWithPoint(line1, Point(rec1.x, rec1.y));
    //                return distance1;
    //        }
    //    }
    //    else
    //        return 1000;

    if(len1 <= len2)
    {
        int distance1 = distanceLineWithPoint(line2, line1.start);
        int distance2 = distanceLineWithPoint(line2, line1.end);
        if(distance1 <= distance2)
        {
            return distance1;
        }
        else
            return distance2;
    }
    else
    {
        int distance1 = distanceLineWithPoint(line1, line2.start);
        int distance2 = distanceLineWithPoint(line1, line2.end);
        if(distance1 <= distance2)
        {
            return distance2;
        }
        else
            return distance1;
    }
}

void lineEquation(const oneLine line, float& A, float& B, float& C)
{
    A = line.end.y - line.start.y;
    B = line.start.x - line.end.x;
    C = line.end.x * line.start.y - line.start.x * line.end.y;
    //C = (line.start.y - line.end.y)*line.start.x + (line.end.x - line.start.x)*line.start.y;
}
float distanceLineWithPoint(const oneLine line, const Point p)
{
    float a,b,c;
    lineEquation(line, a, b, c);
    return abs((float)(a*p.x + b*p.y + c))/sqrt(a*a + b*b);
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
    if (length2 <= min)
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

    linePoinits lsd_lines;
    int valid_line_num = 0;
    Mat img_64fc1;
    newImg.convertTo(img_64fc1, CV_64FC1);

    int cols  = img_64fc1.cols;
    int rows = img_64fc1.rows;

    image_double image = new_image_double(cols, rows);
    image->data = img_64fc1.ptr<double>(0);
    // ntuple_list ntl = lsd(image);//**** lsd

    vector<rect> LSD = lsd(image);
   // cv::Mat lsd_display = cv::Mat::zeros(rows, cols, CV_8UC1);
    cv::Mat lsd_display_point;
    src_raw.copyTo(lsd_display_point);

    displayRect(LSD, lsd_display_point);
    imshow("lsd",lsd_display_point);

    imgProcess(LSD, src_raw);


    //    cv::Point pt1, pt2;
    //    for (int j = 0; j != LSD.size(); ++j)
    //    {
    //        rect rect_line = LSD.at(j);
    //        pt1.x = int(rect_line.x1);
    //        pt1.y = int(rect_line.y1);
    //        pt2.x = int(rect_line.x2);
    //        pt2.y = int(rect_line.y2);
    //        int width = int(rect_line.width);

    //        double length = rect_line.length; // sqrt((pt2.x - pt1.x)*(pt2.x - pt1.x) + (pt2.y - pt1.y)*(pt2.y - pt1.y) );
    //        cout<<length<<endl;
    //        if(width >2 && length>5)
    //            // if(width >2 && length>15)
    //        {
    //            valid_line_num++;
    //            cv::line(lsd_display, pt1, pt2, cv::Scalar(255), width, CV_AA);
    //            cv::line(lsd_display_point, pt1, pt2, cv::Scalar(255), 1, CV_AA);//width

    //            oneLine  line(pt1, pt2, width, j);
    //            line.seen = false;
    //            line.count = valid_line_num;
    //            lsd_lines.lines.push_back(line);
    //            lsd_lines.count = valid_line_num;

    ////            rectangle(lsd_display_point, cvPoint(100-1,100-1),
    ////                      cvPoint(100+1,100+1),
    ////                      CV_RGB(255,0,0),5,8);       // remark x and y   -->x
    ////                                                                   // |y
    ////            rectangle(lsd_display_point, cvPoint(150-1,200-1),
    ////                      cvPoint(150+1,200+1),
    ////                      CV_RGB(255,0,0),5,8);        // remark x and y

    //            rectangle(lsd_display_point, cvPoint(line.start.x-1,line.start.y-1),
    //                      cvPoint(line.start.x+1,line.start.y+1),
    //                      CV_RGB(0,255,0),1,8);       // 画矩形点 //green
    //            rectangle(lsd_display_point, cvPoint(line.end.x-1,line.end.y-1),
    //                      cvPoint(line.end.x+1,line.end.y+1),
    //                      CV_RGB(255,0,0),1,8);       // 画矩形点 //red
    //            if(line.count > 560)  //no order
    //            cv::putText(lsd_display_point,int2str(line.count),cvPoint(line.start.x-1,line.start.y-1),
    //                        CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0) );
    //        }
    //    }
    //    //free_ntuple_list(ntl);
    //    linkPoint2(lsd_lines, src_raw);

    //    cv::imshow("lsd_display_point", lsd_display_point);
    //    imwrite("./lsd_display_point.png", lsd_display_point);


    cv::waitKey(0);
    cv::destroyAllWindows();
    return 0;
}


bool linkPoint3(linePoinits& lines, Mat img)
{
    static int replace, link;
    Mat display_test, display, result;
    img.copyTo(display_test);
    img.copyTo(display);
    img.copyTo(result);
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
            int length0 = lengthTwoPoint(curr_line.start, temp_line.start);
            int length1 = lengthTwoPoint(curr_line.start, temp_line.end);
            int length2 = lengthTwoPoint(curr_line.end, temp_line.start);
            int length3 = lengthTwoPoint(curr_line.end, temp_line.end);

            //int line4 = distanceTwoLine(curr_line, temp_line);
            int length = findMiniLength(length0, length1, length2, length3, state);

            if(length < min_distance)
            {
                min_distance = length;
                min_count = j;
                lines.lines.at(min_count).seen = true;
            }
        }

        if(min_distance < 30 )
        {
            cout<<" min_distance "<<min_distance<<endl;
            oneLine min_line = lines.lines.at(min_count);

            Point  orient1 = findOrient(curr_line.start, curr_line.end);
            Point  orient2 = findOrient(min_line.start,  min_line.end);

            float curr_length = sqrt(orient1.x*orient1.x + orient1.y*orient1.y);
            float min_length = sqrt(orient2.x*orient2.x + orient2.y*orient2.y);

            float cost_value = (float)(orient1.x*orient2.x + orient1.y*orient2.y)/(curr_length*min_length);
            if(cost_value < -1 && cost_value > -2) cost_value = -1;
            else if(cost_value > 1 && cost_value <2)  cost_value = 1;

            float angle = acos(cost_value)*180.0/M_PI;//PI; //
            //float angle = findAngle(curr_line, min_line);
            cout<<"angle"<<angle<<endl;

            cout<<"horiental process"<<endl;
            switch(state)
            {
            case 0:
            {
                if (abs(angle ) < 20.0 | abs(angle - 180) < 20.0 )//horiental
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.end);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced
                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                    else if(lengthEndPoint < curr_length | lengthEndPoint < min_length) //replace another
                    {
                        replace++;
                        cout<<"horiental line : replace"<<endl;
                        if(curr_length < min_length)
                        {
                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced
                            lines.lines.at(i).end = lines.lines.at(min_count).end;

                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                        else
                        {
                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                            lines.lines.at(min_count).end = lines.lines.at(i).end;

                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                    }
                }
                else if(min_distance < 12)
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.end);
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced

                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                }
                break;
            }
            case 1:
            {
                if (abs(angle ) < 20.0 | abs(angle - 180) < 20.0 )//horiental
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.start);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).start = lines.lines.at(min_count).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).end = lines.lines.at(i).start; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                    else if(lengthEndPoint < curr_length | lengthEndPoint < min_length) //replace another
                    {
                        replace++;
                        cout<<"horiental line : replace"<<endl;
                        if(curr_length < min_length)
                        {
                            //cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(i).start = lines.lines.at(min_count).end; // be replaced
                            lines.lines.at(i).end = lines.lines.at(min_count).start;

                            // cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                        else
                        {
                            // cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(min_count).start = lines.lines.at(i).end; // be replaced
                            lines.lines.at(min_count).end = lines.lines.at(i).start;

                            // cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                    }
                }
                else if(min_distance < 12)
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.end, min_line.start);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).start = lines.lines.at(min_count).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).end = lines.lines.at(i).start; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                }
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end ;
                //  endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                //cout<< " "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end<<endl;
                break;
            }
            case 2:
            {
                if (abs(angle ) < 20.0 | abs(angle - 180) < 20.0 )//horiental
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.end);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).end = lines.lines.at(min_count).start; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).start = lines.lines.at(i).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                    else if(lengthEndPoint < curr_length | lengthEndPoint < min_length) //replace another
                    {
                        replace++;
                        cout<<"horiental line : replace"<<endl;
                        if(curr_length < min_length)
                        {
                            //cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(i).start = lines.lines.at(min_count).end; // be replaced
                            lines.lines.at(i).end = lines.lines.at(min_count).start;

                            // cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                        else
                        {
                            // cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(min_count).start = lines.lines.at(i).end; // be replaced
                            lines.lines.at(min_count).end = lines.lines.at(i).start;

                            // cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                    }
                }
                else if(min_distance < 12)
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.end);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).end = lines.lines.at(min_count).start; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).start = lines.lines.at(i).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                }
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end ;
                // endToStart(lines.lines.at(i).start, lines.lines.at(i).end);
                //cout<< " "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end<<endl;
                break;
            }
            case 3:
            {
                if (abs(angle ) < 20.0 | abs(angle - 180) < 20.0 )//horiental
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.start);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).end = lines.lines.at(min_count).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).end = lines.lines.at(i).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }
                    else if(lengthEndPoint < curr_length | lengthEndPoint < min_length) //replace another
                    {
                        replace++;
                        cout<<"horiental line : replace"<<endl;
                        if(curr_length < min_length)
                        {
                            //cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(i).start = lines.lines.at(min_count).start; // be replaced
                            lines.lines.at(i).end = lines.lines.at(min_count).end;

                            // cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                        else
                        {
                            // cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end,CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)
                            lines.lines.at(min_count).start = lines.lines.at(i).start; // be replaced
                            lines.lines.at(min_count).end = lines.lines.at(i).end;

                            // cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(255,0,0),1,CV_AA);//lines.width.at(j)

                        }
                    }
                }
                else if(min_distance < 12)
                {
                    int lengthEndPoint = lengthTwoPoint(curr_line.start, min_line.start);
                    //int length_another = sqrt((lines.start.at(i).x - lines.end.at(min_count).x)*(lines.start.at(i).x - lines.end.at(min_count).x) + (lines.start.at(i).y - lines.end.at(min_count).y)*(lines.start.at(i).y - lines.end.at(min_count).y));
                    if(lengthEndPoint >=  curr_length &&  lengthEndPoint>=min_length  ) // middle point ==> link
                    {
                        link++;
                        if(curr_length < min_length)
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(i).end = lines.lines.at(min_count).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }
                        else
                        {
                            //                            cv::line(display_test, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display_test, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)

                            lines.lines.at(min_count).end = lines.lines.at(i).end; // be replaced
                            //                            cv::line(display, lines.lines.at(min_count).start, lines.lines.at(min_count).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                            //                            cv::line(display, lines.lines.at(i).start, lines.lines.at(i).end, CV_RGB(0,255,0),1,CV_AA);//lines.width.at(j)
                        }

                    }

                }
                //cout<<" state (before after ): "<<state<<" "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end ;
                //cout<<" "<<state<<" "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end ;

                //endToStart(lines.lines.at(min_count).start, lines.lines.at(min_count).end);
                //endToStart(lines.lines.at(i).start, lines.lines.at(i).end);

                //cout<< " "<<lines.lines.at(min_count).start<<" "<<lines.lines.at(min_count).end;
                //cout<< " "<<lines.lines.at(i).start<<" "<<lines.lines.at(i).end<<endl;
                break;
            default :
                    break;
                }
            }

        }
    }
    cout<<" link "<<link<<" replace "<<replace<< endl;
    showLinkImg(lines, result);
    imshow("display_test",display_test);
    imwrite("./linkPoint.png", display_test);
    imwrite("./result.png", result);
    imwrite("./display.png", display);
    imshow("display",display);
}

string int2str(int num)
{
    std::stringstream ss;
    ss << num;
    return ss.str();
}
