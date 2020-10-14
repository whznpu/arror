#include<opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include<boost/lexical_cast.hpp>

#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include<string>



using namespace cv;
using namespace std;
using namespace boost::asio;


#define T_ANGLE_THRE 10
#define T_SIZE_THRE 5

void brightAdjust(Mat src, Mat dst, double dContrast, double dBright); //亮度调节函数
void getDiffImage(Mat src1, Mat src2, Mat dst, int nThre); //二值化
vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse,Mat src); //检测装甲
void drawBox(RotatedRect box, Mat img); //标记装甲
void send_center(Point2f point);//发送坐标
void send_center(Point2f point)
{
    io_service iosev;
    //python virtual com
        //cout<< "start to set port No" << endl;

        serial_port sp(iosev,"/dev/pts/2");

       //cout << "port created "<< endl;

         //设置参数

       sp.set_option(serial_port::baud_rate(115200));

       sp.set_option(serial_port::flow_control(serial_port::flow_control::none));

       sp.set_option(serial_port::parity(serial_port::parity::none));

       sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));

       sp.set_option(serial_port::character_size(8));

         //cout<< "port parameter setup" << endl;

       // 向串口写数据
       
       string x = boost::lexical_cast <string> (point.x);
       string y=boost::lexical_cast<string> (point.y);
       string place=x+" "+y;
       //cout<<point.x<<" "<<point.y<<endl;
       //cout<<x<<"   "<<y<<endl;

       write(sp, buffer(place, place.size()));
}


 Mat frame0;
 Point2f center_point;
int main()
{

    VideoCapture cap0("1.mp4");


    Size imgSize;
    RotatedRect s;   //定义旋转矩形
    vector<RotatedRect> vEllipse; //定以旋转矩形的向量，用于存储发现的目标区域
    vector<RotatedRect> vRlt;
    vector<RotatedRect> vArmor;
    bool bFlag = false;

    vector<vector<Point> > contour;

    cap0 >> frame0;
    imgSize = frame0.size();

    Mat rawImg = Mat(imgSize, CV_8UC3);

    Mat grayImage = Mat(imgSize, CV_8UC1);
    Mat rImage = Mat(imgSize, CV_8UC1);
    Mat gImage = Mat(imgSize, CV_8UC1);
    Mat bImage = Mat(imgSize, CV_8UC1);
    Mat binary = Mat(imgSize, CV_8UC1);
    Mat rlt = Mat(imgSize, CV_8UC1);
    namedWindow("Raw");


     double fps = cap0.get(CV_CAP_PROP_FPS); 
    while (1)
    {
        if (cap0.read(frame0))
        {
            brightAdjust(frame0, rawImg, 1, -120);  //每个像素每个通道的值都减去120
            //namedWindow("亮度调节",WINDOW_AUTOSIZE);
            //imshow("亮度调节",rawImg);
            Mat bgr[3];
            split(rawImg, bgr); //将三个通道的像素值分离
            bImage = bgr[0];
            gImage = bgr[1];
            rImage = bgr[2];
          //如果像素R值-G值大于25，则返回的二值图像的值为255，否则为0
            getDiffImage(rImage, gImage, binary, 25); 
            namedWindow("二值化",WINDOW_AUTOSIZE);
            imshow("二值化",binary);

            dilate(binary, grayImage, Mat(), Point(-1,-1), 1);   //图像膨胀
            namedWindow("膨胀",WINDOW_AUTOSIZE);
            imshow("膨胀",grayImage);
            erode(grayImage, rlt, Mat(), Point(-1,-1), 1);  //图像腐蚀，先膨胀在腐蚀属于闭运算
            namedWindow("腐蚀",WINDOW_AUTOSIZE);
            imshow("腐蚀",rlt);
            findContours(rlt, contour, RETR_CCOMP , CHAIN_APPROX_SIMPLE); //在二值图像中寻找轮廓


            for (int i=0; i<contour.size(); i++)
            {
                if (contour[i].size()> 10)  //判断当前轮廓是否大于10个像素点
                {
                    bFlag = true;   //如果大于10个，则检测到目标区域
                  //拟合目标区域成为椭圆，返回一个旋转矩形（中心、角度、尺寸）
                    s = fitEllipse(contour[i]);  
                    for (int nI = 0; nI < 5; nI++)
                    {
                        for (int nJ = 0; nJ < 5; nJ++)  //遍历以旋转矩形中心点为中心的5*5的像素块
                        {
                            if (s.center.y - 2 + nJ > 0 && s.center.y - 2 + nJ < 480 && s.center.x - 2 + nI > 0 && s.center.x - 2 + nI <  640)  //判断该像素是否在有效的位置
                            {   
                                Vec3b v3b = frame0.at<Vec3b>((int)(s.center.y - 2 + nJ), (int)(s.center.x - 2 + nI)); //获取遍历点点像素值
                               //判断中心点是否接近白色
                                if (v3b[0] < 200 || v3b[1] < 200 || v3b[2] < 200)
                                    bFlag = false;        //如果中心不是白色，则不是目标区域
                            }
                        }
                    }
                    if (bFlag)
                    {
                        vEllipse.push_back(s); //将发现的目标保存
                    }
                }
            
                //drawBox(s,frame0);

            }
        //调用子程序，在输入的LED所在旋转矩形的vector中找出装甲的位置，并包装成旋转矩形，存入vector并返回
            vRlt = armorDetect(vEllipse,frame0); 
            for (unsigned int nI = 0; nI < vRlt.size(); nI++) //在当前图像中标出装甲的位置
                drawBox(vRlt[nI], frame0);
            imshow("Raw", frame0);
            std::thread t2(send_center,center_point);
            t2.detach();

          
            vEllipse.clear();
            vRlt.clear();
            vArmor.clear();
            
        }
        else
        {
            break;
        }
        
        
         char c=waitKey(1000/fps);
                  if (c == 27) {
            break;}
            
    
    }
    
    cap0.release();
    return 0;


}

void brightAdjust(Mat src, Mat dst, double dContrast, double dBright)
{
    int nVal;


    for (int nI = 0; nI<src.rows; nI++)
    {
        Vec3b* p1 = src.ptr<Vec3b>(nI);
        Vec3b* p2 = dst.ptr<Vec3b>(nI);
        for (int nJ = 0; nJ <src.cols; nJ++)
        {
            for (int nK = 0; nK < 3; nK++)
            {
               //每个像素的每个通道的值都进行线性变换
                nVal = (int)(dContrast * p1[nJ][nK] + dBright);
                if (nVal < 0)
                    nVal = 0;
                if (nVal > 255)
                    nVal = 255;
                p2[nJ][nK] = nVal;
            }
        }
    }
}

void getDiffImage(Mat src1, Mat src2, Mat dst, int nThre)
{


    for (int nI = 0; nI<src1.rows; nI++)
    {
        uchar* pchar1 = src1.ptr<uchar>(nI);
        uchar* pchar2 = src2.ptr<uchar>(nI);
        uchar* pchar3 = dst.ptr<uchar>(nI);
        for (int nJ = 0; nJ <src1.cols; nJ++)
        {
            if (pchar1[nJ] - pchar2[nJ]> nThre) //
            {
                pchar3[nJ] = 255;
            }
            else
            {
                pchar3[nJ] = 0;
            }
        }
    }
}

vector<RotatedRect> armorDetect(vector<RotatedRect> vEllipse,cv::Mat src)
{
    vector<RotatedRect> vRlt;
    RotatedRect armor; //定义装甲区域的旋转矩形
    int nL, nW;
    double dAngle;
    vRlt.clear();
    if (vEllipse.size() < 2) //如果检测到的旋转矩形个数小于2，则直接返回
        return vRlt;
    for (unsigned int nI = 0; nI < vEllipse.size() - 1; nI++) //求任意两个旋转矩形的夹角
    {
        for (unsigned int nJ = nI + 1; nJ < vEllipse.size(); nJ++)
        {
            dAngle = abs(vEllipse[nI].angle - vEllipse[nJ].angle);
            while (dAngle > 180)
                dAngle -= 180;
          //判断这两个旋转矩形是否是一个装甲的两个LED等条
            if ((dAngle < T_ANGLE_THRE || 180 - dAngle < T_ANGLE_THRE) && abs(vEllipse[nI].size.height - vEllipse[nJ].size.height) < (vEllipse[nI].size.height + vEllipse[nJ].size.height) / T_SIZE_THRE && abs(vEllipse[nI].size.width - vEllipse[nJ].size.width) < (vEllipse[nI].size.width + vEllipse[nJ].size.width) / T_SIZE_THRE) 
            {
                armor.center.x = (vEllipse[nI].center.x + vEllipse[nJ].center.x) / 2; //装甲中心的x坐标 
                armor.center.y = (vEllipse[nI].center.y + vEllipse[nJ].center.y) / 2; //装甲中心的y坐标
                //把点存进去
                //cout<<armor.center.x<<" "<<armor.center.y<<endl;
               center_point.x=armor.center.x;
               center_point.y=armor.center.y;
                
                circle(src,Point(armor.center.x,armor.center.y),3,Scalar(0,0,255),-1,8);//绘制圆心坐标
                armor.angle = (vEllipse[nI].angle + vEllipse[nJ].angle) / 2;   //装甲所在旋转矩形的旋转角度
                if (180 - dAngle < T_ANGLE_THRE)
                    armor.angle += 90;
                nL = (vEllipse[nI].size.height + vEllipse[nJ].size.height) / 2; //装甲的高度
                nW = sqrt((vEllipse[nI].center.x - vEllipse[nJ].center.x) * (vEllipse[nI].center.x - vEllipse[nJ].center.x) + (vEllipse[nI].center.y - vEllipse[nJ].center.y) * (vEllipse[nI].center.y - vEllipse[nJ].center.y)); //装甲的宽度等于两侧LED所在旋转矩形中心坐标的距离
                if (nL < nW)
                {
                    armor.size.height = nL;
                    armor.size.width = nW;
                }
                else
                {
                    armor.size.height = nW;
                    armor.size.width = nL;
                }
                vRlt.push_back(armor); //将找出的装甲的旋转矩形保存到vector
            }
        }
    }
    return vRlt;
}

void drawBox(RotatedRect box, Mat img)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }
    box.points(pt); //计算二维盒子顶点 
    line(img, pt[0], pt[1], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 0, 255), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 0, 255), 2, 8, 0);
}