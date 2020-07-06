#ifndef QRCODELOCATE_H
#define QRCODELOCATE_H
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <opencv2/imgproc/types_c.h>
#include <QMessageBox>
#include <QString>
#include <QDebug>

using namespace std;
using namespace zbar;  //添加zbar名称空间

class QRCodeLocater
{
public:
    QRCodeLocater();
    bool locate_recognize(std::string file_path,std::string &code_type,std::string &code_data);
    ~QRCodeLocater(){}
private:

    cv::Mat src;
    cv::Mat src_gray;
    cv::RNG rng;

    cv::Point Center_cal(vector<vector<cv::Point> > contours, int i);
    bool QRCode_Recognize(cv::Mat &image, std::string &code_type,std::string &code_data);

};

#endif // QRCODELOCATE_H
