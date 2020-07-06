#include <pcl/apps/point_cloud_editor/qrcodelocate.h>
#include <QObject>

bool
QRCodeLocater::QRCode_Recognize(cv::Mat &image, std::string &code_type,std::string &code_data)
{
    ImageScanner scanner;
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);
    // cv::Mat image = cv::imread(s);
    cv::Mat imageGray;
    cv::cvtColor(image,imageGray,CV_RGB2GRAY);
    int width = imageGray.cols;
    int height = imageGray.rows;
    auto *raw = imageGray.data;
    Image imageZbar(static_cast<unsigned int>(width), static_cast<unsigned int>(height), "Y800", raw,
                    static_cast<unsigned long>(width * height));
    scanner.scan(imageZbar); //扫描条码
    Image::SymbolIterator symbol = imageZbar.symbol_begin();
    if(imageZbar.symbol_begin()==imageZbar.symbol_end())
    {
        return false;
    }
    for(;symbol != imageZbar.symbol_end();++symbol)
    {
        std::cout<<symbol->get_type_name()<<"数据是"<<symbol->get_data()<<std::endl;
        code_type=symbol->get_type_name();
        code_data=symbol->get_data();
        return true;
    }
}

cv::Point
QRCodeLocater::Center_cal(vector<vector<cv::Point> > contours, int i)
{
    int centerx = 0, centery = 0, n = static_cast<int>(contours[i].size());
    //在提取的小正方形的边界上每隔周长个像素提取一个点的坐标，
    //求所提取四个点的平均坐标（即为小正方形的大致中心）
    centerx = (contours[i][n / 4].x + contours[i][n * 2 / 4].x + contours[i][3 * n / 4].x + contours[i][n - 1].x) / 4;
    centery = (contours[i][n / 4].y + contours[i][n * 2 / 4].y + contours[i][3 * n / 4].y + contours[i][n - 1].y) / 4;
    cv::Point point1 =cv::Point(centerx, centery);
    return point1;
}

QRCodeLocater::QRCodeLocater(){}


bool
QRCodeLocater::locate_recognize(std::string path,std::string &code_type,std::string &code_data)
{
    rng=cv::RNG(123456);
    src = cv::imread(path);
    cv::Mat src_all = src.clone();

    qDebug()<<"彩色图转灰色图";
    //彩色图转灰度图
    cv::cvtColor(src, src_gray, CV_BGR2GRAY);
    qDebug()<<"对图像进行平滑处理";
    //对图像进行平滑处理
    cv::blur(src_gray, src_gray, cv::Size(3, 3));
    qDebug()<<"使灰度图象直方图均衡化";
    //使灰度图象直方图均衡化
    cv::equalizeHist(src_gray, src_gray);
    //    cv::namedWindow("src_gray");
    //    cv::imshow("src_gray", src_gray);


    cv::Scalar color = cv::Scalar(1, 1, 255);
    cv::Mat threshold_output;
    vector<vector<cv::Point> > contours, contours2;
    vector<cv::Vec4i> hierarchy;
    cv::Mat drawing = cv::Mat::zeros(src.size(), CV_8UC3);
    cv::Mat drawing2 = cv::Mat::zeros(src.size(), CV_8UC3);
    cv::Mat drawingAllContours = cv::Mat::zeros(src.size(), CV_8UC3);

    qDebug()<<"指定112阀值进行二值化";
    //指定112阀值进行二值化
    cv::threshold(src_gray, threshold_output, 112, 255, cv::THRESH_BINARY);

    //    cv::namedWindow("Threshold_output");
    //    cv::imshow("Threshold_output", threshold_output);


    /*查找轮廓
         *  参数说明
            输入图像image必须为一个2值单通道图像
            contours参数为检测的轮廓数组，每一个轮廓用一个point类型的vector表示
            hiararchy参数和轮廓个数相同，每个轮廓contours[ i ]对应4个hierarchy元素hierarchy[ i ][ 0 ] ~hierarchy[ i ][ 3 ]，
                分别表示后一个轮廓、前一个轮廓、父轮廓、内嵌轮廓的索引编号，如果没有对应项，该值设置为负数。
            mode表示轮廓的检索模式
                CV_RETR_EXTERNAL 表示只检测外轮廓
                CV_RETR_LIST 检测的轮廓不建立等级关系
                CV_RETR_CCOMP 建立两个等级的轮廓，上面的一层为外边界，里面的一层为内孔的边界信息。如果内孔内还有一个连通物体，这个物体的边界也在顶层。
                CV_RETR_TREE 建立一个等级树结构的轮廓。具体参考contours.c这个demo
            method为轮廓的近似办法
                CV_CHAIN_APPROX_NONE 存储所有的轮廓点，相邻的两个点的像素位置差不超过1，即max（abs（x1-x2），abs（y2-y1））==1
                CV_CHAIN_APPROX_SIMPLE 压缩水平方向，垂直方向，对角线方向的元素，只保留该方向的终点坐标，例如一个矩形轮廓只需4个点来保存轮廓信息
                CV_CHAIN_APPROX_TC89_L1，CV_CHAIN_APPROX_TC89_KCOS 使用teh-Chinl chain 近似算法
            offset表示代表轮廓点的偏移量，可以设置为任意值。对ROI图像中找出的轮廓，并要在整个图像中进行分析时，这个参数还是很有用的。
         */
    cv::findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, cv::CHAIN_APPROX_NONE, cv::Point(0, 0));

    int c = 0, ic = 0, k = 0, area = 0;

    qDebug()<<"通过黑色定位角作为父轮廓"<<endl;
    //通过黑色定位角作为父轮廓，有两个子轮廓的特点，筛选出三个定位角
    int parentIdx = -1;
    for (int i = 0; i < contours.size(); i++) {
        //画出所以轮廓图
        cv::drawContours(drawingAllContours, contours, parentIdx, CV_RGB(255, 255, 255), 1, 8);
        if (hierarchy[i][2] != -1 && ic == 0) {
            parentIdx = i;
            ic++;
        } else if (hierarchy[i][2] != -1) {
            ic++;
        } else if (hierarchy[i][2] == -1) {
            ic = 0;
            parentIdx = -1;
        }
        //有两个子轮廓
        if (ic >= 2) {
            //保存找到的三个黑色定位角
            contours2.push_back(contours[parentIdx]);
            //画出三个黑色定位角的轮廓
            cv::drawContours(drawing, contours, parentIdx,
                             CV_RGB(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255)), 1, 8);
            ic = 0;
            parentIdx = -1;
        }
    }
    //填充的方式画出三个黑色定位角的轮廓
    for (int i = 0; i < contours2.size(); i++)
        cv::drawContours(drawing2, contours2, i,
                         CV_RGB(rng.uniform(100, 255), rng.uniform(100, 255), rng.uniform(100, 255)), -1, 4,
                         hierarchy[k][2], 0, cv::Point());

    //获取三个定位角的中心坐标
    cv::Point point[3];
    for (int i = 0; i < contours2.size(); i++) {
        point[i] = Center_cal(contours2, i);
    }
    qDebug()<<"计算轮廓的面积，计算定位角的面积，从而计算出边长";
    //计算轮廓的面积，计算定位角的面积，从而计算出边长
    if(contours2.size()>=3	){
        area = static_cast<int>(contourArea(contours2[1]));
        int area_side = cvRound(sqrt(double(area)));
        for (int i = 0; i < contours2.size(); i++) {
            //画出三个定位角的中心连线
            cv::line(drawing2, point[i % contours2.size()], point[(i + 1) % contours2.size()], color, area_side / 2, 8);
        }
    }


    //    cv::namedWindow("DrawingAllContours");
    //    cv::imshow("DrawingAllContours", drawingAllContours);

    //    cv::namedWindow("Drawing2");
    //    cv::imshow("Drawing2", drawing2);

    //    cv::namedWindow("Drawing");
    //    cv::imshow("Drawing", drawing);

    qDebug()<<"接下来要框出这整个二维码";
    //接下来要框出这整个二维码
    cv::Mat gray_all, threshold_output_all;
    vector<vector<cv::Point> > contours_all;
    vector<cv::Vec4i> hierarchy_all;
    cv::cvtColor(drawing2, gray_all, CV_BGR2GRAY);


    cv::threshold(gray_all, threshold_output_all, 45, 255, cv::THRESH_BINARY);
    cv::findContours(threshold_output_all, contours_all, hierarchy_all, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE,
                     cv::Point(0, 0));//RETR_EXTERNAL表示只寻找最外层轮廓


    cv::Point2f fourPoint2f[4];
    //接下来要框出这整个二维码
    qDebug()<<"接下来要框出这整个二维码";
    qDebug()<<contours_all.size();
    if(contours_all.size()>0){
        cv::RotatedRect rectPoint = cv::minAreaRect(contours_all[0]);
        cv::Rect myRect = cv::boundingRect(contours_all[0]);
        //将rectPoint变量中存储的坐标值放到 fourPoint的数组中
        rectPoint.points(fourPoint2f);

        qDebug()<<"开始循环";
        for (int i = 0; i < 4; i++) {
            cv::line(src_all, fourPoint2f[i % 4], fourPoint2f[(i + 1) % 4], cv::Scalar(20, 21, 237), 1);
        }

        cv::namedWindow("Src_all");
        //   cv::setWindowProperty("Src_all", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);
        //cv::setWindowProperty("Src_all", CV_WND_PROP_FULLSCREEN, CV_WINDOW_NORMAL);
        cv::imshow("Src_all", src_all);
        qDebug()<<"存储完毕";
        char resultFileNameSring[100];
        sprintf(resultFileNameSring, "QRCode_Locate_result.png");

        cv::Mat resultImage = cv::Mat(src_all, myRect);
        //cv::imwrite(resultFileNameSring, resultImage);

        //框出二维码后，就可以提取出二维码，然后使用解码库zxing，解出码的信息。
        //或者研究二维码的排布规则，自己写解码部分
        return QRCode_Recognize(resultImage,code_type,code_data);
    }
    else
        return false;
}
