//
// Created by adalberto-oliveira on 13/12/22.
//

#include "../include/usb_cam_lib.h"

using namespace std;


struct enchantedImage{
    cv::Mat img;
    double beta;
    double alpha;
};

typedef enchantedImage Image_t;

Image_t auto_enchant_img(cv::Mat &img){
    cv::Mat gray_image;
    cv::cvtColor(img, gray_image, cv::COLOR_RGB2GRAY);

    int histSize = 256;
    float range[] = { 0, 256 }; //the upper boundary is exclusive
    const float* histRange[] = { range };

    bool uniform = true, accumulate = false;
    cv::Mat hist;

    cv::calcHist( &gray_image, 1, nullptr, cv::Mat(), hist, 1, &histSize, histRange, uniform, accumulate );

    std::vector<float> accumulator = {hist.at<float>(0, 0)};

    for(int i = 1; i<hist.rows; i++) {
        accumulator.push_back(accumulator.at(i - 1) + hist.at<float>(i, 0) / 3);
    }

    float max = accumulator[accumulator.size() - 1];

    double clip_hist_percent = max / 100;

    clip_hist_percent /= 2.0;

    int minimum_gray = 0;

    while(accumulator.at(minimum_gray) < clip_hist_percent){
        minimum_gray += 5;
    }

    int maximum_gray = histSize - 1;

    while(accumulator.at(maximum_gray) >= (max - clip_hist_percent)){
        maximum_gray -= 1;
    }

    double alpha = 255.0 / (maximum_gray - minimum_gray);
    double beta = -minimum_gray * alpha;

    cv::Mat img_enchanted;
    cv::convertScaleAbs(img, img_enchanted, alpha=alpha, beta=beta);

    Image_t result;
    result.img = img_enchanted;
    result.alpha = alpha;
    result.beta = beta;

    return result;
}

int main(int argc, char** argv)
{
    // Receiving video param from argv
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    // Starting the image processors
    image_transport::ImageTransport it(nh);
    image_transport::Publisher imgRawPub = it.advertise("image_raw", 1);

    // Creating the video object
    int videoSource;
    int frame_width;
    int frame_height;
    bool showImg;

    ros::param::get("usb_cam/video/video_source",videoSource);
    ros::param::get("usb_cam/video/frame_width",frame_width);
    ros::param::get("usb_cam/video/frame_height",frame_height);
    cv::VideoCapture cap(videoSource);

    int rateHz;
    ros::param::get("usb_cam/control/rate",rateHz);
    ros::param::get("usb_cam/control/show_img",showImg);
    ros::Rate loop_rate(rateHz);



    // Check if video device can be opened with the given index
    cap.set(cv::CAP_PROP_FRAME_WIDTH,frame_width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT,frame_height);
    if(!cap.isOpened()) return 0;
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    cout << "Camera node is ready to use!\nUsing Video " << argv[1] << endl;
    while (nh.ok())
    {
        cap >> frame;

        Image_t enchantedStruct = auto_enchant_img(frame);

        std::cout<<"Beta: "<<enchantedStruct.beta<<"\tAlpha: "<<enchantedStruct.alpha<<std::endl;

        if(!frame.empty())
        {
            try
            {
                msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", enchantedStruct.img).toImageMsg();
            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("cv_bridge exception: %s", e.what());
            }
        }

        imgRawPub.publish(msg);
        if (showImg)
        {
            cv::imshow("Raw Image",frame);
            cv::imshow("Processed Img",enchantedStruct.img);
            cv::waitKey(1);

        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
