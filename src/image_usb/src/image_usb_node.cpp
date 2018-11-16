#include "ros/ros.h"
#include "DJI_guidance.h"
#include "DJI_utility.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <stdio.h>
#include <string.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH  320
#define IMAGE_SIZE ( IMAGE_HEIGHT * IMAGE_WIDTH)

using namespace cv;

e_vbus_index CAMERA_ID = e_vbus5;
e_image_data_frequecy CAMERA_FEQ = e_frequecy_20;
DJI_event g_event;
DJI_lock  g_lock;


Mat greyImageLeft(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
Mat greyImageRight(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);

ros::Publisher leftImagePub;
ros::Publisher rightImagePub;




int grey_image_callback( int eventType, int dataLen, char *dataRev)
{
    g_lock.enter();

    if ( eventType == e_image && dataRev != NULL )
    {
        
        image_data* data = (image_data*)dataRev;
        if ( data->m_greyscale_image_left[CAMERA_ID] )
        {
            memcpy(greyImageLeft.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
            imshow("left", greyImageLeft);

            cv_bridge::CvImage leftImage;
            greyImageLeft.copyTo(leftImage.image);;
            leftImage.header.frame_id = "guidance";
            leftImage.header.stamp = ros::Time::now();
            leftImage.encoding = sensor_msgs::image_encodings::MONO8;
            leftImagePub.publish(leftImage.toImageMsg());
        }

        if ( data->m_greyscale_image_right[CAMERA_ID] )
        {
            memcpy(greyImageRight.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
            imshow("right", greyImageRight);

            cv_bridge::CvImage rightImage;
            greyImageLeft.copyTo(rightImage.image);;
            rightImage.header.frame_id = "guidance";
            rightImage.header.stamp = ros::Time::now();
            rightImage.encoding = sensor_msgs::image_encodings::MONO8;
            rightImagePub.publish(rightImage.toImageMsg());
        }
        waitKey(1);
    }

    g_lock.leave();
    g_event.set_event();
    
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_usb_node");
    ros::NodeHandle n;

    leftImagePub = n.advertise<sensor_msgs::Image>("cam0/image_raw", 1);
    rightImagePub = n.advertise<sensor_msgs::Image>("cam1/image_raw", 1);

    reset_config();                           //清除之前订阅的所有数据及信息
    static int errCode  = init_transfer();    //等到板子校验完成并完成发送线程初始化
    printf("初始化摄像头：%d\n", errCode);

    set_image_frequecy( CAMERA_FEQ );
    errCode = select_greyscale_image( CAMERA_ID, true ); //订阅左摄像头图像
    printf("订阅左摄像头灰度图：%d\n", errCode);

    errCode = select_greyscale_image( CAMERA_ID, false ); //订阅右摄像头图像
    printf("订阅右摄像头灰度图：%d\n", errCode);

    errCode = set_sdk_event_handler( grey_image_callback );
    printf("设置回调函数：%d\n", errCode);

    errCode = start_transfer();
    printf("开始发送数据：%d\n", errCode);

    while( ros::ok() )
    {
        g_event.wait_event();
        ros::spinOnce();
    }

    errCode = stop_transfer();
    sleep(1000000);
    errCode = release_transfer();
}
