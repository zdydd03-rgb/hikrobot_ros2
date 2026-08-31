#include "../common/hik_camera.h"

#include <stdexcept>

using namespace std;

class ImagePublish : public rclcpp::Node
{
public:
    ImagePublish() : Node("hik_camera_image_publisher"),strSerialNumber("None"),nSeq(0)
    {
        this->declare_parameter<std::string>("cam_sn", "None");
        this->declare_parameter<std::string>("aligned_depth_topic", "/camera/aligned_depth");
        this->declare_parameter<std::string>("rgb_frame_id", "camera_rgb_optical_frame");
        this->declare_parameter<std::string>("depth_frame_id", "camera_depth_optical_frame");
        this->declare_parameter<std::string>("left_ir_frame_id", "camera_left_ir_optical_frame");
        this->declare_parameter<std::string>("right_ir_frame_id", "camera_right_ir_optical_frame");

        this->get_parameter("aligned_depth_topic", alignedDepthTopic);
        this->get_parameter("rgb_frame_id", rgbFrameId);
        this->get_parameter("depth_frame_id", depthFrameId);
        this->get_parameter("left_ir_frame_id", leftIrFrameId);
        this->get_parameter("right_ir_frame_id", rightIrFrameId);

        depthPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/depth", 1000);
        rgbPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rgb", 1000);
        alignedDepthPub = this->create_publisher<sensor_msgs::msg::Image>(alignedDepthTopic, 1000);
        leftIrPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/leftIr", 1000);
        rightIrPub = this->create_publisher<sensor_msgs::msg::Image>("/camera/rightIr", 1000);
        depthCamInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/depthCamInfo", 1000);
        rgbCamInfoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/rgbCamInfo", 1000);   
        // 获取序列号
        this->get_parameter("cam_sn", strSerialNumber);
        camera.initialize();
        if(strSerialNumber == "None")
        {
            // 默认连接第一个相机
            camera.connect();
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "---------------- Connect by serial number: %s ----------------", strSerialNumber.c_str());
            // 根据序列号进行连接
            camera.connect(strSerialNumber.c_str());
        }

        // 显式关闭SDK内部对齐，并读取SDK标定参数供同帧Depth->RGB转换使用。
        const int alignmentRet = camera.configureDepthToRgbAlignment();
        if(MV3D_RGBD_OK != alignmentRet)
        {
            RCLCPP_FATAL(this->get_logger(),
                "Failed to configure Depth-to-RGB alignment, status=0x%08x",
                static_cast<unsigned int>(alignmentRet));
            throw std::runtime_error("Depth-to-RGB alignment configuration failed");
        }
        RCLCPP_INFO(this->get_logger(), "Camera SN: %s", camera.getSerialNumber().c_str());
        RCLCPP_INFO(this->get_logger(), "ImageAlign: %lld",
            static_cast<long long>(camera.getImageAlignValue()));
        RCLCPP_INFO(this->get_logger(), "ZUnit: %.9g", static_cast<double>(camera.getZUnit()));
        RCLCPP_INFO(this->get_logger(), "RGB intrinsic loaded");
        RCLCPP_INFO(this->get_logger(), "Depth intrinsic loaded");
        RCLCPP_INFO(this->get_logger(), "Depth2RGB extrinsic loaded");
        RCLCPP_INFO(this->get_logger(), "Aligned depth topic: %s", alignedDepthTopic.c_str());
        RCLCPP_INFO(this->get_logger(), "RGB frame_id: %s", rgbFrameId.c_str());
        RCLCPP_INFO(this->get_logger(), "Depth frame_id: %s", depthFrameId.c_str());

        // 获取标定信息
        stCamInfo = camera.getCamInfo();
        // 开始取流
        camera.start();
        timer = this->create_wall_timer(30ms, std::bind(&ImagePublish::publishImage, this));
    }
    
    HikCamera camera;
private:
    void publishImage()
    {
        // 获取图像指针信息
        vector<sensor_msgs::msg::Image::Ptr> stImagePtr = camera.getImage();
        // 获取图像类型信息
        uint32_t nImgType = camera.getMsgImgType();
        if(0 == nImgType)
        {
            return;
        }

        // 同一次FetchFrame产生的所有消息只取一次ROS时间，保证stamp完全一致。
        const auto frameStamp = this->now();
        // 发布节点
        if((nImgType & TYPE_DEPTH) && stImagePtr[CAMERA_IMAGE_DEPTH])
        {
            stImagePtr[CAMERA_IMAGE_DEPTH]->header.stamp = frameStamp;
            stImagePtr[CAMERA_IMAGE_DEPTH]->header.frame_id = depthFrameId;
            depthPub->publish(*stImagePtr[CAMERA_IMAGE_DEPTH]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send depth image [%u] success", nSeq);
        }
        if((nImgType & TYPE_RGB) && stImagePtr[CAMERA_IMAGE_RGB])
        {
            stImagePtr[CAMERA_IMAGE_RGB]->header.stamp = frameStamp;
            stImagePtr[CAMERA_IMAGE_RGB]->header.frame_id = rgbFrameId;
            rgbPub->publish(*stImagePtr[CAMERA_IMAGE_RGB]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send rgb image [%u] success", nSeq);
        }
        if((nImgType & TYPE_ALIGNED_DEPTH) && stImagePtr[CAMERA_IMAGE_ALIGNED_DEPTH])
        {
            stImagePtr[CAMERA_IMAGE_ALIGNED_DEPTH]->header.stamp = frameStamp;
            stImagePtr[CAMERA_IMAGE_ALIGNED_DEPTH]->header.frame_id = rgbFrameId;
            alignedDepthPub->publish(*stImagePtr[CAMERA_IMAGE_ALIGNED_DEPTH]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send aligned depth image [%u] success", nSeq);
        }
        if((nImgType & TYPE_LEFT_IR) && stImagePtr[CAMERA_IMAGE_LEFT_IR])
        {
            stImagePtr[CAMERA_IMAGE_LEFT_IR]->header.stamp = frameStamp;
            stImagePtr[CAMERA_IMAGE_LEFT_IR]->header.frame_id = leftIrFrameId;
            leftIrPub->publish(*stImagePtr[CAMERA_IMAGE_LEFT_IR]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send left ir image [%u] success", nSeq);
        }
        if((nImgType & TYPE_RIGHT_IR) && stImagePtr[CAMERA_IMAGE_RIGHT_IR])
        {
            stImagePtr[CAMERA_IMAGE_RIGHT_IR]->header.stamp = frameStamp;
            stImagePtr[CAMERA_IMAGE_RIGHT_IR]->header.frame_id = rightIrFrameId;
            rightIrPub->publish(*stImagePtr[CAMERA_IMAGE_RIGHT_IR]);
            RCLCPP_INFO(this->get_logger(), "[Publisher] Send right ir image [%u] success", nSeq);
        }

        stCamInfo[CAMERA_IMAGE_DEPTH].header.stamp = frameStamp;
        stCamInfo[CAMERA_IMAGE_DEPTH].header.frame_id = depthFrameId;
        stCamInfo[CAMERA_IMAGE_RGB].header.stamp = frameStamp;
        stCamInfo[CAMERA_IMAGE_RGB].header.frame_id = rgbFrameId;
        depthCamInfoPub->publish(stCamInfo[0]);
        rgbCamInfoPub->publish(stCamInfo[1]);
        nSeq++;
    }
private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgbPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr alignedDepthPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr leftIrPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rightIrPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depthCamInfoPub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgbCamInfoPub;
    vector<sensor_msgs::msg::CameraInfo> stCamInfo; 
    rclcpp::TimerBase::SharedPtr timer;
    string strSerialNumber;
    string alignedDepthTopic;
    string rgbFrameId;
    string depthFrameId;
    string leftIrFrameId;
    string rightIrFrameId;
    uint32_t nSeq;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublish>();
    rclcpp::spin(node);
    node->camera.stop();
    node->camera.close();
    rclcpp::shutdown();
    return 0;
}
