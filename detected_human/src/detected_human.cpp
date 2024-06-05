#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <gazebo_msgs/ModelStates.h>
#include <fstream>

using namespace cv;
using namespace dnn;

class HumanDetector
{
public:
    HumanDetector()
        : it_(nh_)
    {
        // Subscribe to input video feed
        image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &HumanDetector::imageCb, this);
        human_pub_ = nh_.advertise<gazebo_msgs::ModelStates>("/mybot_description/detected_human", 10);
        image_pub_ = it_.advertise("/camera/rgb/image_annotated", 1);

        // Load YOLO model
        net_ = readNetFromDarknet("/home/quyenanhpt/yolo/yolov3.cfg", "/home/quyenanhpt/yolo/yolov3.weights");
        net_.setPreferableBackend(DNN_BACKEND_OPENCV);
        net_.setPreferableTarget(DNN_TARGET_CPU);

        // Load names of classes
        std::ifstream ifs("/home/quyenanhpt/yolo/coco.names");
        std::string line;
        while (getline(ifs, line))
            classes_.push_back(line);
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Detect humans in the image
        detectHumans(cv_ptr->image);

        // Annotate and publish the image
        cv_bridge::CvImage out_msg;
        out_msg.header = cv_ptr->header;
        out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = cv_ptr->image;
        image_pub_.publish(out_msg.toImageMsg());
    }

    void detectHumans(cv::Mat &frame)
    {
        Mat blob;
        blobFromImage(frame, blob, 1 / 255.0, Size(416, 416), Scalar(0, 0, 0), true, false);
        net_.setInput(blob);
        std::vector<Mat> outs;
        net_.forward(outs, getOutputsNames(net_));

        std::vector<int> classIds;
        std::vector<float> confidences;
        std::vector<Rect> boxes;
        for (size_t i = 0; i < outs.size(); ++i)
        {
            float *data = (float *)outs[i].data;
            for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
            {
                Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
                Point classIdPoint;
                double confidence;
                minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
                if (confidence > 0.5 && classes_[classIdPoint.x] == "person") // Kiểm tra confidence
                {
                    int centerX = (int)(data[0] * frame.cols);
                    int centerY = (int)(data[1] * frame.rows);
                    int width = (int)(data[2] * frame.cols);
                    int height = (int)(data[3] * frame.rows);
                    int left = centerX - width / 2;
                    int top = centerY - height / 2;

                    classIds.push_back(classIdPoint.x);
                    confidences.push_back((float)confidence);
                    boxes.push_back(Rect(left, top, width, height));

                    // In thông tin phát hiện ra màn hình
                    ROS_INFO("Detected human: left=%d, top=%d, width=%d, height=%d, confidence=%.2f", left, top, width, height, confidence);
                }
            }
        }

        std::vector<int> indices;
        NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

        gazebo_msgs::ModelStates detectedHumans;
        for (size_t i = 0; i < indices.size(); ++i)
        {
            int idx = indices[i];
            Rect box = boxes[idx];
            detectedHumans.name.push_back("human");
            geometry_msgs::Pose pose;
            pose.position.x = box.x + box.width / 2;
            pose.position.y = box.y + box.height / 2;
            pose.position.z = 0.0; // Assuming 2D detection
            detectedHumans.pose.push_back(pose);

            geometry_msgs::Twist twist;
            twist.linear.x = 0.0;
            twist.linear.y = 0.0;
            twist.linear.z = 0.0;
            twist.angular.x = 0.0;
            twist.angular.y = 0.0;
            twist.angular.z = 0.0;
            detectedHumans.twist.push_back(twist);

            // Draw the bounding box
            rectangle(frame, box, Scalar(0, 255, 0), 3);
        }

        human_pub_.publish(detectedHumans);
    }

    std::vector<String> getOutputsNames(const Net &net)
    {
        static std::vector<String> names;
        if (names.empty())
        {
            // Get the indices of the output layers, i.e. the layers with unconnected outputs
            std::vector<int> outLayers = net.getUnconnectedOutLayers();
            // get the names of all the layers in the network
            std::vector<String> layersNames = net.getLayerNames();
            // Get the names of the output layers in names
            names.resize(outLayers.size());
            for (size_t i = 0; i < outLayers.size(); ++i)
                names[i] = layersNames[outLayers[i] - 1];
        }
        return names;
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher human_pub_;

    Net net_;
    std::vector<std::string> classes_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "human_detector");
    HumanDetector hd;
    ros::spin();
    return 0;
}
