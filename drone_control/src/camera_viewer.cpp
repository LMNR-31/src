#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <map>
#include <mutex>
#include <string>
#include <vector>

class CameraViewer : public rclcpp::Node {
private:
  std::map<std::string, cv::Mat> images_rgb_;
  std::mutex image_mutex_;

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;

  int window_width_{1600};
  int window_height_{900};

public:
  CameraViewer() : Node("camera_viewer") {
    declare_parameter<int>("window_width", 1600);
    declare_parameter<int>("window_height", 900);
    window_width_ = get_parameter("window_width").as_int();
    window_height_ = get_parameter("window_height").as_int();

    cv::namedWindow("Drone Cameras", cv::WINDOW_NORMAL);
    cv::resizeWindow("Drone Cameras", window_width_, window_height_);

    const std::vector<std::string> topics = {
        "/uav1/rgbd_front/color/image_raw",
        "/uav1/rgbd_down/color/image_raw",
    };

    auto qos = rclcpp::SensorDataQoS();

    for (const auto &topic : topics) {
      auto sub = this->create_subscription<sensor_msgs::msg::Image>(
          topic, qos,
          [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
            this->imageCallback(msg, topic);
          });
      subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "Subscribed to: %s", topic.c_str());
    }
  }

  void spinRender() {
    // WallRate não depende do contexto ROS estar válido (evita crash no Ctrl+C)
    rclcpp::WallRate rate(30);

    while (rclcpp::ok()) {
      rclcpp::spin_some(shared_from_this());

      // snapshot das imagens
      std::map<std::string, cv::Mat> images_copy;
      {
        std::lock_guard<std::mutex> lock(image_mutex_);
        images_copy = images_rgb_;
      }

      cv::Mat canvas_bgr = createCanvasBGR(images_copy);
      cv::imshow("Drone Cameras", canvas_bgr);

      int key = cv::waitKey(1) & 0xFF;
      if (key == 27) {  // ESC
        rclcpp::shutdown();
        break;
      }

      // Se Ctrl+C aconteceu durante o loop, sai sem tentar dormir
      if (!rclcpp::ok()) {
        break;
      }
      rate.sleep();
    }
  }

private:
  void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                     const std::string &topic) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
                         "RX %s %ux%u encoding=%s",
                         topic.c_str(), msg->width, msg->height, msg->encoding.c_str());

    try {
      // Guarda como RGB
      auto cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
      const cv::Mat &rgb = cv_ptr->image;

      std::lock_guard<std::mutex> lock(image_mutex_);
      images_rgb_[topic] = rgb.clone();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge error on %s: %s", topic.c_str(), e.what());
    }
  }

  cv::Mat createCanvasBGR(const std::map<std::string, cv::Mat> &images_rgb) {
    // fundo preto
    cv::Mat canvas(window_height_, window_width_, CV_8UC3, cv::Scalar(0, 0, 0));

    // Queremos sempre 2 “slots” (front e down)
    struct Slot { std::string topic; std::string label; };
    const std::vector<Slot> slots = {
        {"/uav1/rgbd_front/color/image_raw", "rgbd_front"},
        {"/uav1/rgbd_down/color/image_raw",  "rgbd_down"},
    };

    const int cols = 2;
    const int rows = 1;
    const int cell_w = window_width_ / cols;
    const int cell_h = window_height_ / rows;

    for (int i = 0; i < (int)slots.size(); ++i) {
      int col = i % cols;
      int row = i / cols;
      int x0 = col * cell_w;
      int y0 = row * cell_h;

      auto it = images_rgb.find(slots[i].topic);
      if (it == images_rgb.end() || it->second.empty()) {
        cv::putText(canvas, ("waiting: " + slots[i].label),
                    cv::Point(x0 + 20, y0 + 40),
                    cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
        continue;
      }

      // resize RGB
      cv::Mat resized_rgb;
      cv::resize(it->second, resized_rgb, cv::Size(cell_w, cell_h));

      // converte RGB->BGR para mostrar
      cv::Mat resized_bgr;
      cv::cvtColor(resized_rgb, resized_bgr, cv::COLOR_RGB2BGR);

      resized_bgr.copyTo(canvas(cv::Rect(x0, y0, cell_w, cell_h)));

      cv::putText(canvas, slots[i].label,
                  cv::Point(x0 + 20, y0 + 40),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    }

    // linha separadora vertical
    cv::line(canvas, cv::Point(cell_w, 0), cv::Point(cell_w, window_height_),
             cv::Scalar(200, 200, 200), 2);

    return canvas;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraViewer>();
  node->spinRender();
  rclcpp::shutdown();
  return 0;
}