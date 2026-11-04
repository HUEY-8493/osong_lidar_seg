#include "ros_adapter.hpp"
#include <condition_variable>
#include <thread>
namespace general_v2
{
class Live : public Adapter
{
public:
  Live() : Adapter(false)
  {
    subscription = create_subscription<sensor_msgs::msg::PointCloud2>(input, rclcpp::SensorDataQoS(),
                                                                      [this](sensor_msgs::msg::PointCloud2::
                                                                             ConstSharedPtr m){
        std::lock_guard<std::mutex> lock(mutex);
        pending = std::move(m);
        cv.notify_one();
      });
    worker = std::thread([this]{
        std::vector<Point> points;
        for (;;)
        {
          sensor_msgs::msg::PointCloud2::ConstSharedPtr m;
          {std::unique_lock<std::mutex> lock(mutex);
           cv.wait(lock, [this]{
              return !running || pending;
            });
           if (!running)
           {
             return;
           }
           m = std::move(pending);
          }
          try{process(*m, points);
          }
          catch (const std::exception&e){RCLCPP_ERROR(get_logger(), "GeneralV2 rejected frame: %s", e.what());
          }
        }
      });
    RCLCPP_INFO(get_logger(),
                "GeneralV2 independent engine; no static background; obstacle height >= %.3fm above observed ground",
                core->config().obstacle_min);
  }
  ~Live() override
  {
    {
      std::lock_guard<std::mutex> lock(mutex);
      running = false;
    }
    cv.notify_all();
    if (worker.joinable())
    {
      worker.join();
    }
  }
private:
  std::mutex mutex;
  std::condition_variable cv;
  bool running = true;
  std::thread worker;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pending;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription;
};
}
int main(int argc, char**argv)
{
  if(general_v2::core_source_sha()!=GENERAL_V2_SOURCE_SHA){std::cerr<<"GeneralV2 node/core build mismatch\n";return 2;}
  rclcpp::init(argc, argv);
  int result = 0;
  try{
    rclcpp::spin(std::make_shared<general_v2::Live>());
  }
  catch (const std::exception&e){
    std::cerr << e.what() << '\n';
    result = 2;
  }
  rclcpp::shutdown();
  return result;
}
