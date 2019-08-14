#include <ros/ros.h>

class ChPublisher : public ChTransport {
private:
  ros::Publisher pub_;
  int update_rate_;

public:
  ChPublisher(std::string id,
              std::shared_ptr<boost::asio::ip::tcp::socket> socket,
              ros::NodeHandle &n, std::string topic_name, int queue_size,
              int update_rate);

  void spinOnce();
};
