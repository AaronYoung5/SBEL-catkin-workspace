#include <ros/ros.h>

class ChSubscriber : public ChTransport {
private:
  ros::Subscriber sub_;

public:
  ChSubscriber(std::shared_ptr<boost::asio::ip::tcp::socket> socket,
               ros::NodeHandle &n, std::string topic_name, int queue_size,
               int update_rate);

  void spinOnce();
};
