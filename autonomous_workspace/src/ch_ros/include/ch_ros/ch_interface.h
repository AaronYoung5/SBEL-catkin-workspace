namespace chrono {
class ChInterface {
private:
  ChSensorManager sensor_manager_;

  ChPublisher time_;
  ChSubscriber control_;

public:
  ChInterface(ros::NodeHandle &n);

  ~ChInterface();
}
} // namespace chrono
