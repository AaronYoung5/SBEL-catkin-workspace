namespace chrono {
class ChInterface {
private:
  ChTransportManager transport_manager_;

public:
  ChInterface(ros::NodeHandle &n);

  ~ChInterface();

  void run();
}
} // namespace chrono
