#include <wolfvision.hpp>

int main (int argc, char* argv[]) {
  fmt::print("[{}] WolfVision built on g++ version: {}\n", idntifier, __VERSION__);
  fmt::print("[{}] WolfVision config file path: {}\n", idntifier, CONFIG_FILE_PATH);
  // 网页客户端
  auto streamer_ptr = std::make_shared<nadjieb::MJPEGStreamer>();
  streamer_ptr->start(8080, fmt::format("{}{}", SOURCE_PATH, "/utils/streamer.html"));
  // 串口初始化
  auto serial = std::make_shared<RoboSerial>("/dev/ttyUSB0", 115200);
  RoboCmd     robo_cmd;
  RoboInf     robo_inf;
  std::thread uart_write_thread(uartWriteThread, serial, std::ref(robo_cmd));
  uart_write_thread.detach();

  std::thread uart_read_thread(uartReadThread, serial, std::ref(robo_inf));
  uart_read_thread.detach();

  std::thread ptz_camera_thread(PTZCameraThread, std::ref(robo_cmd), std::ref(robo_inf), std::ref(streamer_ptr));
  ptz_camera_thread.detach();

  std::thread watch_dog_thread(watchDogThread, std::ref(ptz_camera_thread), std::ref(uart_read_thread));
  watch_dog_thread.join();

  return 0;
}