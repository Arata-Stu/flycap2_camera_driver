#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <FlyCapture2.h>

class Grasshopper3 : public rclcpp::Node
{
public:
  Grasshopper3() : Node("grasshopper")
  {
    // パラメータの役割
    declare_parameter("fps", 20);
    declare_parameter("mode", 2);
    declare_parameter("format", "raw");
    declare_parameter("timeout", 1000);

    fps = get_parameter("fps").as_int();
    mode = get_parameter("mode").as_int();
    format = get_parameter("format").as_string();
    timeout = get_parameter("timeout").as_int();

    desired_mode = static_cast<FlyCapture2::Mode>(mode);

    if (format == "rgb")
    {
      desired_pixel_format = FlyCapture2::PIXEL_FORMAT_RGB8;
    }
    else
    {
      desired_pixel_format = FlyCapture2::PIXEL_FORMAT_RAW8;
    }

    int camera_num = get_num_cameras(&busMgr);
    initialize_cameras(cameras, &busMgr, camera_num, desired_mode, desired_pixel_format, timeout);

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_raw", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&Grasshopper3::timer_callback, this));

    start_capture(cameras);
  }

private:
  unsigned int get_num_cameras(FlyCapture2::BusManager* bus_manager)
  {
    unsigned int cameras;
    FlyCapture2::Error error = bus_manager->GetNumOfCameras(&cameras);
    if (error != FlyCapture2::PGRERROR_OK)
    {
      error.PrintErrorTrace();
      std::exit(-1);
    }

    std::cout << "Number of cameras detected: " << cameras << std::endl;

    if (cameras < 1)
    {
      std::cerr << "Error: This program requires at least 1 camera." << std::endl;
      std::exit(-1);
    }
    return cameras;
  }

  void initialize_cameras(std::vector<FlyCapture2::Camera *> &cameras,
                          FlyCapture2::BusManager *bus_manager,
                          int camera_num,
                          FlyCapture2::Mode desired_mode,
                          FlyCapture2::PixelFormat desired_pixel_format,
                          int timeout_ms)
  {
    for (int i = 0; i < camera_num; i++)
    {
      FlyCapture2::Camera *camera = new FlyCapture2::Camera();

      FlyCapture2::PGRGuid guid;
      FlyCapture2::Error error = bus_manager->GetCameraFromIndex(i, &guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      error = camera->Connect(&guid);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      FlyCapture2::EmbeddedImageInfo image_info;
      error = camera->GetEmbeddedImageInfo(&image_info);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      image_info.timestamp.onOff = false;
      error = camera->SetEmbeddedImageInfo(&image_info);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      FlyCapture2::CameraInfo camera_info;
      error = camera->GetCameraInfo(&camera_info);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      FlyCapture2::VideoMode default_video_mode;
      FlyCapture2::FrameRate default_frame_rate;

      error = camera->GetVideoModeAndFrameRate(&default_video_mode, &default_frame_rate);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      FlyCapture2::Format7ImageSettings image_settings;
      bool supported = false;
      unsigned int packet_size;
      float percentage;

      FlyCapture2::Format7Info format7_info;
      format7_info.mode = desired_mode;

      error = camera->GetFormat7Info(&format7_info, &supported);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }

      print_format7_info(format7_info, supported);

      if (supported)
      {
        error = camera->GetFormat7Configuration(&image_settings, &packet_size, &percentage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
          error.PrintErrorTrace();
          std::exit(-1);
        }

        image_settings.mode = desired_mode;
        image_settings.pixelFormat = desired_pixel_format;
        image_settings.offsetX = 0;
        image_settings.offsetY = 0;
        image_settings.width = format7_info.maxWidth;
        image_settings.height = format7_info.maxHeight;

        FlyCapture2::Format7PacketInfo packet_info;
        bool valid_settings = false;
        error = camera->ValidateFormat7Settings(&image_settings, &valid_settings, &packet_info);
        if (error != FlyCapture2::PGRERROR_OK)
        {
          error.PrintErrorTrace();
          std::exit(-1);
        }
        packet_size = packet_info.recommendedBytesPerPacket;
        error = camera->SetFormat7Configuration(&image_settings, packet_size);
        if (error != FlyCapture2::PGRERROR_OK)
        {
          error.PrintErrorTrace();
          std::exit(-1);
        }

        error = camera->GetFormat7Configuration(&image_settings, &packet_size, &percentage);
        if (error != FlyCapture2::PGRERROR_OK)
        {
          error.PrintErrorTrace();
          std::exit(-1);
        }

        print_image_settings(image_settings, packet_size, percentage);
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Selected Mode not supported, using last working mode.");
      }

      FlyCapture2::FC2Config camera_config;
      error = camera->GetConfiguration(&camera_config);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        RCLCPP_INFO(this->get_logger(), "Could not read configuration from Camera");
      }
      else
      {
        if (timeout_ms > 0)
          camera_config.grabTimeout = timeout_ms;

        error = camera->SetConfiguration(&camera_config);
        if (error != FlyCapture2::PGRERROR_OK)
        {
          error.PrintErrorTrace();
          RCLCPP_INFO(this->get_logger(), "Could not set configuration on Camera");
        }
      }

      print_camera_info(&camera_info);
      cameras.push_back(camera);
    }
  }

  void print_camera_info(FlyCapture2::CameraInfo* info)
  {
    std::cout << "\n*** CAMERA INFORMATION ***\n"
              << "\tSerial number       - " << info->serialNumber << "\n"
              << "\tCamera model        - " << info->modelName << "\n"
              << "\tCamera vendor       - " << info->vendorName << "\n"
              << "\tSensor              - " << info->sensorInfo << "\n"
              << "\tResolution          - " << info->sensorResolution << "\n"
              << "\tFirmware version    - " << info->firmwareVersion << "\n"
              << "\tFirmware build time - " << info->firmwareBuildTime << "\n"
              << std::endl;
  }

  void print_image_settings(FlyCapture2::Format7ImageSettings settings, unsigned int packet_size, float percentage)
  {
    std::cout << "Custom image settings:\n"
              << "    mode: " << settings.mode << "\n"
              << "    offsetX: " << settings.offsetX << "\n"
              << "    offsetY: " << settings.offsetY << "\n"
              << "    width: " << settings.width << "\n"
              << "    height: " << settings.height << "\n"
              << "    pixelFormat: " << settings.pixelFormat << "\n"
              << "    packet size: " << packet_size << "\n"
              << "    percentage: " << percentage << "\n";
  }

  void print_format7_info(FlyCapture2::Format7Info info, bool supported)
  {
    std::cout << "Format7 mode supported: " << supported << "\n";
    if (supported)
    {
      std::cout << "Max image pixels: (" << info.maxWidth << ", " << info.maxHeight << ")\n";
      std::cout << "Image unit size: (" << info.imageHStepSize << ", " << info.imageVStepSize << ")\n";
      std::cout << "Offset unit size: (" << info.offsetHStepSize << ", " << info.offsetVStepSize << ")\n";
      std::cout << "Pixel format bitfield: 0x" << std::hex << info.pixelFormatBitField << "\n";
    }
    else
    {
      std::cout << "Selected Mode not supported, using last working mode.\n";
    }
  }

  void start_capture(std::vector<FlyCapture2::Camera *> cameras)
  {
    for (auto *camera : cameras)
    {
      FlyCapture2::Error error = camera->StartCapture();
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        std::exit(-1);
      }
    }
  }

  void timer_callback()
  {
    for (auto *camera : cameras)
    {
      FlyCapture2::Image image;
      FlyCapture2::Error error = camera->RetrieveBuffer(&image);
      if (error != FlyCapture2::PGRERROR_OK)
      {
        error.PrintErrorTrace();
        continue;
      }

      // check encoding pattern
      std::string encoding_pattern;
      switch (image.GetBayerTileFormat()) {
      case FlyCapture2::RGGB:
        encoding_pattern = "bayer_rggb8";
        break;
      case FlyCapture2::GRBG:
        encoding_pattern = "bayer_grbg8";
        break;
      case FlyCapture2::GBRG:
        encoding_pattern = "bayer_gbrg8";
        break;
      case FlyCapture2::BGGR:
        encoding_pattern = "bayer_bggr8";
        break;
      default:
        encoding_pattern = "rgb8";
      }

      auto msg = sensor_msgs::msg::Image();
      msg.header.stamp = this->get_clock()->now();
      msg.header.frame_id = "grasshopper3";
      msg.height = image.GetRows();
      msg.width  = image.GetCols();
      msg.encoding = encoding_pattern;
      msg.step = image.GetStride();

      size_t image_size = image.GetDataSize();
      msg.data.resize(image_size);
      memcpy(msg.data.data(), image.GetData(), image_size);

      publisher_->publish(msg);
    }
  };

  int fps;
  int mode;
  std::string format;
  int timeout;

  FlyCapture2::BusManager busMgr;
  FlyCapture2::Mode desired_mode;
  FlyCapture2::PixelFormat desired_pixel_format;
  std::vector<FlyCapture2::Camera*> cameras;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Grasshopper3>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
