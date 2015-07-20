#ifndef SPI_PLATFORM_HPP_
#define SPI_PLATFORM_HPP_

class SPIPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static SPIPlatform& getInstance() {
    static SPIPlatform platform;
    return platform;
  }

private:
  SPIPlatform();

  SPIPlatform(SPIPlatform& platform) = delete;
  void operator=(SPIPlatform& platform) = delete;
};

#endif
