#ifndef SPI_PLATFORM_HPP_
#define SPI_PLATFORM_HPP_

class SPIPlatform {
public:
  SPIPlatform();

  /**
   * Get the singleton instance.
   */
  static SPIPlatform& getInstance() {
    static SPIPlatform platform;
    return platform;
  }
};

#endif
