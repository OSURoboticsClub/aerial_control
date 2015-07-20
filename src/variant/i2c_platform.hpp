#ifndef I2C_PLATFORM_HPP_
#define I2C_PLATFORM_HPP_

class I2CPlatform {
public:
  /**
   * Get the singleton instance.
   */
  static I2CPlatform& getInstance() {
    static I2CPlatform platform;
    return platform;
  }

private:
  I2CPlatform();

  I2CPlatform(I2CPlatform& platform) = delete;
  void operator=(I2CPlatform& platform) = delete;
};

#endif
