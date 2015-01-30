template <typename M>
void RateLimitedStream::publish(M& m) {
  communicator.send(m);
  lastPublish = chibios_rt::System::getTime();
}
