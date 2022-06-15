#ifndef __LIGHTEN__AIO__CHANNEL__HPP
#define __LIGHTEN__AIO__CHANNEL__HPP

#include <memory>

#include <lighten/future.hpp>
#include <lighten/net/address.hpp>
#include <lighten/net/socket.hpp>

namespace lighten::aio {

enum request_type {
  READ,
  WRITE,
  ACCEPT,
  SEND,
  RECV,
  SENDTO,
  RECVFROM
};

class channel {
  public:
  //virtual future<std::unique_ptr<lighten::io::buffer>> submit(request_type, std::unique_ptr<lighten::io::buffer> buf, unsigned int flags = 0) = 0;
  virtual future<int> submit(request_type, lighten::ref_ptr<lighten::io::buffer> buf, unsigned int flags = 0) = 0;
};

}

#endif
