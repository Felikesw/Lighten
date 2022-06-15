#ifndef __NET__ADDRESS__HPP
#define __NET__ADDRESS__HPP

#include <sys/socket.h>

namespace lighten::net {

class address{
  public:
  virtual struct sockaddr *get_addr() const = 0; 
  operator struct sockaddr *() {
    return get_addr();
  }
  virtual socklen_t length() const  = 0;
};

}

#endif
