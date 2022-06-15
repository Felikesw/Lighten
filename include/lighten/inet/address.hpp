#ifndef __LIGHTEN__INET__ADDRESS_HPP
#define __LIGHTEN__INET__ADDRESS_HPP

#include <netinet/in.h>
#include <lighten/net/address.hpp>

namespace lighten::inet {

class address: public lighten::net::address {
  struct sockaddr_in addr;
  public:
  address(const char *);
  address(const char *, int );
  address(int);
  struct sockaddr *get_addr() {
    return (struct sockaddr *)&addr;
  }
  socklen_t length() {
    return sizeof(struct sockaddr_in);
  }
};

}

#endif
