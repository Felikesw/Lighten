#ifndef __LIGHTEN__INET__TCP__HPP
#define __LIGHTEN__INET__TCP__HPP

#include <lighten/net/socket.hpp>

namespace lighten::inet {

class tcp_socket: public lighten::net::socket {
  public:
  tcp_socket();
};

}


#endif
