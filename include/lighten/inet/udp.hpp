#ifndef __LIGHTEN__INET__UDP__HPP
#define __LIGHTEN__INET__UDP__HPP

#include <lighten/net/address.hpp>

namespace lighten::inet {

class udp_socket: public lighten::net::socket {
  public:
  udp_socket();
};

}


#endif
