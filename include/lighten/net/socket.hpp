#ifndef __LIGHTEN__NET__SOCKET__HPP
#define __LIGHTEN__NET__SOCKET__HPP

#include <sys/socket.h>
#include <lighten/io/buffer.hpp>
#include <lighten/net/address.hpp>

namespace lighten::net {

class socket {
  protected:
  int sock;
  public:
  socket(int af, int type, int proto);
  socket(int sock) {
    this->sock = sock;
  }

  ~socket();

  operator int() const{
    return sock;
  }

  int send(const lighten::io::buffer &, int flags = 0);
  int sendto(const lighten::io::buffer &, const address &, int flags = 0);
  int recv(lighten::io::buffer &, int flags = 0);
  int recvfrom(lighten::io::buffer &, address &, int flags = 0);
  int bind(const address &addr); 
  int connect(const address &addr);
  int listen(int backlog = 10);
  socket accept(address &);
  int set_reuse_addr(bool);
};

}

#endif
