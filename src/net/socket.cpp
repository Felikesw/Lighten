#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>

#include <lighten/net/socket.hpp>

namespace lighten::net {
  socket::socket(int af, int type, int proto) {
    sock = ::socket(af, type, proto);
  }

  socket::~socket() {
    if(sock >= 0)close(sock);
  }

  int socket::send(const lighten::io::buffer &buf, int flags) {
    if(buf.count() == 1) {
      return ::send(sock, buf.get_addr(), buf.get_size(), flags);
    }
    else {
      int count = buf.count();
      struct iovec vec[count];
      struct msghdr msg;

      for(int i=0; i<count; i++){
        vec[i].iov_base = buf.get_addr(i);
        vec[i].iov_len = buf.get_size(i);
      }

      memset(&msg, 0, sizeof(msg));
      msg.msg_iov = vec;
      msg.msg_iovlen = count;

      return sendmsg(sock, &msg, flags);
    }
  }

  int socket::sendto(const lighten::io::buffer &buf, const address &dst, int flags) {
    if(buf.count() == 1) {
      return ::sendto(sock, buf.get_addr(), buf.get_size(), flags, dst.get_addr(), dst.length());
    }
    else {
      int count = buf.count();
      struct iovec vec[count];
      struct msghdr msg;

      for(int i=0; i<count; i++){
        vec[i].iov_base = (void *)buf.get_addr(i);
        vec[i].iov_len = buf.get_size(i);
      }

      memset(&msg, 0, sizeof(msg));
      msg.msg_iov = vec;
      msg.msg_iovlen = count;
      msg.msg_name = (void *)dst.get_addr();
      msg.msg_namelen = dst.length();

      return sendmsg(sock, &msg, flags);
    }
  }

  int socket::recv(lighten::io::buffer &buf, int flags) {
    if(buf.count() == 1) {
      return ::recv(sock, buf.get_addr(), buf.get_size(), flags);
    }
    else {
      int count = buf.count();
      struct iovec vec[count];
      struct msghdr msg;

      for(int i=0; i<count; i++){
        vec[i].iov_base = buf.get_addr(i);
        vec[i].iov_len = buf.get_size(i);
      }

      memset(&msg, 0, sizeof(msg));
      msg.msg_iov = vec;
      msg.msg_iovlen = count;

      return recvmsg(sock, &msg, flags);
    }
  }

  int socket::recvfrom(lighten::io::buffer &buf, address &src, int flags) {
    if(buf.count() == 1) {
      socklen_t addrlen = src.length();
      return ::recvfrom(sock, buf.get_addr(), buf.get_size(), flags, src.get_addr(), &addrlen);
    }
    else {
      int count = buf.count();
      struct iovec vec[count];
      struct msghdr msg;

      for(int i=0; i<count; i++){
        vec[i].iov_base = (void *)buf.get_addr(i);
        vec[i].iov_len = buf.get_size(i);
      }

      memset(&msg, 0, sizeof(msg));
      msg.msg_iov = vec;
      msg.msg_iovlen = count;
      msg.msg_name = (void *)src.get_addr();
      msg.msg_namelen = src.length();

      return recvmsg(sock, &msg, flags);
    }
  }

  int socket::bind(const address &addr) {
    if(::bind(sock, addr.get_addr(), addr.length())<0){
      fprintf(stderr, "cannot bind address: %m\n");
      return -1;
    }

    return 0;
  }

  int socket::connect(const address &addr){
    return ::connect(sock, addr.get_addr(), addr.length());
  }

  int socket::listen(int backlog){
    return ::listen(sock, backlog);
  }

  socket socket::accept(address &addr){
    socklen_t addrlen = addr.length();
    int s = ::accept(sock, addr.get_addr(), &addrlen);
    return socket(s);
  }

  int socket::set_reuse_addr(bool enable){
    int v = enable?1:0;

    if (::setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &v, sizeof(int)) < 0){
      fprintf(stderr, "cannot set SO_REUSEADDR socket: %m\n");
      return -1;
    }

    return 0;
  }
}
