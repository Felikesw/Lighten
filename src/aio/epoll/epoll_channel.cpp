#include <sys/uio.h>
#include <string.h>

#include "epoll_channel.hpp"

namespace lighten::aio {

lighten::future<int> epoll_channel::submit(request_type type, lighten::ref_ptr<lighten::io::buffer> buf, unsigned int flags)
{
  request req(type, buf, flags);
  lighten::future<int> fut = req.promise.get_future();

  switch(type){
    case READ:
    case ACCEPT:
    case RECV:
    case RECVFROM:
      in.submit(std::move(req));
      break;
    case WRITE:
    case SEND:
    case SENDTO:
      out.submit(std::move(req));
      break;
    default:
      break;
  }

  return fut;
}

/*
bool epoll_channel::process_input()
{
  if(in.empty())return false;

  auto it = in.front();

  if(it.get().type == ACCEPT){
    socklen_t addrlen = it.get().buffer->get_size();
restart_1:
    int sock = accept(fd, (struct sockaddr *)it.get().buffer->get_addr(), &addrlen);
    if(sock < 0){
      if(errno == EINTR){
        goto restart_1;
      }
      else if(errno == EAGAIN){
        return false;
      }
      else {
      }
    }
    else {
      it.get().promise.set_value(sock);
      in.pop();
    }

    return true;
  }
  else {
    int count = it.get().buffer->count();
    int idx = 0;
    unsigned int off = it.get().offset;

    if(it.get().type == RECVFROM){
      if(count < 2){
      }
      else {
        count --;
      }
    }

    if(it.get().flags && it.get().offset) {
      for(int i=0; count > 0; i++){
        size_t size = it.get().buffer->get_size(i);
        if(size > off){
          break;
        }
        else {
          off -= size;
          count --;
          idx++;
        }
      }
    }

    if(count == 0) {
      it.get().promise.set_value(it.get().offset);
      return true;
    }

    struct iovec vec[count];
    for(int i=0; i<count; i++){
      vec[i].iov_base = it.get().buffer->get_addr(idx) + off;
      vec[i].iov_len = it.get().buffer->get_size(idx) - off;
      off = 0;
      idx++;
    }

    int res;
restart_2:
    switch(it.get().type){
      case READ:
        res = ::readv(fd, vec, count);
        break;
      case RECV:
      case RECVFROM: {
        struct msghdr msg;
        memset(&msg, 0, sizeof(msg));
        msg.msg_iov = vec;
        msg.msg_iovlen = count;
        if(it.get().type == RECVFROM){
          msg.msg_name = (void*)it.get().buffer->get_addr(idx);
          msg.msg_namelen = it.get().buffer->get_size(idx);
        }
        res = ::recvmsg(fd, &msg, 0);
      }break;
      default:
        break;
    }

    if(res <= 0){
      if(res < 0){
        if(errno == EINTR){
          goto restart_2;
        }
        else if(errno == EAGAIN){
          return false;
        }
        else {
        }
      }
    }
    else {
      if(!it.get().flags){
        it.get().promise.set_value(res);
        in.pop();
      }
      else {
        it.get().offset += res;
      }
    }

    return true;
  }
}

void epoll_channel::on_input()
{
  bool res;
  do{
    res = process_input();
  }while(res);
}

bool epoll_channel::process_output()
{
  if(out.empty())return false;

  return true;
}

void epoll_channel::on_output()
{
  bool res;
  do{
    res = process_output();
  }while(res);
}
*/

}
