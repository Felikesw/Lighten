#include <iostream>
#include <lighten/future.hpp>
#include <lighten/io/buffer.hpp>
#include <lighten/inet/address.hpp>
#include <lighten/inet/tcp.hpp>
#include <lighten/aio.hpp>
#include <lighten/queue.hpp>

namespace ltn = lighten;

struct s {
  int i;
};

void future_test()
{
  ltn::queue<struct s> q;

  int i = ((s &)q.front()).i;

  ltn::packaged_task<int(const char *, size_t)> proc([](const char *data, size_t size){
    return (int)size;
  });

  //ltn::future<string> fut_str =
  //proc("test", 4)
  proc.get_future()
    .then([](ltn::future<int> res){ return std::string("text"); })
    //.then([](ltn::future<string> txt){ ltn::promise<string> pr; return pr.get_future(); })
    .then(ltn::executor(), [](ltn::future<std::string> txt){ std::cout << txt.get();})
    .then(ltn::executor(), []{
    })
    .then([]{
    })
    /*
    .then([](ltn::future<util::executor> fut){
    })
    */
    ;
}


