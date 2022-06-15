#include <lighten/future.hpp>
#include <lighten/net/socket.hpp>
#include <lighten/aio/channel.hpp>

namespace lighten::aio {

class port{
  public:
  ref_ptr<channel> add(lighten::net::socket&&);
};

}
