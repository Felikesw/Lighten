#include <lighten/aio/channel.hpp>
#include <lighten/queue.hpp>

namespace lighten::aio {

class epoll_channel: public channel {
  protected:
  int fd;
  struct request {
    request_type type;
    unsigned int flags;
    unsigned int offset;
    lighten::ref_ptr<lighten::io::buffer> buffer;
    lighten::promise<int> promise;
    request(const request &) = delete;
    request(request &&) = default;
    request(request_type type, const lighten::ref_ptr<lighten::io::buffer> &buffer, unsigned int flags): type(type), flags(flags), /*idx(0), offset(0),*/ buffer(buffer){};
  };
  public:
  class request_queue {
    protected:
    std::atomic<int> state;
    lighten::queue<request> queue;
    bool get_lock() {
      return false;
    }
    bool put_lock() {
      return false;
    }
    virtual bool handle_request(request &) = 0;
    public:
    void submit(request &&req) {
      //return queue.push(req);
    }
    void run() {
    }
    void close() {
    }
  };
  class input_queue: public request_queue {
    protected:
    bool handle_request(request &) override;
  };

  class output_queue: public request_queue {
    protected:
    bool handle_request(request &) override;
  };

  /*
  lighten::queue<request> in;
  lighten::queue<request> out;
  bool process_input();
  bool process_output();
  */
  future<int> submit(request_type, lighten::ref_ptr<lighten::io::buffer> buf, unsigned int flags = 0) override;
  input_queue in;
  output_queue out;

  //void on_input();
  //void on_output();
};

}
