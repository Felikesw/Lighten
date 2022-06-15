#ifndef __MEDIA__BUFFER__HPP
#define __MEDIA__BUFFER__HPP

#include <stdint.h>

namespace lighten::io {

class buffer {
  protected:
  unsigned int flags;

  public:
  enum flags{
    PHYSICAL_ADDR = 1, 
    UNDERLYING_FILE = 2,
  };

  struct metadata {
    unsigned int type;
  };

  struct physical_addr: public metadata {
    uintptr_t addr;
  };

  virtual unsigned int count() const = 0;
  unsigned int get_flags() const { return flags; }
  virtual uint8_t *get_addr(unsigned int idx = 0) const = 0; 
  virtual size_t get_size(unsigned int idx = 0) const = 0;

  virtual int get_metadata(unsigned int idx, metadata &data) {
    return -1;
  }
  virtual void flush(unsigned int idx = 0);

  uint8_t *operator[](int idx) const { return get_addr(idx); }

  buffer(): flags(0) {}
};

class single_buffer: public buffer
{
  public:
    unsigned int count() const override{
      return 1;
    }
    uint8_t *get_addr(unsigned int idx) const override {
      return (idx < 1)?buffer:nullptr;
    }
    virtual size_t get_size(unsigned int idx) const override {
      return (idx< 1)?size:0;
    }


  protected:
    single_buffer(uint8_t *buffer, size_t size): buffer(buffer), size(size){}
    uint8_t *buffer;
    size_t size;
};

class mem_buffer: public single_buffer
{
  public:
  mem_buffer(size_t size): single_buffer(new uint8_t[size], size){
  }

  virtual ~mem_buffer() {
    delete buffer;
  }
};

template<int N>
class scatter_buffer: public buffer{
  public:
    virtual unsigned int count() const override{
      return segment_count;
    }

    virtual uint8_t *get_addr(unsigned int idx) const override {
      return (idx < segment_count)?segments[idx].addr:nullptr;
    }
    virtual size_t get_size(unsigned int idx) const override {
      return (idx< segment_count)?segments[idx].size:0;
    }

  protected:
    scatter_buffer(): segment_count(N){}
    unsigned int segment_count;
    struct segment {
      uint8_t *addr;
      size_t size;
    }segments[N];
};

class file_buffer: public buffer{
  public:
  struct range {
    off_t offset;
    size_t length;
  };

  struct underlying_file: public metadata {
    int fd;
    struct range range;
  };

  file_buffer(int fd);
  file_buffer(int fd, const struct range *ranges, unsigned int count);
  file_buffer(const char *filename);
  file_buffer(const char *filename, const struct range *ranges, unsigned int count);
  unsigned int count() const override{
    return range_count;
  }
  uint8_t *get_addr(unsigned int idx) const override;
  size_t get_size(unsigned int idx) const override {
    return (idx< range_count)?ranges[idx].length:0;
  }
  int get_metadata(unsigned int idx, metadata &data) override; 

  protected:
  int fd;
  unsigned int range_count;
  struct range_map: public range {
    uint8_t *addr;
  };
  struct range_map *ranges;
};

}
#endif
