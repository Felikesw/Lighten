#ifndef __LIGHTEN__QUEUE__HPP
#define __LIGHTEN__QUEUE__HPP

#include <lighten/ref.hpp>

namespace lighten {

template<class T>
class queue {
  protected:
  class node: public referenceable {
    public:
    ref_ptr<node> next;
    T value;
  };
  ref_ptr<node> first;
  ref_ptr<node> last;
  public:
  /*
  class iterator {
    ref_ptr<node> _node;
    public:
    T& operator*() const {
      return _node->value;
    }

    T *operator->() const{
      if(_node) {
        return &_node->value;
      }
      else {
        return nullptr;
      }
    }
  };
  */
  class item_wrapper {
    ref_ptr<node> _node;
    public:
    constexpr operator T&() const noexcept { return _node->value; }
    constexpr T& get() const noexcept { return _node->value; }
  };
  bool push(const T &value);
  bool push(T &&value);
  bool pop();
  item_wrapper front();
  item_wrapper back();
  bool empty();
  template< class... Args >
  item_wrapper emplace( Args&&... args );
};

}

#endif
