#ifndef POTM_RINGBUF_H_
#define POTM_RINGBUF_H_

#include <optional>

/// Ring buffer class
/// This is designed so we can "push" by getting a reference to where the new element will be
/// without a copy, and then add direct into it. In a congested use, this means the effective size is
/// one short, but we can just size accordingly if that becomes a problem.
/// This is intended to be lock free, so we dont need to waste a critical section to access it
/// HOWEVER this requires the index type to fix in an atomic register of the processor!
/// AND there b only a single thread pushing and another single thread popping 
template<typename ELEMENT, int NUM_ELEMENTS, typename INDEX_TYPE = uint8_t>
class RingBuffer {
public:
  typedef INDEX_TYPE Index_t;
  typedef ELEMENT Element_t;
private:
  Element_t ring[NUM_ELEMENTS];
  Index_t head; // this is the slot where the next element will be added to
  Index_t tail; // the slots between tail..head are in use (count = wrapped(head - tail) )
  bool reserved; // true between call to reserve() and advance() 
  Index_t advanced; // next value of head computed in reserve()

public:
  RingBuffer() : head(0), tail(0), reserved(false), advanced(0) {}

  Index_t getMaxElements() const { return NUM_ELEMENTS; }

  /// @brief Advance the ring buffer, provided we havent already called reserve()
  /// @return Pointer to slot to be used for element being added, or NULL if full
  /// (It would be nice if std::optional<T&> was finally sorted out!)
  Element_t* reserve() {
    assert(!reserved);
    if (reserved) { return NULL; }
    Index_t next = head + 1;
    if (next == (Index_t)NUM_ELEMENTS) { next = 0; }
    if (next == tail) { return NULL; } // full

    reserved = true;
    Index_t cur = head;
    advanced = next;
    return &ring[cur];
  }

  /// @brief Call when done with the memory from reserve() and the other side is free to pop
  /// Note, the purpose of these shenanigans limit the number of memcpys
  /// It is only safe for reserve() and advance() to be called in the same context
  void advance() {
    assert(reserved);
    if (!reserved) { return; }
    head = advanced;
    reserved = false;
  }


  /// @brief Remove oldest element from the ring buffer
  /// @return false if already empty
  bool pop() {
    if (head == tail) { return false; } // empty
    Index_t next = tail;
    next++;
    if (next == (Index_t)NUM_ELEMENTS) { next = 0; }
    tail = next;
    return true;
  }   

  /// @brief Copy oldest element from the ring buffer. This is only safe if caller is only place also calling popBack() 
  /// @return Copy of back object, or nullptr if empty
  std::optional<Element_t> peek() const {
    if (head == tail) { return {}; } // empty
    return ring[tail];
  }   

};



#endif
