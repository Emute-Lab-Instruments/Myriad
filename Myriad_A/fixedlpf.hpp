#ifndef FIXEDLPF_HPP
#define FIXEDLPF_HPP

template <size_t bitdepth=12, size_t alpha=12>
class FixedLpf {
 public:
  FixedLpf() { }
  ~FixedLpf() { }
  
  inline size_t play(const size_t value) {
    const size_t v = value << bitdepth;
    const int32_t delta = v - state;
    state += (delta >> alpha);
    return state >> bitdepth;
  }
  
 private:
  int32_t state=0;
  
};

#endif // FIXEDLPF_HPP