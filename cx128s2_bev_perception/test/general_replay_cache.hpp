#pragma once
// Offline-only, fieldwise lossless encoding. Never dump C++ object padding or pointers.
#include <boost/pfr.hpp>
#include <openssl/sha.h>
#include <lz4.h>
// Public libxxhash 0.8 ABI; no hash implementation is copied into the harness.
extern "C" {
struct XXH128_hash_t { uint64_t low64, high64; };
XXH128_hash_t XXH3_128bits(const void *, size_t);
}
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
#include <cstring>
#include <type_traits>
#include <stdexcept>
#include <map>
namespace replay_cache {
using Json = nlohmann::json;
namespace fs = std::filesystem;
inline std::string sha(const void * p, size_t n) {
  unsigned char hash[SHA256_DIGEST_LENGTH]; SHA256(static_cast<const unsigned char *>(p), n, hash);
  static const char hex[]="0123456789abcdef"; std::string s;
  for(auto b:hash){s+=hex[b>>4];s+=hex[b&15];} return s;
}
inline std::string shaFile(const fs::path &p) {
  std::ifstream in(p,std::ios::binary); if(!in)throw std::runtime_error("Cannot read dependency: "+p.string());
  SHA256_CTX c; SHA256_Init(&c); char b[1<<20];
  while(in){in.read(b,sizeof b);SHA256_Update(&c,b,in.gcount());}
  unsigned char h[32];SHA256_Final(h,&c);std::string s;const char *x="0123456789abcdef";
  for(auto v:h){s+=x[v>>4];s+=x[v&15];}return s;
}
template<class T>struct Custom;
template<class T>struct Sequence:std::false_type{};
template<class T,class A>struct Sequence<std::vector<T,A>>:std::true_type{};
template<class T,class A>struct Sequence<std::deque<T,A>>:std::true_type{};
template<class T,size_t N>struct Fixed:std::false_type{};
// std::array is an aggregate, but PFR treats it specially; handle explicitly.
template<class T>struct Vector:std::false_type{};
template<class T,class A>struct Vector<std::vector<T,A>>:std::true_type{};
template<class T>struct Array:std::false_type{};
template<class T,size_t N>struct Array<std::array<T,N>>:std::true_type{};
template<class T>struct Mapping:std::false_type{};
template<class K,class V,class C,class A>struct Mapping<std::map<K,V,C,A>>:std::true_type{};
template<class K,class V,class H,class E,class A>struct Mapping<std::unordered_map<K,V,H,E,A>>:std::true_type{};
struct Writer {
  static constexpr bool reading=false;std::vector<char> bytes;
  void raw(void *p,size_t n){auto b=static_cast<char *>(p);bytes.insert(bytes.end(),b,b+n);}
  template<class T>void operator()(const T &v){apply(*this,const_cast<T &>(v));}
};
struct Reader {
  static constexpr bool reading=true;const std::vector<char>&bytes;size_t pos=0;
  void raw(void *p,size_t n){if(n>bytes.size()-pos)throw std::runtime_error("Truncated cache field");std::memcpy(p,bytes.data()+pos,n);pos+=n;}
  template<class T>void operator()(T &v){apply(*this,v);}
};
template<class A,class T>void apply(A &a,T &v){
  if constexpr(std::is_same_v<T,std::vector<cx128s2_bev_perception::PointXYZIRT>>) {
    // Explicit 22-byte contract, without the 2 native tail/alignment padding bytes.
    uint64_t n=v.size();a(n);if(n>100000000)throw std::runtime_error("Point count limit");
    if constexpr(A::reading)v.resize(n);
    std::vector<char> packed(n*22);char *p=packed.data();
    if constexpr(!A::reading)for(auto &q:v){std::memcpy(p,&q.x,4);std::memcpy(p+4,&q.y,4);std::memcpy(p+8,&q.z,4);std::memcpy(p+12,&q.intensity,4);std::memcpy(p+16,&q.ring,2);std::memcpy(p+18,&q.time,4);p+=22;}
    if(n)a.raw(packed.data(),packed.size());
    p=packed.data();if constexpr(A::reading)for(auto&q:v){std::memcpy(&q.x,p,4);std::memcpy(&q.y,p+4,4);std::memcpy(&q.z,p+8,4);std::memcpy(&q.intensity,p+12,4);std::memcpy(&q.ring,p+16,2);std::memcpy(&q.time,p+18,4);p+=22;}
  }
  else if constexpr(std::is_arithmetic_v<T>||std::is_enum_v<T>) a.raw(&v,sizeof v);
  else if constexpr(std::is_same_v<T,std::string>){uint64_t n=v.size();a(n);if(n>uint64_t(1)<<32)throw std::runtime_error("Cache string limit");if constexpr(A::reading)v.resize(n);if(n)a.raw(v.data(),n);}
  else if constexpr(Sequence<T>::value){uint64_t n=v.size();a(n);if(n>100000000)throw std::runtime_error("Cache sequence limit");if constexpr(A::reading)v.resize(n);if constexpr(Vector<T>::value && std::is_arithmetic_v<typename T::value_type> && !std::is_same_v<typename T::value_type,bool>){if(n)a.raw(v.data(),n*sizeof(typename T::value_type));}else for(size_t i=0;i<n;++i){if constexpr(std::is_same_v<typename T::value_type,bool>){bool b=v[i];a(b);if constexpr(A::reading)v[i]=b;}else a(v[i]);}}
  else if constexpr(Array<T>::value){for(auto &x:v)a(x);}
  else if constexpr(Mapping<T>::value){uint64_t n=v.size();a(n);if(n>10000000)throw std::runtime_error("Cache map limit");if constexpr(A::reading){v.clear();for(size_t i=0;i<n;++i){typename T::key_type k;typename T::mapped_type x;a(k);a(x);if(!v.emplace(k,std::move(x)).second)throw std::runtime_error("Duplicate key");}}else{std::map<typename T::key_type,typename T::mapped_type> ordered(v.begin(),v.end());for(auto &kv:ordered){a(kv.first);a(kv.second);}}}
  else if constexpr(std::is_aggregate_v<T>) boost::pfr::for_each_field(v,[&](auto &x){a(x);});
  else Custom<T>::visit(a,v);
}
template<class... T>std::string digest(const T &...v){Writer w;(w(v),...);return sha(w.bytes.data(),w.bytes.size());}
inline void saveJson(const fs::path&p,const Json&j){std::ofstream o(p);o<<j.dump(2)<<'\n';if(!o)throw std::runtime_error("Cannot write "+p.string());}
inline Json readJson(const fs::path&p){std::ifstream in(p);if(!in)throw std::runtime_error("Missing file: "+p.string());Json j;in>>j;return j;}
// Schema 3: lengths, SHA256(raw), XXH3-128(raw), optional lossless LZ4 block.
// Strict verification always checks SHA256. --fast still checks XXH3-128 on every record.
inline void writeRecord(std::ofstream &out,const std::vector<char>&b,bool compressed){
  uint64_t raw=b.size();std::vector<char> packed;
  uint64_t n=raw;
  if(compressed){packed.resize(LZ4_compressBound(static_cast<int>(raw)));
    const int size=LZ4_compress_default(b.data(),packed.data(),static_cast<int>(raw),static_cast<int>(packed.size()));
    if(size<=0)throw std::runtime_error("LZ4 compression failed");
    n=static_cast<uint64_t>(size);
  }
  if(!compressed||n>=raw){n=raw;packed=b;}
  auto h=sha(b.data(),b.size());auto fast=XXH3_128bits(b.data(),b.size());
  out.write(reinterpret_cast<const char *>(&n),sizeof n);out.write(reinterpret_cast<const char *>(&raw),sizeof raw);
  out.write(h.data(),64);out.write(reinterpret_cast<const char *>(&fast.low64),8);out.write(reinterpret_cast<const char *>(&fast.high64),8);
  out.write(packed.data(),n);if(!out)throw std::runtime_error("Cache write failed");
}
inline std::vector<char> readRecord(std::ifstream &in,bool strict=true){
  uint64_t n=0,raw=0;in.read(reinterpret_cast<char *>(&n),sizeof n);in.read(reinterpret_cast<char *>(&raw),sizeof raw);
  if(!in||n>(uint64_t(1)<<30)||raw>(uint64_t(1)<<30))throw std::runtime_error("Invalid cache record length");
  std::string h(64,' ');XXH128_hash_t expected{};in.read(h.data(),64);
  in.read(reinterpret_cast<char *>(&expected.low64),8);in.read(reinterpret_cast<char *>(&expected.high64),8);
  std::vector<char>packed(n);in.read(packed.data(),n);if(!in)throw std::runtime_error("Truncated cache record");
  std::vector<char>b;
  if(n==raw)b=std::move(packed);
  else {b.resize(raw);if(LZ4_decompress_safe(packed.data(),b.data(),static_cast<int>(n),static_cast<int>(raw))!=static_cast<int>(raw))throw std::runtime_error("LZ4 decompression failed");}
  auto actual=XXH3_128bits(b.data(),b.size());
  if(actual.low64!=expected.low64||actual.high64!=expected.high64||(strict&&sha(b.data(),b.size())!=h))throw std::runtime_error("Cache checksum error");
  return b;
}
}
