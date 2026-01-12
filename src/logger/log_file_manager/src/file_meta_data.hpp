#include <cstdint>

#include "log_class/message.hpp"
#include "log_class/time_stamp.hpp"
#include "log_class/trivially_copyable_variant.hpp"

// the part of FileMetaDataInner that will always be the same size so this can
// be stack allocated durring deserialization
struct FileMetaDataPreamble {
  time_stamp time;
  TagType variant_count;

  FileMetaDataPreamble() : time(), variant_count() {}

  FileMetaDataPreamble(int64_t variant_count)
      : time(std::chrono::system_clock::now()), variant_count(variant_count) {}
};

template <typename>
struct FileMetaDataInner;

// data put at the begining of a file to describe the file
template <typename... Ts>
struct FileMetaDataInner<TriviallyCopyableVariant<Ts...>> {
  FileMetaDataPreamble pre;
  int64_t variant_sizes[sizeof...(Ts)];

  FileMetaDataInner() : pre(sizeof...(Ts)), variant_sizes{sizeof(Ts)...} {}
};

// in order to make FileMetaData that is tied to are log type we make
// FileMetaDataInner witch can be tied to any type then we make FileMeta data a
// version of that that is tied to our log type
typedef FileMetaDataInner<LogInner> FileMetaData;