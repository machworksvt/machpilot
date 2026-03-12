#include <cstdint>

#include "log_class/message.hpp"
#include "log_class/time_stamp.hpp"
#include "log_class/trivially_copyable_variant.hpp"

/**
 * @brief Current version of the log file format.
 * Used to ensure backward compatibility or detect breaking changes in layout.
 */
const std::uint64_t VERSION = 0;

/**
 * @brief Fixed-size header for the log file metadata.
 * * This struct contains the fundamental information required to start 
 * parsing a log file. Because its size is constant, it can be safely 
 * read into a stack-allocated buffer during deserialization.
 */
struct FileMetaDataPreamble {
  /// File format version identifier.
  std::uint64_t version;
  /// The time when the log file was created or opened.
  time_stamp time;
  /// Number of unique variant types supported by this file's schema.
  TagType variant_count;

  /**
   * @brief Default constructor initializing a zeroed preamble.
   */
  FileMetaDataPreamble() : time(), variant_count() {}

  /**
   * @brief Constructs a preamble with current time and specific variant count.
   * @param variant_count The number of types in the associated variant.
   */
  FileMetaDataPreamble(int64_t variant_count)
      : time(std::chrono::system_clock::now()), variant_count(variant_count) {
    this->version = VERSION;
  }
};

/**
 * @brief Template helper for metadata generation (Primary Template).
 */
template <typename>
struct FileMetaDataGeneric;

/**
 * @brief Specialized metadata structure for a specific TriviallyCopyableVariant.
 * * This structure is written to the head of the log file. It maps every 
 * possible @c TagType to its expected byte-size, enabling the reader to 
 * validate and recover from corrupted log entries.
 * * @tparam Ts The parameter pack of types stored in the variant.
 */
template <typename... Ts>
struct FileMetaDataGeneric<TriviallyCopyableVariant<Ts...>> {
  /// The fixed-size portion of the metadata.
  FileMetaDataPreamble pre;
  /// An array mapping variant tags to their respective sizes in bytes.
  int64_t variant_sizes[sizeof...(Ts)];

  /**
   * @brief Initializes metadata based on the sizes of types in the variant.
   */
  FileMetaDataGeneric() : pre(sizeof...(Ts)), variant_sizes{sizeof(Ts)...} {}
};

/**
 * @brief The specific FileMetaData type used by the logging system.
 */
typedef FileMetaDataGeneric<LogInner> FileMetaData;