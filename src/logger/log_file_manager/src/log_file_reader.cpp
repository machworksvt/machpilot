#include <assert.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "file_meta_data.hpp"
#include "log_class/message.hpp"
#include "log_class/trivially_copyable_variant.hpp"

enum class LOG_READ_RESULT { OK, Done, FatalError };

// read a single log from file returns LOG_READ_RESULT::OK if buffer is filled
// LOG_READ_RESULT::Done if there is no more data in this case buffer will not
// be filled LOG_READ_RESULT::FatalError if the file is corrupted in a way that
// is not recoverable bytes
LOG_READ_RESULT read_single_log(
    Log* buffer, std::ifstream& file,
    const std::vector<std::uint64_t>& variant_sizes) {
  // a macro to read from file and return LOG_READ_RESULT::FatalError if there is
  // not enough data in file
  #define try_read(pos, size)                                   \
    file.read(reinterpret_cast<char*>(pos), size);              \
    if ((std::uint64_t)file.gcount() != (std::uint64_t)size) {  \
      std::cerr << "partial log file encountered" << std::endl; \
      return LOG_READ_RESULT::FatalError;                       \
    }

  // check if the file is empty
  if (file.peek() == std::ifstream::traits_type::eof()) {
    return LOG_READ_RESULT::Done;
  }

  // read all constant size stuff into buffer
  try_read(&buffer->time, sizeof(time_stamp));

  // we can't read source or severity directly into buffer becuase they are a enum
  // class and writting data that does not corsipond to a type into a enm class is
  // undefined behavior so we read into a int check the int then write into buffer

  std::underlying_type_t<Severity> severity_int;
  try_read(&severity_int, sizeof(Severity));
  std::underlying_type_t<Source> source_int;
  try_read(&source_int, sizeof(Source));


  try_read(&buffer->sub_log.type_id, sizeof(TagType));

  TagType tag = buffer->sub_log.type_id;

  //check if the flag is to big for the meta data
  if (tag >= variant_sizes.size()) {
    std::cerr << "invalid tag " << tag << " meta data implies the max tag is "
              << variant_sizes.size() - 1;
    // we don't know how long this log is so we can't just skip to the next log
    return LOG_READ_RESULT::FatalError;
  }

  std::size_t size_from_meta_data = variant_sizes[tag];

  // After this point all found errors will not be fatal that is becuase even though
  // the log entry might be corrupted the size of the log was recoverable meaning we 
  // we can just skip the bytes assosated with this log this prevents some bugs from
  // invalidating a entire file

  // check severity
  if (severity_int<SEVERITY_SIZE){
    buffer->severity=static_cast<Severity>(severity_int);
  }else{
    buffer->severity=Severity::Error;
    buffer->source=Source::Logger;
    file.seekg(size_from_meta_data, std::ios_base::cur);
    buffer->sub_log = InvalidDeserializationBadSeverity(severity_int);
    return LOG_READ_RESULT::OK;
  }

  // check source
  if (source_int<SOURCE_SIZE){
    buffer->source=static_cast<Source>(source_int);
  }else{
    buffer->source=Source::Logger;
    file.seekg(size_from_meta_data, std::ios_base::cur);
    buffer->sub_log = InvalidDeserializationBadSouce(source_int);
    return LOG_READ_RESULT::OK;
  }

  // check tag
  if (tag >= LogInner::TypeCount) {
    file.seekg(size_from_meta_data, std::ios_base::cur);
    buffer->sub_log = InvaidDeserializationBadTag(tag);
    return LOG_READ_RESULT::OK;
  }

  // check sizes match
  std::size_t size_from_known_sizes = LogInner::Sizes[tag];
  if (size_from_meta_data != size_from_known_sizes) {
    file.seekg(size_from_meta_data, std::ios_base::cur);
    buffer->sub_log = InvalidDeserializationBadVariantSize(
        tag, size_from_known_sizes, size_from_meta_data);
    return LOG_READ_RESULT::OK;
  }

  try_read(&buffer->sub_log.storage, size_from_meta_data);

  return LOG_READ_RESULT::OK;
}

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cout << "Expected path to file to read" << std::endl;
    return 1;
  }

  std::ifstream file(argv[1], std::ios::in | std::ios::binary);

  if (!file) {
    std::cerr << "Error opening file." << std::endl;
    return 1;
  }

  FileMetaDataPreamble meta_data;

  // read in the statically size part of the meta data
  file.read(reinterpret_cast<char*>(&meta_data), sizeof(FileMetaDataPreamble));

  if ((std::uint64_t)file.gcount() !=
      (std::uint64_t)sizeof(FileMetaDataPreamble)) {
    std::cerr << "meta data was cuttoff" << std::endl;
    return 1;
  }

  if (meta_data.version != VERSION){
    std::cerr << "Wrong Version: metadata indicates made with version " << meta_data.version << " this software parses version " << VERSION << std::endl;
    return 1;
  }

  if (meta_data.variant_count > 1024) {
    std::cerr << "Corrupted metadata: unreasonable variant count. "<< meta_data.variant_count << std::endl;
    return 1;
  }

  std::vector<uint64_t> variant_sizes(meta_data.variant_count);

  // read in the non-statically size part of the meta data
  file.read(reinterpret_cast<char*>(variant_sizes.data()),
            sizeof(uint64_t) * meta_data.variant_count);

  if ((uint64_t)file.gcount() !=
      (uint64_t)(sizeof(uint64_t) * meta_data.variant_count)) {
    std::cerr << "meta data was cuttoff" << std::endl;
    return 1;
  }

  std::cout << Log(meta_data.time, Severity::Log, Source::Logger, LoggerStartup{}) << std::endl;

  // the buffer we will read into
  Log buffer;

  while (true) {
    switch (read_single_log(&buffer, file, variant_sizes)) {
      case LOG_READ_RESULT::OK:
        break;
      case LOG_READ_RESULT::FatalError:
        return 1;
      case LOG_READ_RESULT::Done:
        return 0;
    }

    std::cout << buffer << std::endl;
  }
}