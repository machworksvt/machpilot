#include <iostream>

#include "severity.hpp"
#include "source.hpp"
#include "trivially_copyable_variant.hpp"

class Heartbeat {
  friend std::ostream& operator<<(std::ostream& os, const Heartbeat& log);
};

class LoggerStartup {
  friend std::ostream& operator<<(std::ostream& os, const LoggerStartup& log);
};

class InvaidDeserializationBadTag {
  friend std::ostream& operator<<(std::ostream& os,
                                  const InvaidDeserializationBadTag& log);

 public:
  InvaidDeserializationBadTag(TagType tag) : tag(tag) {}
  TagType tag;
};

class InvalidDeserializationBadVariantSize {
  friend std::ostream& operator<<(
      std::ostream& os, const InvalidDeserializationBadVariantSize& log);

 public:
  InvalidDeserializationBadVariantSize(TagType tag, std::uint64_t correct_size,
                                       std::uint64_t given_size)
      : tag(tag), correct_size(correct_size), given_size(given_size) {}
  TagType tag;
  std::uint64_t correct_size;
  std::uint64_t given_size;
};

class InvalidDeserializationBadSeverity{
  friend std::ostream& operator<<(
      std::ostream& os, const InvalidDeserializationBadSeverity& log);
  
  public:
    InvalidDeserializationBadSeverity(std::underlying_type_t<Severity> given_id)
      : given_id(given_id) {}

  std::underlying_type_t<Severity> given_id;
};

class InvalidDeserializationBadSouce{
  friend std::ostream& operator<<(
      std::ostream& os, const InvalidDeserializationBadSouce& log);
  
  public:
    InvalidDeserializationBadSouce(std::underlying_type_t<Source> given_id)
      : given_id(given_id) {}
      
  std::underlying_type_t<Source> given_id;
};