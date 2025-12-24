#ifndef TRIVIALLY_COPYABLE_VARIANT_HPP
#define TRIVIALLY_COPYABLE_VARIANT_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <utility>

using TagType = std::uint64_t;

// --- Recursive Union Storage ---

// Forward declaration
template <typename... Ts>
union VariantStorage {};

// Recursive Case: Head + Rest
template <typename T, typename... Ts>
union VariantStorage<T, Ts...> {
  T head;
  VariantStorage<Ts...> rest;

  VariantStorage() {}
};

// Base Case: Single element (stops recursion)
template <typename T>
union VariantStorage<T> {
  T head;

  VariantStorage() {}
};

// --- Type Index Helper ---
template <typename T, typename... Types>
struct TypeIndex;

template <typename T, typename U, typename... Types>
struct TypeIndex<T, U, Types...> {
  static constexpr TagType value = 1 + TypeIndex<T, Types...>::value;
};

template <typename T, typename... Types>
struct TypeIndex<T, T, Types...> {
  static constexpr TagType value = 0;
};

// --- The Variant ---
template <typename... Ts>
struct TriviallyCopyableVariant {
  // --- Assertions ---
  static_assert((std::is_trivially_copyable_v<Ts> && ...),
                "All types must be trivial for file serialization!");
  static_assert((std::is_standard_layout_v<Ts> && ...),
                "All types must have standard layout!");
  static_assert((std::is_trivially_destructible_v<Ts> && ...),
                "Types inside a trivial union must be trivially destructible");

  static constexpr TagType TypeCount = sizeof...(Ts);
  static constexpr std::size_t Sizes[] = {sizeof(Ts)...};
  static constexpr std::size_t MaxSize = sizeof(VariantStorage<Ts...>);

  // The Tag
  TagType type_id;
  // The Storage (Recursive Union)
  VariantStorage<Ts...> storage;

  // --- Constructors ---
  TriviallyCopyableVariant() = delete;

  template <typename T>
  TriviallyCopyableVariant(const T& val) : type_id(get_tag<T>()) {
    // we are using memcpy and not get_unchecked becuase that mean getting a
    // pointer to a inactive member of the union with is undefined we
    // static_cast storage to void because else we get a warning that
    // VaraintStoange is non trival. It is non trival becuase we have a
    // contructor. The contructor is needed or it will not compile if the
    // subtypes do not have default contructor. This is safe becuase we know
    // that val plus arbitrary padding is a valid bit pattern for storage
    std::memcpy(static_cast<void*>(&storage), &val, sizeof(T));
  }

  template <typename T>
  TriviallyCopyableVariant& operator=(const T& val) {
    type_id = get_tag<T>();
    // we are using memmove and not get_unchecked becuase that mean getting a
    // pointer to a inactive member of the union with is undefined we are using
    // memmove and not memcpy becuase a user could pass in a reference to the
    // internal storange into this function
    std::memmove(&storage, &val, sizeof(T));
    return *this;
  }

  // --- Accessors ---

  // get extract type T from variant will return null if type T is not stored in
  // the variant
  template <typename T>
  T* get() {
    return (type_id == get_tag<T>()) ? get_unchecked<T>() : nullptr;
  }

  // get extract type T from variant will return null if type T is not stored in
  // the variant
  template <typename T>
  const T* get() const {
    return (type_id == get_tag<T>()) ? get_unchecked<T>() : nullptr;
  }

  // get extract type T from variant will be undefined behavior if type T is not
  // stored in the variant
  template <typename T>
  T* get_unchecked() {
    return reinterpret_cast<T*>(&storage);
  }

  // get extract type T from variant will be undefined behavior if type T is not
  // stored in the variant
  template <typename T>
  const T* get_unchecked() const {
    return reinterpret_cast<const T*>(&storage);
  }

  // get the tag associated with the type
  template <typename T>
  constexpr static TagType get_tag() {
    return TypeIndex<T, Ts...>::value;
  }

  // --- Visitor ---

  // calls the function passed into visit with the current thing stored in the
  // variant
  template <typename Visitor>
  decltype(auto) visit(Visitor&& visitor) {
    using ReturnType =
        std::common_type_t<std::invoke_result_t<Visitor, Ts&>...>;

    using StorageType = VariantStorage<Ts...>;
    using FuncPtr = ReturnType (*)(StorageType*, Visitor&&);

    static constexpr FuncPtr table[] = {
        // a function that takes in storange type and visitor and cast the
        // storange to a sub type calls then calls visitor we generate this
        // function for each subtype
        +[](StorageType* s, Visitor&& vis) -> ReturnType {
          return vis(*reinterpret_cast<Ts*>(s));
        }...};
    // call the function pointer with the type assocated with the type_id
    return table[type_id](&storage, std::forward<Visitor>(visitor));
  }

  // calls the function passed into visit with the current thing stored in the
  // variant
  template <typename Visitor>
  decltype(auto) visit(Visitor&& visitor) const {
    using ReturnType =
        std::common_type_t<std::invoke_result_t<Visitor, const Ts&>...>;

    using StorageType = VariantStorage<Ts...>;
    using FuncPtr = ReturnType (*)(const StorageType*, Visitor&&);

    static constexpr FuncPtr table[] = {
        // a function that takes in storange type and visitor and cast the
        // storange to a sub type calls then calls visitor we generate this
        // function for each subtype
        +[](const StorageType* s, Visitor&& vis) -> ReturnType {
          return vis(*reinterpret_cast<const Ts*>(s));
        }...};
    // call the function pointer with the type assocated with the type_id
    return table[type_id](&storage, std::forward<Visitor>(visitor));
  }
};

#endif  // TRIVIALLY_COPYABLE_VARIANT_HPP