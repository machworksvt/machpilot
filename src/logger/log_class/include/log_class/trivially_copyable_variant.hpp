#ifndef TRIVIALLY_COPYABLE_VARIANT_HPP
#define TRIVIALLY_COPYABLE_VARIANT_HPP

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <type_traits>
#include <utility>

/**
 * @brief Type used for indexing and identifying the current active type in the variant.
 */
using TagType = std::uint64_t;

// --- Recursive Union Storage ---

template <typename... Ts>
union VariantStorage {};

template <typename T, typename... Ts>
union VariantStorage<T, Ts...> {
  T head;
  VariantStorage<Ts...> rest;

  // The constructor is needed or it will not compile if the subtypes do not 
  // have default constructors.
  VariantStorage() {}
};

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

/**
 * @brief A type-safe union for Trivially Copyable types.
 * * Provides a fixed-layout container suitable for direct binary serialization 
 * to disk. All held types must satisfy triviality and standard layout requirements.
 * * @tparam Ts The set of types the variant can hold.
 */
template <typename... Ts>
struct TriviallyCopyableVariant {
  // --- Assertions ---
  static_assert((std::is_trivially_copyable_v<Ts> && ...),
                "All types must be trivial for file serialization!");
  static_assert((std::is_standard_layout_v<Ts> && ...),
                "All types must have standard layout!");
  static_assert((std::is_trivially_destructible_v<Ts> && ...),
                "Types inside a trivial union must be trivially destructible");

  /// Number of types in the variant.
  static constexpr TagType TypeCount = sizeof...(Ts);
  /// Array of sizes for each type in the variant.
  static constexpr std::size_t Sizes[] = {sizeof(Ts)...};
  /// Total memory footprint of the internal storage.
  static constexpr std::size_t MaxSize = sizeof(VariantStorage<Ts...>);

  /// Current active type index.
  TagType type_id;
  /// Internal storage for the variant data.
  VariantStorage<Ts...> storage;

  // --- Constructors ---
  TriviallyCopyableVariant() = delete;

  /**
   * @brief Construct the variant with a value of type T.
   */
  template <typename T>
  TriviallyCopyableVariant(const T& val) : type_id(get_tag<T>()) {
    // Using memcpy instead of get_unchecked because obtaining a pointer to an 
    // inactive member of the union is undefined behavior. 
    // We static_cast storage to void* because otherwise we get a warning that 
    // VariantStorage is non-trivial (due to the manual constructor).
    // This is safe because any bit pattern is valid for storage bytes.
    std::memcpy(static_cast<void*>(&storage), &val, sizeof(T));
  }

  /**
   * @brief Assign a new value of type T to the variant.
   */
  template <typename T>
  TriviallyCopyableVariant& operator=(const T& val) {
    type_id = get_tag<T>();
    // Using memmove instead of memcpy because the user could pass in a 
    // reference to the internal storage (self-assignment).
    std::memmove(&storage, &val, sizeof(T));
    return *this;
  }

  // --- Accessors ---

  /**
   * @brief Extracts a pointer to type T if it is the currently active type.
   * @return T* or nullptr if the tag does not match.
   */
  template <typename T>
  T* get() {
    return (type_id == get_tag<T>()) ? get_unchecked<T>() : nullptr;
  }

  /**
   * @brief Extracts a const pointer to type T if it is the currently active type.
   * @return const T* or nullptr if the tag does not match.
   */
  template <typename T>
  const T* get() const {
    return (type_id == get_tag<T>()) ? get_unchecked<T>() : nullptr;
  }

  /**
   * @brief Extracts a pointer to type T without checking the active tag.
   * @warning Calling this with a type T that is not active is Undefined Behavior.
   */
  template <typename T>
  T* get_unchecked() {
    return reinterpret_cast<T*>(&storage);
  }

  /**
   * @brief Extracts a const pointer to type T without checking the active tag.
   * @warning Calling this with a type T that is not active is Undefined Behavior.
   */
  template <typename T>
  const T* get_unchecked() const {
    return reinterpret_cast<const T*>(&storage);
  }

  /**
   * @brief Returns the static tag associated with type T.
   */
  template <typename T>
  constexpr static TagType get_tag() {
    return TypeIndex<T, Ts...>::value;
  }

  // --- Visitor ---

  /**
   * @brief Dispatches the active type to the provided visitor.
   * @param visitor A callable that accepts all types Ts...
   */
  template <typename Visitor>
  decltype(auto) visit(Visitor&& visitor) {
    using ReturnType =
        std::common_type_t<std::invoke_result_t<Visitor, Ts&>...>;

    using StorageType = VariantStorage<Ts...>;
    using FuncPtr = ReturnType (*)(StorageType*, Visitor&&);

    // Jump table containing generated functions that cast the internal storage 
    // to the specific subtype before calling the visitor.
    static constexpr FuncPtr table[] = {
        +[](StorageType* s, Visitor&& vis) -> ReturnType {
          return vis(*reinterpret_cast<Ts*>(s));
        }...};

    return table[type_id](&storage, std::forward<Visitor>(visitor));
  }

  /**
   * @brief Dispatches the active type to the provided visitor (const version).
   */
  template <typename Visitor>
  decltype(auto) visit(Visitor&& visitor) const {
    using ReturnType =
        std::common_type_t<std::invoke_result_t<Visitor, const Ts&>...>;

    using StorageType = VariantStorage<Ts...>;
    using FuncPtr = ReturnType (*)(const StorageType*, Visitor&&);

    static constexpr FuncPtr table[] = {
        +[](const StorageType* s, Visitor&& vis) -> ReturnType {
          return vis(*reinterpret_cast<const Ts*>(s));
        }...};

    return table[type_id](&storage, std::forward<Visitor>(visitor));
  }
};

#endif  // TRIVIALLY_COPYABLE_VARIANT_HPP