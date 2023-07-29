// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: proto/student.proto

#ifndef GOOGLE_PROTOBUF_INCLUDED_proto_2fstudent_2eproto
#define GOOGLE_PROTOBUF_INCLUDED_proto_2fstudent_2eproto

#include <limits>
#include <string>

#include <google/protobuf/port_def.inc>
#if PROTOBUF_VERSION < 3019000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers. Please update
#error your headers.
#endif
#if 3019004 < PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers. Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/port_undef.inc>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/arena.h>
#include <google/protobuf/arenastring.h>
#include <google/protobuf/generated_message_table_driven.h>
#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/metadata_lite.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>  // IWYU pragma: export
#include <google/protobuf/extension_set.h>  // IWYU pragma: export
#include <google/protobuf/generated_enum_reflection.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>
#define PROTOBUF_INTERNAL_EXPORT_proto_2fstudent_2eproto
PROTOBUF_NAMESPACE_OPEN
namespace internal {
class AnyMetadata;
}  // namespace internal
PROTOBUF_NAMESPACE_CLOSE

// Internal implementation detail -- do not use these members.
struct TableStruct_proto_2fstudent_2eproto {
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTableField entries[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::AuxiliaryParseTableField aux[]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::ParseTable schema[2]
    PROTOBUF_SECTION_VARIABLE(protodesc_cold);
  static const ::PROTOBUF_NAMESPACE_ID::internal::FieldMetadata field_metadata[];
  static const ::PROTOBUF_NAMESPACE_ID::internal::SerializationTable serialization_table[];
  static const uint32_t offsets[];
};
extern const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_proto_2fstudent_2eproto;
namespace protobuf_learning {
class Student;
struct StudentDefaultTypeInternal;
extern StudentDefaultTypeInternal _Student_default_instance_;
class Student_PhoneNumber;
struct Student_PhoneNumberDefaultTypeInternal;
extern Student_PhoneNumberDefaultTypeInternal _Student_PhoneNumber_default_instance_;
}  // namespace protobuf_learning
PROTOBUF_NAMESPACE_OPEN
template<> ::protobuf_learning::Student* Arena::CreateMaybeMessage<::protobuf_learning::Student>(Arena*);
template<> ::protobuf_learning::Student_PhoneNumber* Arena::CreateMaybeMessage<::protobuf_learning::Student_PhoneNumber>(Arena*);
PROTOBUF_NAMESPACE_CLOSE
namespace protobuf_learning {

enum Student_PhoneType : int {
  Student_PhoneType_MOBILE = 0,
  Student_PhoneType_HOME = 1,
  Student_PhoneType_Student_PhoneType_INT_MIN_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::min(),
  Student_PhoneType_Student_PhoneType_INT_MAX_SENTINEL_DO_NOT_USE_ = std::numeric_limits<int32_t>::max()
};
bool Student_PhoneType_IsValid(int value);
constexpr Student_PhoneType Student_PhoneType_PhoneType_MIN = Student_PhoneType_MOBILE;
constexpr Student_PhoneType Student_PhoneType_PhoneType_MAX = Student_PhoneType_HOME;
constexpr int Student_PhoneType_PhoneType_ARRAYSIZE = Student_PhoneType_PhoneType_MAX + 1;

const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Student_PhoneType_descriptor();
template<typename T>
inline const std::string& Student_PhoneType_Name(T enum_t_value) {
  static_assert(::std::is_same<T, Student_PhoneType>::value ||
    ::std::is_integral<T>::value,
    "Incorrect type passed to function Student_PhoneType_Name.");
  return ::PROTOBUF_NAMESPACE_ID::internal::NameOfEnum(
    Student_PhoneType_descriptor(), enum_t_value);
}
inline bool Student_PhoneType_Parse(
    ::PROTOBUF_NAMESPACE_ID::ConstStringParam name, Student_PhoneType* value) {
  return ::PROTOBUF_NAMESPACE_ID::internal::ParseNamedEnum<Student_PhoneType>(
    Student_PhoneType_descriptor(), name, value);
}
// ===================================================================

class Student_PhoneNumber final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:protobuf_learning.Student.PhoneNumber) */ {
 public:
  inline Student_PhoneNumber() : Student_PhoneNumber(nullptr) {}
  ~Student_PhoneNumber() override;
  explicit constexpr Student_PhoneNumber(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Student_PhoneNumber(const Student_PhoneNumber& from);
  Student_PhoneNumber(Student_PhoneNumber&& from) noexcept
    : Student_PhoneNumber() {
    *this = ::std::move(from);
  }

  inline Student_PhoneNumber& operator=(const Student_PhoneNumber& from) {
    CopyFrom(from);
    return *this;
  }
  inline Student_PhoneNumber& operator=(Student_PhoneNumber&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Student_PhoneNumber& default_instance() {
    return *internal_default_instance();
  }
  static inline const Student_PhoneNumber* internal_default_instance() {
    return reinterpret_cast<const Student_PhoneNumber*>(
               &_Student_PhoneNumber_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    0;

  friend void swap(Student_PhoneNumber& a, Student_PhoneNumber& b) {
    a.Swap(&b);
  }
  inline void Swap(Student_PhoneNumber* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Student_PhoneNumber* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Student_PhoneNumber* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Student_PhoneNumber>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Student_PhoneNumber& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Student_PhoneNumber& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Student_PhoneNumber* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "protobuf_learning.Student.PhoneNumber";
  }
  protected:
  explicit Student_PhoneNumber(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  enum : int {
    kNumberFieldNumber = 1,
    kTypeFieldNumber = 2,
  };
  // string number = 1;
  void clear_number();
  const std::string& number() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_number(ArgT0&& arg0, ArgT... args);
  std::string* mutable_number();
  PROTOBUF_NODISCARD std::string* release_number();
  void set_allocated_number(std::string* number);
  private:
  const std::string& _internal_number() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_number(const std::string& value);
  std::string* _internal_mutable_number();
  public:

  // optional .protobuf_learning.Student.PhoneType type = 2;
  bool has_type() const;
  private:
  bool _internal_has_type() const;
  public:
  void clear_type();
  ::protobuf_learning::Student_PhoneType type() const;
  void set_type(::protobuf_learning::Student_PhoneType value);
  private:
  ::protobuf_learning::Student_PhoneType _internal_type() const;
  void _internal_set_type(::protobuf_learning::Student_PhoneType value);
  public:

  // @@protoc_insertion_point(class_scope:protobuf_learning.Student.PhoneNumber)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr number_;
  int type_;
  friend struct ::TableStruct_proto_2fstudent_2eproto;
};
// -------------------------------------------------------------------

class Student final :
    public ::PROTOBUF_NAMESPACE_ID::Message /* @@protoc_insertion_point(class_definition:protobuf_learning.Student) */ {
 public:
  inline Student() : Student(nullptr) {}
  ~Student() override;
  explicit constexpr Student(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized);

  Student(const Student& from);
  Student(Student&& from) noexcept
    : Student() {
    *this = ::std::move(from);
  }

  inline Student& operator=(const Student& from) {
    CopyFrom(from);
    return *this;
  }
  inline Student& operator=(Student&& from) noexcept {
    if (this == &from) return *this;
    if (GetOwningArena() == from.GetOwningArena()
  #ifdef PROTOBUF_FORCE_COPY_IN_MOVE
        && GetOwningArena() != nullptr
  #endif  // !PROTOBUF_FORCE_COPY_IN_MOVE
    ) {
      InternalSwap(&from);
    } else {
      CopyFrom(from);
    }
    return *this;
  }

  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* descriptor() {
    return GetDescriptor();
  }
  static const ::PROTOBUF_NAMESPACE_ID::Descriptor* GetDescriptor() {
    return default_instance().GetMetadata().descriptor;
  }
  static const ::PROTOBUF_NAMESPACE_ID::Reflection* GetReflection() {
    return default_instance().GetMetadata().reflection;
  }
  static const Student& default_instance() {
    return *internal_default_instance();
  }
  static inline const Student* internal_default_instance() {
    return reinterpret_cast<const Student*>(
               &_Student_default_instance_);
  }
  static constexpr int kIndexInFileMessages =
    1;

  friend void swap(Student& a, Student& b) {
    a.Swap(&b);
  }
  inline void Swap(Student* other) {
    if (other == this) return;
  #ifdef PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() != nullptr &&
        GetOwningArena() == other->GetOwningArena()) {
   #else  // PROTOBUF_FORCE_COPY_IN_SWAP
    if (GetOwningArena() == other->GetOwningArena()) {
  #endif  // !PROTOBUF_FORCE_COPY_IN_SWAP
      InternalSwap(other);
    } else {
      ::PROTOBUF_NAMESPACE_ID::internal::GenericSwap(this, other);
    }
  }
  void UnsafeArenaSwap(Student* other) {
    if (other == this) return;
    GOOGLE_DCHECK(GetOwningArena() == other->GetOwningArena());
    InternalSwap(other);
  }

  // implements Message ----------------------------------------------

  Student* New(::PROTOBUF_NAMESPACE_ID::Arena* arena = nullptr) const final {
    return CreateMaybeMessage<Student>(arena);
  }
  using ::PROTOBUF_NAMESPACE_ID::Message::CopyFrom;
  void CopyFrom(const Student& from);
  using ::PROTOBUF_NAMESPACE_ID::Message::MergeFrom;
  void MergeFrom(const Student& from);
  private:
  static void MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to, const ::PROTOBUF_NAMESPACE_ID::Message& from);
  public:
  PROTOBUF_ATTRIBUTE_REINITIALIZES void Clear() final;
  bool IsInitialized() const final;

  size_t ByteSizeLong() const final;
  const char* _InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) final;
  uint8_t* _InternalSerialize(
      uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const final;
  int GetCachedSize() const final { return _cached_size_.Get(); }

  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const final;
  void InternalSwap(Student* other);

  private:
  friend class ::PROTOBUF_NAMESPACE_ID::internal::AnyMetadata;
  static ::PROTOBUF_NAMESPACE_ID::StringPiece FullMessageName() {
    return "protobuf_learning.Student";
  }
  protected:
  explicit Student(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                       bool is_message_owned = false);
  private:
  static void ArenaDtor(void* object);
  inline void RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena* arena);
  public:

  static const ClassData _class_data_;
  const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*GetClassData() const final;

  ::PROTOBUF_NAMESPACE_ID::Metadata GetMetadata() const final;

  // nested types ----------------------------------------------------

  typedef Student_PhoneNumber PhoneNumber;

  typedef Student_PhoneType PhoneType;
  static constexpr PhoneType MOBILE =
    Student_PhoneType_MOBILE;
  static constexpr PhoneType HOME =
    Student_PhoneType_HOME;
  static inline bool PhoneType_IsValid(int value) {
    return Student_PhoneType_IsValid(value);
  }
  static constexpr PhoneType PhoneType_MIN =
    Student_PhoneType_PhoneType_MIN;
  static constexpr PhoneType PhoneType_MAX =
    Student_PhoneType_PhoneType_MAX;
  static constexpr int PhoneType_ARRAYSIZE =
    Student_PhoneType_PhoneType_ARRAYSIZE;
  static inline const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor*
  PhoneType_descriptor() {
    return Student_PhoneType_descriptor();
  }
  template<typename T>
  static inline const std::string& PhoneType_Name(T enum_t_value) {
    static_assert(::std::is_same<T, PhoneType>::value ||
      ::std::is_integral<T>::value,
      "Incorrect type passed to function PhoneType_Name.");
    return Student_PhoneType_Name(enum_t_value);
  }
  static inline bool PhoneType_Parse(::PROTOBUF_NAMESPACE_ID::ConstStringParam name,
      PhoneType* value) {
    return Student_PhoneType_Parse(name, value);
  }

  // accessors -------------------------------------------------------

  enum : int {
    kPhoneFieldNumber = 4,
    kNameFieldNumber = 2,
    kEmailFieldNumber = 3,
    kIdFieldNumber = 1,
  };
  // repeated .protobuf_learning.Student.PhoneNumber phone = 4;
  int phone_size() const;
  private:
  int _internal_phone_size() const;
  public:
  void clear_phone();
  ::protobuf_learning::Student_PhoneNumber* mutable_phone(int index);
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::protobuf_learning::Student_PhoneNumber >*
      mutable_phone();
  private:
  const ::protobuf_learning::Student_PhoneNumber& _internal_phone(int index) const;
  ::protobuf_learning::Student_PhoneNumber* _internal_add_phone();
  public:
  const ::protobuf_learning::Student_PhoneNumber& phone(int index) const;
  ::protobuf_learning::Student_PhoneNumber* add_phone();
  const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::protobuf_learning::Student_PhoneNumber >&
      phone() const;

  // string name = 2;
  void clear_name();
  const std::string& name() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_name(ArgT0&& arg0, ArgT... args);
  std::string* mutable_name();
  PROTOBUF_NODISCARD std::string* release_name();
  void set_allocated_name(std::string* name);
  private:
  const std::string& _internal_name() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_name(const std::string& value);
  std::string* _internal_mutable_name();
  public:

  // optional string email = 3;
  bool has_email() const;
  private:
  bool _internal_has_email() const;
  public:
  void clear_email();
  const std::string& email() const;
  template <typename ArgT0 = const std::string&, typename... ArgT>
  void set_email(ArgT0&& arg0, ArgT... args);
  std::string* mutable_email();
  PROTOBUF_NODISCARD std::string* release_email();
  void set_allocated_email(std::string* email);
  private:
  const std::string& _internal_email() const;
  inline PROTOBUF_ALWAYS_INLINE void _internal_set_email(const std::string& value);
  std::string* _internal_mutable_email();
  public:

  // uint64 id = 1;
  void clear_id();
  uint64_t id() const;
  void set_id(uint64_t value);
  private:
  uint64_t _internal_id() const;
  void _internal_set_id(uint64_t value);
  public:

  // @@protoc_insertion_point(class_scope:protobuf_learning.Student)
 private:
  class _Internal;

  template <typename T> friend class ::PROTOBUF_NAMESPACE_ID::Arena::InternalHelper;
  typedef void InternalArenaConstructable_;
  typedef void DestructorSkippable_;
  ::PROTOBUF_NAMESPACE_ID::internal::HasBits<1> _has_bits_;
  mutable ::PROTOBUF_NAMESPACE_ID::internal::CachedSize _cached_size_;
  ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::protobuf_learning::Student_PhoneNumber > phone_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr name_;
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr email_;
  uint64_t id_;
  friend struct ::TableStruct_proto_2fstudent_2eproto;
};
// ===================================================================


// ===================================================================

#ifdef __GNUC__
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wstrict-aliasing"
#endif  // __GNUC__
// Student_PhoneNumber

// string number = 1;
inline void Student_PhoneNumber::clear_number() {
  number_.ClearToEmpty();
}
inline const std::string& Student_PhoneNumber::number() const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.PhoneNumber.number)
  return _internal_number();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void Student_PhoneNumber::set_number(ArgT0&& arg0, ArgT... args) {
 
 number_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:protobuf_learning.Student.PhoneNumber.number)
}
inline std::string* Student_PhoneNumber::mutable_number() {
  std::string* _s = _internal_mutable_number();
  // @@protoc_insertion_point(field_mutable:protobuf_learning.Student.PhoneNumber.number)
  return _s;
}
inline const std::string& Student_PhoneNumber::_internal_number() const {
  return number_.Get();
}
inline void Student_PhoneNumber::_internal_set_number(const std::string& value) {
  
  number_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* Student_PhoneNumber::_internal_mutable_number() {
  
  return number_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* Student_PhoneNumber::release_number() {
  // @@protoc_insertion_point(field_release:protobuf_learning.Student.PhoneNumber.number)
  return number_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void Student_PhoneNumber::set_allocated_number(std::string* number) {
  if (number != nullptr) {
    
  } else {
    
  }
  number_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), number,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (number_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    number_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:protobuf_learning.Student.PhoneNumber.number)
}

// optional .protobuf_learning.Student.PhoneType type = 2;
inline bool Student_PhoneNumber::_internal_has_type() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Student_PhoneNumber::has_type() const {
  return _internal_has_type();
}
inline void Student_PhoneNumber::clear_type() {
  type_ = 0;
  _has_bits_[0] &= ~0x00000001u;
}
inline ::protobuf_learning::Student_PhoneType Student_PhoneNumber::_internal_type() const {
  return static_cast< ::protobuf_learning::Student_PhoneType >(type_);
}
inline ::protobuf_learning::Student_PhoneType Student_PhoneNumber::type() const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.PhoneNumber.type)
  return _internal_type();
}
inline void Student_PhoneNumber::_internal_set_type(::protobuf_learning::Student_PhoneType value) {
  _has_bits_[0] |= 0x00000001u;
  type_ = value;
}
inline void Student_PhoneNumber::set_type(::protobuf_learning::Student_PhoneType value) {
  _internal_set_type(value);
  // @@protoc_insertion_point(field_set:protobuf_learning.Student.PhoneNumber.type)
}

// -------------------------------------------------------------------

// Student

// uint64 id = 1;
inline void Student::clear_id() {
  id_ = uint64_t{0u};
}
inline uint64_t Student::_internal_id() const {
  return id_;
}
inline uint64_t Student::id() const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.id)
  return _internal_id();
}
inline void Student::_internal_set_id(uint64_t value) {
  
  id_ = value;
}
inline void Student::set_id(uint64_t value) {
  _internal_set_id(value);
  // @@protoc_insertion_point(field_set:protobuf_learning.Student.id)
}

// string name = 2;
inline void Student::clear_name() {
  name_.ClearToEmpty();
}
inline const std::string& Student::name() const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.name)
  return _internal_name();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void Student::set_name(ArgT0&& arg0, ArgT... args) {
 
 name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:protobuf_learning.Student.name)
}
inline std::string* Student::mutable_name() {
  std::string* _s = _internal_mutable_name();
  // @@protoc_insertion_point(field_mutable:protobuf_learning.Student.name)
  return _s;
}
inline const std::string& Student::_internal_name() const {
  return name_.Get();
}
inline void Student::_internal_set_name(const std::string& value) {
  
  name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* Student::_internal_mutable_name() {
  
  return name_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* Student::release_name() {
  // @@protoc_insertion_point(field_release:protobuf_learning.Student.name)
  return name_.Release(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
}
inline void Student::set_allocated_name(std::string* name) {
  if (name != nullptr) {
    
  } else {
    
  }
  name_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), name,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (name_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:protobuf_learning.Student.name)
}

// optional string email = 3;
inline bool Student::_internal_has_email() const {
  bool value = (_has_bits_[0] & 0x00000001u) != 0;
  return value;
}
inline bool Student::has_email() const {
  return _internal_has_email();
}
inline void Student::clear_email() {
  email_.ClearToEmpty();
  _has_bits_[0] &= ~0x00000001u;
}
inline const std::string& Student::email() const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.email)
  return _internal_email();
}
template <typename ArgT0, typename... ArgT>
inline PROTOBUF_ALWAYS_INLINE
void Student::set_email(ArgT0&& arg0, ArgT... args) {
 _has_bits_[0] |= 0x00000001u;
 email_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, static_cast<ArgT0 &&>(arg0), args..., GetArenaForAllocation());
  // @@protoc_insertion_point(field_set:protobuf_learning.Student.email)
}
inline std::string* Student::mutable_email() {
  std::string* _s = _internal_mutable_email();
  // @@protoc_insertion_point(field_mutable:protobuf_learning.Student.email)
  return _s;
}
inline const std::string& Student::_internal_email() const {
  return email_.Get();
}
inline void Student::_internal_set_email(const std::string& value) {
  _has_bits_[0] |= 0x00000001u;
  email_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, value, GetArenaForAllocation());
}
inline std::string* Student::_internal_mutable_email() {
  _has_bits_[0] |= 0x00000001u;
  return email_.Mutable(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, GetArenaForAllocation());
}
inline std::string* Student::release_email() {
  // @@protoc_insertion_point(field_release:protobuf_learning.Student.email)
  if (!_internal_has_email()) {
    return nullptr;
  }
  _has_bits_[0] &= ~0x00000001u;
  auto* p = email_.ReleaseNonDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (email_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    email_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  return p;
}
inline void Student::set_allocated_email(std::string* email) {
  if (email != nullptr) {
    _has_bits_[0] |= 0x00000001u;
  } else {
    _has_bits_[0] &= ~0x00000001u;
  }
  email_.SetAllocated(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), email,
      GetArenaForAllocation());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (email_.IsDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited())) {
    email_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  }
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  // @@protoc_insertion_point(field_set_allocated:protobuf_learning.Student.email)
}

// repeated .protobuf_learning.Student.PhoneNumber phone = 4;
inline int Student::_internal_phone_size() const {
  return phone_.size();
}
inline int Student::phone_size() const {
  return _internal_phone_size();
}
inline void Student::clear_phone() {
  phone_.Clear();
}
inline ::protobuf_learning::Student_PhoneNumber* Student::mutable_phone(int index) {
  // @@protoc_insertion_point(field_mutable:protobuf_learning.Student.phone)
  return phone_.Mutable(index);
}
inline ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::protobuf_learning::Student_PhoneNumber >*
Student::mutable_phone() {
  // @@protoc_insertion_point(field_mutable_list:protobuf_learning.Student.phone)
  return &phone_;
}
inline const ::protobuf_learning::Student_PhoneNumber& Student::_internal_phone(int index) const {
  return phone_.Get(index);
}
inline const ::protobuf_learning::Student_PhoneNumber& Student::phone(int index) const {
  // @@protoc_insertion_point(field_get:protobuf_learning.Student.phone)
  return _internal_phone(index);
}
inline ::protobuf_learning::Student_PhoneNumber* Student::_internal_add_phone() {
  return phone_.Add();
}
inline ::protobuf_learning::Student_PhoneNumber* Student::add_phone() {
  ::protobuf_learning::Student_PhoneNumber* _add = _internal_add_phone();
  // @@protoc_insertion_point(field_add:protobuf_learning.Student.phone)
  return _add;
}
inline const ::PROTOBUF_NAMESPACE_ID::RepeatedPtrField< ::protobuf_learning::Student_PhoneNumber >&
Student::phone() const {
  // @@protoc_insertion_point(field_list:protobuf_learning.Student.phone)
  return phone_;
}

#ifdef __GNUC__
  #pragma GCC diagnostic pop
#endif  // __GNUC__
// -------------------------------------------------------------------


// @@protoc_insertion_point(namespace_scope)

}  // namespace protobuf_learning

PROTOBUF_NAMESPACE_OPEN

template <> struct is_proto_enum< ::protobuf_learning::Student_PhoneType> : ::std::true_type {};
template <>
inline const EnumDescriptor* GetEnumDescriptor< ::protobuf_learning::Student_PhoneType>() {
  return ::protobuf_learning::Student_PhoneType_descriptor();
}

PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)

#include <google/protobuf/port_undef.inc>
#endif  // GOOGLE_PROTOBUF_INCLUDED_GOOGLE_PROTOBUF_INCLUDED_proto_2fstudent_2eproto
