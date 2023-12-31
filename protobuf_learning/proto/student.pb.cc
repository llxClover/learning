// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: proto/student.proto

#include "proto/student.pb.h"

#include <algorithm>

#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/wire_format_lite.h>
#include <google/protobuf/descriptor.h>
#include <google/protobuf/generated_message_reflection.h>
#include <google/protobuf/reflection_ops.h>
#include <google/protobuf/wire_format.h>
// @@protoc_insertion_point(includes)
#include <google/protobuf/port_def.inc>

PROTOBUF_PRAGMA_INIT_SEG
namespace protobuf_learning {
constexpr Student_PhoneNumber::Student_PhoneNumber(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : number_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , type_(0)
{}
struct Student_PhoneNumberDefaultTypeInternal {
  constexpr Student_PhoneNumberDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~Student_PhoneNumberDefaultTypeInternal() {}
  union {
    Student_PhoneNumber _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT Student_PhoneNumberDefaultTypeInternal _Student_PhoneNumber_default_instance_;
constexpr Student::Student(
  ::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized)
  : phone_()
  , name_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , email_(&::PROTOBUF_NAMESPACE_ID::internal::fixed_address_empty_string)
  , id_(uint64_t{0u}){}
struct StudentDefaultTypeInternal {
  constexpr StudentDefaultTypeInternal()
    : _instance(::PROTOBUF_NAMESPACE_ID::internal::ConstantInitialized{}) {}
  ~StudentDefaultTypeInternal() {}
  union {
    Student _instance;
  };
};
PROTOBUF_ATTRIBUTE_NO_DESTROY PROTOBUF_CONSTINIT StudentDefaultTypeInternal _Student_default_instance_;
}  // namespace protobuf_learning
static ::PROTOBUF_NAMESPACE_ID::Metadata file_level_metadata_proto_2fstudent_2eproto[2];
static const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* file_level_enum_descriptors_proto_2fstudent_2eproto[1];
static constexpr ::PROTOBUF_NAMESPACE_ID::ServiceDescriptor const** file_level_service_descriptors_proto_2fstudent_2eproto = nullptr;

const uint32_t TableStruct_proto_2fstudent_2eproto::offsets[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student_PhoneNumber, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student_PhoneNumber, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student_PhoneNumber, number_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student_PhoneNumber, type_),
  ~0u,
  0,
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, _has_bits_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, _internal_metadata_),
  ~0u,  // no _extensions_
  ~0u,  // no _oneof_case_
  ~0u,  // no _weak_field_map_
  ~0u,  // no _inlined_string_donated_
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, id_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, name_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, email_),
  PROTOBUF_FIELD_OFFSET(::protobuf_learning::Student, phone_),
  ~0u,
  ~0u,
  0,
  ~0u,
};
static const ::PROTOBUF_NAMESPACE_ID::internal::MigrationSchema schemas[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) = {
  { 0, 8, -1, sizeof(::protobuf_learning::Student_PhoneNumber)},
  { 10, 20, -1, sizeof(::protobuf_learning::Student)},
};

static ::PROTOBUF_NAMESPACE_ID::Message const * const file_default_instances[] = {
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::protobuf_learning::_Student_PhoneNumber_default_instance_),
  reinterpret_cast<const ::PROTOBUF_NAMESPACE_ID::Message*>(&::protobuf_learning::_Student_default_instance_),
};

const char descriptor_table_protodef_proto_2fstudent_2eproto[] PROTOBUF_SECTION_VARIABLE(protodesc_cold) =
  "\n\023proto/student.proto\022\021protobuf_learning"
  "\"\374\001\n\007Student\022\n\n\002id\030\001 \001(\004\022\014\n\004name\030\002 \001(\t\022\022"
  "\n\005email\030\003 \001(\tH\000\210\001\001\0225\n\005phone\030\004 \003(\0132&.prot"
  "obuf_learning.Student.PhoneNumber\032_\n\013Pho"
  "neNumber\022\016\n\006number\030\001 \001(\t\0227\n\004type\030\002 \001(\0162$"
  ".protobuf_learning.Student.PhoneTypeH\000\210\001"
  "\001B\007\n\005_type\"!\n\tPhoneType\022\n\n\006MOBILE\020\000\022\010\n\004H"
  "OME\020\001B\010\n\006_emailb\006proto3"
  ;
static ::PROTOBUF_NAMESPACE_ID::internal::once_flag descriptor_table_proto_2fstudent_2eproto_once;
const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable descriptor_table_proto_2fstudent_2eproto = {
  false, false, 303, descriptor_table_protodef_proto_2fstudent_2eproto, "proto/student.proto", 
  &descriptor_table_proto_2fstudent_2eproto_once, nullptr, 0, 2,
  schemas, file_default_instances, TableStruct_proto_2fstudent_2eproto::offsets,
  file_level_metadata_proto_2fstudent_2eproto, file_level_enum_descriptors_proto_2fstudent_2eproto, file_level_service_descriptors_proto_2fstudent_2eproto,
};
PROTOBUF_ATTRIBUTE_WEAK const ::PROTOBUF_NAMESPACE_ID::internal::DescriptorTable* descriptor_table_proto_2fstudent_2eproto_getter() {
  return &descriptor_table_proto_2fstudent_2eproto;
}

// Force running AddDescriptors() at dynamic initialization time.
PROTOBUF_ATTRIBUTE_INIT_PRIORITY static ::PROTOBUF_NAMESPACE_ID::internal::AddDescriptorsRunner dynamic_init_dummy_proto_2fstudent_2eproto(&descriptor_table_proto_2fstudent_2eproto);
namespace protobuf_learning {
const ::PROTOBUF_NAMESPACE_ID::EnumDescriptor* Student_PhoneType_descriptor() {
  ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(&descriptor_table_proto_2fstudent_2eproto);
  return file_level_enum_descriptors_proto_2fstudent_2eproto[0];
}
bool Student_PhoneType_IsValid(int value) {
  switch (value) {
    case 0:
    case 1:
      return true;
    default:
      return false;
  }
}

#if (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))
constexpr Student_PhoneType Student::MOBILE;
constexpr Student_PhoneType Student::HOME;
constexpr Student_PhoneType Student::PhoneType_MIN;
constexpr Student_PhoneType Student::PhoneType_MAX;
constexpr int Student::PhoneType_ARRAYSIZE;
#endif  // (__cplusplus < 201703) && (!defined(_MSC_VER) || (_MSC_VER >= 1900 && _MSC_VER < 1912))

// ===================================================================

class Student_PhoneNumber::_Internal {
 public:
  using HasBits = decltype(std::declval<Student_PhoneNumber>()._has_bits_);
  static void set_has_type(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

Student_PhoneNumber::Student_PhoneNumber(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:protobuf_learning.Student.PhoneNumber)
}
Student_PhoneNumber::Student_PhoneNumber(const Student_PhoneNumber& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  number_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    number_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_number().empty()) {
    number_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_number(), 
      GetArenaForAllocation());
  }
  type_ = from.type_;
  // @@protoc_insertion_point(copy_constructor:protobuf_learning.Student.PhoneNumber)
}

inline void Student_PhoneNumber::SharedCtor() {
number_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  number_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
type_ = 0;
}

Student_PhoneNumber::~Student_PhoneNumber() {
  // @@protoc_insertion_point(destructor:protobuf_learning.Student.PhoneNumber)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Student_PhoneNumber::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  number_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Student_PhoneNumber::ArenaDtor(void* object) {
  Student_PhoneNumber* _this = reinterpret_cast< Student_PhoneNumber* >(object);
  (void)_this;
}
void Student_PhoneNumber::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Student_PhoneNumber::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Student_PhoneNumber::Clear() {
// @@protoc_insertion_point(message_clear_start:protobuf_learning.Student.PhoneNumber)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  number_.ClearToEmpty();
  type_ = 0;
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Student_PhoneNumber::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // string number = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 10)) {
          auto str = _internal_mutable_number();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "protobuf_learning.Student.PhoneNumber.number"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional .protobuf_learning.Student.PhoneType type = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 16)) {
          uint64_t val = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
          _internal_set_type(static_cast<::protobuf_learning::Student_PhoneType>(val));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Student_PhoneNumber::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:protobuf_learning.Student.PhoneNumber)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // string number = 1;
  if (!this->_internal_number().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_number().data(), static_cast<int>(this->_internal_number().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "protobuf_learning.Student.PhoneNumber.number");
    target = stream->WriteStringMaybeAliased(
        1, this->_internal_number(), target);
  }

  // optional .protobuf_learning.Student.PhoneType type = 2;
  if (_internal_has_type()) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteEnumToArray(
      2, this->_internal_type(), target);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:protobuf_learning.Student.PhoneNumber)
  return target;
}

size_t Student_PhoneNumber::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:protobuf_learning.Student.PhoneNumber)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // string number = 1;
  if (!this->_internal_number().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_number());
  }

  // optional .protobuf_learning.Student.PhoneType type = 2;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::EnumSize(this->_internal_type());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Student_PhoneNumber::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Student_PhoneNumber::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Student_PhoneNumber::GetClassData() const { return &_class_data_; }

void Student_PhoneNumber::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Student_PhoneNumber *>(to)->MergeFrom(
      static_cast<const Student_PhoneNumber &>(from));
}


void Student_PhoneNumber::MergeFrom(const Student_PhoneNumber& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:protobuf_learning.Student.PhoneNumber)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  if (!from._internal_number().empty()) {
    _internal_set_number(from._internal_number());
  }
  if (from._internal_has_type()) {
    _internal_set_type(from._internal_type());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Student_PhoneNumber::CopyFrom(const Student_PhoneNumber& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:protobuf_learning.Student.PhoneNumber)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Student_PhoneNumber::IsInitialized() const {
  return true;
}

void Student_PhoneNumber::InternalSwap(Student_PhoneNumber* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &number_, lhs_arena,
      &other->number_, rhs_arena
  );
  swap(type_, other->type_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Student_PhoneNumber::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_proto_2fstudent_2eproto_getter, &descriptor_table_proto_2fstudent_2eproto_once,
      file_level_metadata_proto_2fstudent_2eproto[0]);
}

// ===================================================================

class Student::_Internal {
 public:
  using HasBits = decltype(std::declval<Student>()._has_bits_);
  static void set_has_email(HasBits* has_bits) {
    (*has_bits)[0] |= 1u;
  }
};

Student::Student(::PROTOBUF_NAMESPACE_ID::Arena* arena,
                         bool is_message_owned)
  : ::PROTOBUF_NAMESPACE_ID::Message(arena, is_message_owned),
  phone_(arena) {
  SharedCtor();
  if (!is_message_owned) {
    RegisterArenaDtor(arena);
  }
  // @@protoc_insertion_point(arena_constructor:protobuf_learning.Student)
}
Student::Student(const Student& from)
  : ::PROTOBUF_NAMESPACE_ID::Message(),
      _has_bits_(from._has_bits_),
      phone_(from.phone_) {
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
  name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (!from._internal_name().empty()) {
    name_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_name(), 
      GetArenaForAllocation());
  }
  email_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  #ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
    email_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
  #endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
  if (from._internal_has_email()) {
    email_.Set(::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::EmptyDefault{}, from._internal_email(), 
      GetArenaForAllocation());
  }
  id_ = from.id_;
  // @@protoc_insertion_point(copy_constructor:protobuf_learning.Student)
}

inline void Student::SharedCtor() {
name_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  name_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
email_.UnsafeSetDefault(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
#ifdef PROTOBUF_FORCE_COPY_DEFAULT_STRING
  email_.Set(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(), "", GetArenaForAllocation());
#endif // PROTOBUF_FORCE_COPY_DEFAULT_STRING
id_ = uint64_t{0u};
}

Student::~Student() {
  // @@protoc_insertion_point(destructor:protobuf_learning.Student)
  if (GetArenaForAllocation() != nullptr) return;
  SharedDtor();
  _internal_metadata_.Delete<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

inline void Student::SharedDtor() {
  GOOGLE_DCHECK(GetArenaForAllocation() == nullptr);
  name_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
  email_.DestroyNoArena(&::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited());
}

void Student::ArenaDtor(void* object) {
  Student* _this = reinterpret_cast< Student* >(object);
  (void)_this;
}
void Student::RegisterArenaDtor(::PROTOBUF_NAMESPACE_ID::Arena*) {
}
void Student::SetCachedSize(int size) const {
  _cached_size_.Set(size);
}

void Student::Clear() {
// @@protoc_insertion_point(message_clear_start:protobuf_learning.Student)
  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  phone_.Clear();
  name_.ClearToEmpty();
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    email_.ClearNonDefaultToEmpty();
  }
  id_ = uint64_t{0u};
  _has_bits_.Clear();
  _internal_metadata_.Clear<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>();
}

const char* Student::_InternalParse(const char* ptr, ::PROTOBUF_NAMESPACE_ID::internal::ParseContext* ctx) {
#define CHK_(x) if (PROTOBUF_PREDICT_FALSE(!(x))) goto failure
  _Internal::HasBits has_bits{};
  while (!ctx->Done(&ptr)) {
    uint32_t tag;
    ptr = ::PROTOBUF_NAMESPACE_ID::internal::ReadTag(ptr, &tag);
    switch (tag >> 3) {
      // uint64 id = 1;
      case 1:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 8)) {
          id_ = ::PROTOBUF_NAMESPACE_ID::internal::ReadVarint64(&ptr);
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // string name = 2;
      case 2:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 18)) {
          auto str = _internal_mutable_name();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "protobuf_learning.Student.name"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // optional string email = 3;
      case 3:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 26)) {
          auto str = _internal_mutable_email();
          ptr = ::PROTOBUF_NAMESPACE_ID::internal::InlineGreedyStringParser(str, ptr, ctx);
          CHK_(::PROTOBUF_NAMESPACE_ID::internal::VerifyUTF8(str, "protobuf_learning.Student.email"));
          CHK_(ptr);
        } else
          goto handle_unusual;
        continue;
      // repeated .protobuf_learning.Student.PhoneNumber phone = 4;
      case 4:
        if (PROTOBUF_PREDICT_TRUE(static_cast<uint8_t>(tag) == 34)) {
          ptr -= 1;
          do {
            ptr += 1;
            ptr = ctx->ParseMessage(_internal_add_phone(), ptr);
            CHK_(ptr);
            if (!ctx->DataAvailable(ptr)) break;
          } while (::PROTOBUF_NAMESPACE_ID::internal::ExpectTag<34>(ptr));
        } else
          goto handle_unusual;
        continue;
      default:
        goto handle_unusual;
    }  // switch
  handle_unusual:
    if ((tag == 0) || ((tag & 7) == 4)) {
      CHK_(ptr);
      ctx->SetLastTag(tag);
      goto message_done;
    }
    ptr = UnknownFieldParse(
        tag,
        _internal_metadata_.mutable_unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(),
        ptr, ctx);
    CHK_(ptr != nullptr);
  }  // while
message_done:
  _has_bits_.Or(has_bits);
  return ptr;
failure:
  ptr = nullptr;
  goto message_done;
#undef CHK_
}

uint8_t* Student::_InternalSerialize(
    uint8_t* target, ::PROTOBUF_NAMESPACE_ID::io::EpsCopyOutputStream* stream) const {
  // @@protoc_insertion_point(serialize_to_array_start:protobuf_learning.Student)
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  // uint64 id = 1;
  if (this->_internal_id() != 0) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::WriteUInt64ToArray(1, this->_internal_id(), target);
  }

  // string name = 2;
  if (!this->_internal_name().empty()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_name().data(), static_cast<int>(this->_internal_name().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "protobuf_learning.Student.name");
    target = stream->WriteStringMaybeAliased(
        2, this->_internal_name(), target);
  }

  // optional string email = 3;
  if (_internal_has_email()) {
    ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::VerifyUtf8String(
      this->_internal_email().data(), static_cast<int>(this->_internal_email().length()),
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::SERIALIZE,
      "protobuf_learning.Student.email");
    target = stream->WriteStringMaybeAliased(
        3, this->_internal_email(), target);
  }

  // repeated .protobuf_learning.Student.PhoneNumber phone = 4;
  for (unsigned int i = 0,
      n = static_cast<unsigned int>(this->_internal_phone_size()); i < n; i++) {
    target = stream->EnsureSpace(target);
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::
      InternalWriteMessage(4, this->_internal_phone(i), target, stream);
  }

  if (PROTOBUF_PREDICT_FALSE(_internal_metadata_.have_unknown_fields())) {
    target = ::PROTOBUF_NAMESPACE_ID::internal::WireFormat::InternalSerializeUnknownFieldsToArray(
        _internal_metadata_.unknown_fields<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(::PROTOBUF_NAMESPACE_ID::UnknownFieldSet::default_instance), target, stream);
  }
  // @@protoc_insertion_point(serialize_to_array_end:protobuf_learning.Student)
  return target;
}

size_t Student::ByteSizeLong() const {
// @@protoc_insertion_point(message_byte_size_start:protobuf_learning.Student)
  size_t total_size = 0;

  uint32_t cached_has_bits = 0;
  // Prevent compiler warnings about cached_has_bits being unused
  (void) cached_has_bits;

  // repeated .protobuf_learning.Student.PhoneNumber phone = 4;
  total_size += 1UL * this->_internal_phone_size();
  for (const auto& msg : this->phone_) {
    total_size +=
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::MessageSize(msg);
  }

  // string name = 2;
  if (!this->_internal_name().empty()) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_name());
  }

  // optional string email = 3;
  cached_has_bits = _has_bits_[0];
  if (cached_has_bits & 0x00000001u) {
    total_size += 1 +
      ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::StringSize(
        this->_internal_email());
  }

  // uint64 id = 1;
  if (this->_internal_id() != 0) {
    total_size += ::PROTOBUF_NAMESPACE_ID::internal::WireFormatLite::UInt64SizePlusOne(this->_internal_id());
  }

  return MaybeComputeUnknownFieldsSize(total_size, &_cached_size_);
}

const ::PROTOBUF_NAMESPACE_ID::Message::ClassData Student::_class_data_ = {
    ::PROTOBUF_NAMESPACE_ID::Message::CopyWithSizeCheck,
    Student::MergeImpl
};
const ::PROTOBUF_NAMESPACE_ID::Message::ClassData*Student::GetClassData() const { return &_class_data_; }

void Student::MergeImpl(::PROTOBUF_NAMESPACE_ID::Message* to,
                      const ::PROTOBUF_NAMESPACE_ID::Message& from) {
  static_cast<Student *>(to)->MergeFrom(
      static_cast<const Student &>(from));
}


void Student::MergeFrom(const Student& from) {
// @@protoc_insertion_point(class_specific_merge_from_start:protobuf_learning.Student)
  GOOGLE_DCHECK_NE(&from, this);
  uint32_t cached_has_bits = 0;
  (void) cached_has_bits;

  phone_.MergeFrom(from.phone_);
  if (!from._internal_name().empty()) {
    _internal_set_name(from._internal_name());
  }
  if (from._internal_has_email()) {
    _internal_set_email(from._internal_email());
  }
  if (from._internal_id() != 0) {
    _internal_set_id(from._internal_id());
  }
  _internal_metadata_.MergeFrom<::PROTOBUF_NAMESPACE_ID::UnknownFieldSet>(from._internal_metadata_);
}

void Student::CopyFrom(const Student& from) {
// @@protoc_insertion_point(class_specific_copy_from_start:protobuf_learning.Student)
  if (&from == this) return;
  Clear();
  MergeFrom(from);
}

bool Student::IsInitialized() const {
  return true;
}

void Student::InternalSwap(Student* other) {
  using std::swap;
  auto* lhs_arena = GetArenaForAllocation();
  auto* rhs_arena = other->GetArenaForAllocation();
  _internal_metadata_.InternalSwap(&other->_internal_metadata_);
  swap(_has_bits_[0], other->_has_bits_[0]);
  phone_.InternalSwap(&other->phone_);
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &name_, lhs_arena,
      &other->name_, rhs_arena
  );
  ::PROTOBUF_NAMESPACE_ID::internal::ArenaStringPtr::InternalSwap(
      &::PROTOBUF_NAMESPACE_ID::internal::GetEmptyStringAlreadyInited(),
      &email_, lhs_arena,
      &other->email_, rhs_arena
  );
  swap(id_, other->id_);
}

::PROTOBUF_NAMESPACE_ID::Metadata Student::GetMetadata() const {
  return ::PROTOBUF_NAMESPACE_ID::internal::AssignDescriptors(
      &descriptor_table_proto_2fstudent_2eproto_getter, &descriptor_table_proto_2fstudent_2eproto_once,
      file_level_metadata_proto_2fstudent_2eproto[1]);
}

// @@protoc_insertion_point(namespace_scope)
}  // namespace protobuf_learning
PROTOBUF_NAMESPACE_OPEN
template<> PROTOBUF_NOINLINE ::protobuf_learning::Student_PhoneNumber* Arena::CreateMaybeMessage< ::protobuf_learning::Student_PhoneNumber >(Arena* arena) {
  return Arena::CreateMessageInternal< ::protobuf_learning::Student_PhoneNumber >(arena);
}
template<> PROTOBUF_NOINLINE ::protobuf_learning::Student* Arena::CreateMaybeMessage< ::protobuf_learning::Student >(Arena* arena) {
  return Arena::CreateMessageInternal< ::protobuf_learning::Student >(arena);
}
PROTOBUF_NAMESPACE_CLOSE

// @@protoc_insertion_point(global_scope)
#include <google/protobuf/port_undef.inc>
