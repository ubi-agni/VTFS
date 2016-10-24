// Generated by the protocol buffer compiler.  DO NOT EDIT!
// source: mydata.proto

#ifndef PROTOBUF_mydata_2eproto__INCLUDED
#define PROTOBUF_mydata_2eproto__INCLUDED

#include <string>

#include <google/protobuf/stubs/common.h>

#if GOOGLE_PROTOBUF_VERSION < 2006000
#error This file was generated by a newer version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please update
#error your headers.
#endif
#if 2006001 < GOOGLE_PROTOBUF_MIN_PROTOC_VERSION
#error This file was generated by an older version of protoc which is
#error incompatible with your Protocol Buffer headers.  Please
#error regenerate this file with a newer version of protoc.
#endif

#include <google/protobuf/generated_message_util.h>
#include <google/protobuf/message.h>
#include <google/protobuf/repeated_field.h>
#include <google/protobuf/extension_set.h>
#include <google/protobuf/unknown_field_set.h>
// @@protoc_insertion_point(includes)

namespace manip {

// Internal implementation detail -- do not call these.
void  protobuf_AddDesc_mydata_2eproto();
void protobuf_AssignDesc_mydata_2eproto();
void protobuf_ShutdownFile_mydata_2eproto();

class PCsMsg;
class TacMsg;
class VisMsg;
class RobotMsg;
class MarkerPointsMsg;

// ===================================================================

class PCsMsg : public ::google::protobuf::Message {
 public:
  PCsMsg();
  virtual ~PCsMsg();

  PCsMsg(const PCsMsg& from);

  inline PCsMsg& operator=(const PCsMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const PCsMsg& default_instance();

  void Swap(PCsMsg* other);

  // implements Message ----------------------------------------------

  PCsMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const PCsMsg& from);
  void MergeFrom(const PCsMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float CPPosition3D = 1 [packed = true];
  inline int cpposition3d_size() const;
  inline void clear_cpposition3d();
  static const int kCPPosition3DFieldNumber = 1;
  inline float cpposition3d(int index) const;
  inline void set_cpposition3d(int index, float value);
  inline void add_cpposition3d(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      cpposition3d() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_cpposition3d();

  // repeated float CPPositionObject3D = 2 [packed = true];
  inline int cppositionobject3d_size() const;
  inline void clear_cppositionobject3d();
  static const int kCPPositionObject3DFieldNumber = 2;
  inline float cppositionobject3d(int index) const;
  inline void set_cppositionobject3d(int index, float value);
  inline void add_cppositionobject3d(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      cppositionobject3d() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_cppositionobject3d();

  // repeated float CPNormalVector = 3 [packed = true];
  inline int cpnormalvector_size() const;
  inline void clear_cpnormalvector();
  static const int kCPNormalVectorFieldNumber = 3;
  inline float cpnormalvector(int index) const;
  inline void set_cpnormalvector(int index, float value);
  inline void add_cpnormalvector(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      cpnormalvector() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_cpnormalvector();

  // repeated float CPNormalObjectVector = 4 [packed = true];
  inline int cpnormalobjectvector_size() const;
  inline void clear_cpnormalobjectvector();
  static const int kCPNormalObjectVectorFieldNumber = 4;
  inline float cpnormalobjectvector(int index) const;
  inline void set_cpnormalobjectvector(int index, float value);
  inline void add_cpnormalobjectvector(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      cpnormalobjectvector() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_cpnormalobjectvector();

  // @@protoc_insertion_point(class_scope:manip.PCsMsg)
 private:

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > cpposition3d_;
  mutable int _cpposition3d_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > cppositionobject3d_;
  mutable int _cppositionobject3d_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > cpnormalvector_;
  mutable int _cpnormalvector_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > cpnormalobjectvector_;
  mutable int _cpnormalobjectvector_cached_byte_size_;
  friend void  protobuf_AddDesc_mydata_2eproto();
  friend void protobuf_AssignDesc_mydata_2eproto();
  friend void protobuf_ShutdownFile_mydata_2eproto();

  void InitAsDefaultInstance();
  static PCsMsg* default_instance_;
};
// -------------------------------------------------------------------

class TacMsg : public ::google::protobuf::Message {
 public:
  TacMsg();
  virtual ~TacMsg();

  TacMsg(const TacMsg& from);

  inline TacMsg& operator=(const TacMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const TacMsg& default_instance();

  void Swap(TacMsg* other);

  // implements Message ----------------------------------------------

  TacMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const TacMsg& from);
  void MergeFrom(const TacMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float CPPosition2D = 1 [packed = true];
  inline int cpposition2d_size() const;
  inline void clear_cpposition2d();
  static const int kCPPosition2DFieldNumber = 1;
  inline float cpposition2d(int index) const;
  inline void set_cpposition2d(int index, float value);
  inline void add_cpposition2d(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      cpposition2d() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_cpposition2d();

  // required bool contactflag = 2;
  inline bool has_contactflag() const;
  inline void clear_contactflag();
  static const int kContactflagFieldNumber = 2;
  inline bool contactflag() const;
  inline void set_contactflag(bool value);

  // required int32 contactnum = 3;
  inline bool has_contactnum() const;
  inline void clear_contactnum();
  static const int kContactnumFieldNumber = 3;
  inline ::google::protobuf::int32 contactnum() const;
  inline void set_contactnum(::google::protobuf::int32 value);

  // required float contactforce = 4;
  inline bool has_contactforce() const;
  inline void clear_contactforce();
  static const int kContactforceFieldNumber = 4;
  inline float contactforce() const;
  inline void set_contactforce(float value);

  // required float contactorien = 5;
  inline bool has_contactorien() const;
  inline void clear_contactorien();
  static const int kContactorienFieldNumber = 5;
  inline float contactorien() const;
  inline void set_contactorien(float value);

  // @@protoc_insertion_point(class_scope:manip.TacMsg)
 private:
  inline void set_has_contactflag();
  inline void clear_has_contactflag();
  inline void set_has_contactnum();
  inline void clear_has_contactnum();
  inline void set_has_contactforce();
  inline void clear_has_contactforce();
  inline void set_has_contactorien();
  inline void clear_has_contactorien();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > cpposition2d_;
  mutable int _cpposition2d_cached_byte_size_;
  bool contactflag_;
  ::google::protobuf::int32 contactnum_;
  float contactforce_;
  float contactorien_;
  friend void  protobuf_AddDesc_mydata_2eproto();
  friend void protobuf_AssignDesc_mydata_2eproto();
  friend void protobuf_ShutdownFile_mydata_2eproto();

  void InitAsDefaultInstance();
  static TacMsg* default_instance_;
};
// -------------------------------------------------------------------

class VisMsg : public ::google::protobuf::Message {
 public:
  VisMsg();
  virtual ~VisMsg();

  VisMsg(const VisMsg& from);

  inline VisMsg& operator=(const VisMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const VisMsg& default_instance();

  void Swap(VisMsg* other);

  // implements Message ----------------------------------------------

  VisMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const VisMsg& from);
  void MergeFrom(const VisMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float ObjPosition = 1 [packed = true];
  inline int objposition_size() const;
  inline void clear_objposition();
  static const int kObjPositionFieldNumber = 1;
  inline float objposition(int index) const;
  inline void set_objposition(int index, float value);
  inline void add_objposition(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      objposition() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_objposition();

  // repeated float ObjOrien = 2 [packed = true];
  inline int objorien_size() const;
  inline void clear_objorien();
  static const int kObjOrienFieldNumber = 2;
  inline float objorien(int index) const;
  inline void set_objorien(int index, float value);
  inline void add_objorien(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      objorien() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_objorien();

  // required int32 Objnum = 3;
  inline bool has_objnum() const;
  inline void clear_objnum();
  static const int kObjnumFieldNumber = 3;
  inline ::google::protobuf::int32 objnum() const;
  inline void set_objnum(::google::protobuf::int32 value);

  // @@protoc_insertion_point(class_scope:manip.VisMsg)
 private:
  inline void set_has_objnum();
  inline void clear_has_objnum();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > objposition_;
  mutable int _objposition_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > objorien_;
  mutable int _objorien_cached_byte_size_;
  ::google::protobuf::int32 objnum_;
  friend void  protobuf_AddDesc_mydata_2eproto();
  friend void protobuf_AssignDesc_mydata_2eproto();
  friend void protobuf_ShutdownFile_mydata_2eproto();

  void InitAsDefaultInstance();
  static VisMsg* default_instance_;
};
// -------------------------------------------------------------------

class RobotMsg : public ::google::protobuf::Message {
 public:
  RobotMsg();
  virtual ~RobotMsg();

  RobotMsg(const RobotMsg& from);

  inline RobotMsg& operator=(const RobotMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const RobotMsg& default_instance();

  void Swap(RobotMsg* other);

  // implements Message ----------------------------------------------

  RobotMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const RobotMsg& from);
  void MergeFrom(const RobotMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // repeated float orien = 1 [packed = true];
  inline int orien_size() const;
  inline void clear_orien();
  static const int kOrienFieldNumber = 1;
  inline float orien(int index) const;
  inline void set_orien(int index, float value);
  inline void add_orien(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      orien() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_orien();

  // repeated float position = 2 [packed = true];
  inline int position_size() const;
  inline void clear_position();
  static const int kPositionFieldNumber = 2;
  inline float position(int index) const;
  inline void set_position(int index, float value);
  inline void add_position(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      position() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_position();

  // repeated float ft = 3 [packed = true];
  inline int ft_size() const;
  inline void clear_ft();
  static const int kFtFieldNumber = 3;
  inline float ft(int index) const;
  inline void set_ft(int index, float value);
  inline void add_ft(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      ft() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_ft();

  // @@protoc_insertion_point(class_scope:manip.RobotMsg)
 private:

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > orien_;
  mutable int _orien_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > position_;
  mutable int _position_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > ft_;
  mutable int _ft_cached_byte_size_;
  friend void  protobuf_AddDesc_mydata_2eproto();
  friend void protobuf_AssignDesc_mydata_2eproto();
  friend void protobuf_ShutdownFile_mydata_2eproto();

  void InitAsDefaultInstance();
  static RobotMsg* default_instance_;
};
// -------------------------------------------------------------------

class MarkerPointsMsg : public ::google::protobuf::Message {
 public:
  MarkerPointsMsg();
  virtual ~MarkerPointsMsg();

  MarkerPointsMsg(const MarkerPointsMsg& from);

  inline MarkerPointsMsg& operator=(const MarkerPointsMsg& from) {
    CopyFrom(from);
    return *this;
  }

  inline const ::google::protobuf::UnknownFieldSet& unknown_fields() const {
    return _unknown_fields_;
  }

  inline ::google::protobuf::UnknownFieldSet* mutable_unknown_fields() {
    return &_unknown_fields_;
  }

  static const ::google::protobuf::Descriptor* descriptor();
  static const MarkerPointsMsg& default_instance();

  void Swap(MarkerPointsMsg* other);

  // implements Message ----------------------------------------------

  MarkerPointsMsg* New() const;
  void CopyFrom(const ::google::protobuf::Message& from);
  void MergeFrom(const ::google::protobuf::Message& from);
  void CopyFrom(const MarkerPointsMsg& from);
  void MergeFrom(const MarkerPointsMsg& from);
  void Clear();
  bool IsInitialized() const;

  int ByteSize() const;
  bool MergePartialFromCodedStream(
      ::google::protobuf::io::CodedInputStream* input);
  void SerializeWithCachedSizes(
      ::google::protobuf::io::CodedOutputStream* output) const;
  ::google::protobuf::uint8* SerializeWithCachedSizesToArray(::google::protobuf::uint8* output) const;
  int GetCachedSize() const { return _cached_size_; }
  private:
  void SharedCtor();
  void SharedDtor();
  void SetCachedSize(int size) const;
  public:
  ::google::protobuf::Metadata GetMetadata() const;

  // nested types ----------------------------------------------------

  // accessors -------------------------------------------------------

  // required int32 markernum = 1;
  inline bool has_markernum() const;
  inline void clear_markernum();
  static const int kMarkernumFieldNumber = 1;
  inline ::google::protobuf::int32 markernum() const;
  inline void set_markernum(::google::protobuf::int32 value);

  // repeated float position3D = 2 [packed = true];
  inline int position3d_size() const;
  inline void clear_position3d();
  static const int kPosition3DFieldNumber = 2;
  inline float position3d(int index) const;
  inline void set_position3d(int index, float value);
  inline void add_position3d(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      position3d() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_position3d();

  // repeated float normalvector = 3 [packed = true];
  inline int normalvector_size() const;
  inline void clear_normalvector();
  static const int kNormalvectorFieldNumber = 3;
  inline float normalvector(int index) const;
  inline void set_normalvector(int index, float value);
  inline void add_normalvector(float value);
  inline const ::google::protobuf::RepeatedField< float >&
      normalvector() const;
  inline ::google::protobuf::RepeatedField< float >*
      mutable_normalvector();

  // @@protoc_insertion_point(class_scope:manip.MarkerPointsMsg)
 private:
  inline void set_has_markernum();
  inline void clear_has_markernum();

  ::google::protobuf::UnknownFieldSet _unknown_fields_;

  ::google::protobuf::uint32 _has_bits_[1];
  mutable int _cached_size_;
  ::google::protobuf::RepeatedField< float > position3d_;
  mutable int _position3d_cached_byte_size_;
  ::google::protobuf::RepeatedField< float > normalvector_;
  mutable int _normalvector_cached_byte_size_;
  ::google::protobuf::int32 markernum_;
  friend void  protobuf_AddDesc_mydata_2eproto();
  friend void protobuf_AssignDesc_mydata_2eproto();
  friend void protobuf_ShutdownFile_mydata_2eproto();

  void InitAsDefaultInstance();
  static MarkerPointsMsg* default_instance_;
};
// ===================================================================


// ===================================================================

// PCsMsg

// repeated float CPPosition3D = 1 [packed = true];
inline int PCsMsg::cpposition3d_size() const {
  return cpposition3d_.size();
}
inline void PCsMsg::clear_cpposition3d() {
  cpposition3d_.Clear();
}
inline float PCsMsg::cpposition3d(int index) const {
  // @@protoc_insertion_point(field_get:manip.PCsMsg.CPPosition3D)
  return cpposition3d_.Get(index);
}
inline void PCsMsg::set_cpposition3d(int index, float value) {
  cpposition3d_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.PCsMsg.CPPosition3D)
}
inline void PCsMsg::add_cpposition3d(float value) {
  cpposition3d_.Add(value);
  // @@protoc_insertion_point(field_add:manip.PCsMsg.CPPosition3D)
}
inline const ::google::protobuf::RepeatedField< float >&
PCsMsg::cpposition3d() const {
  // @@protoc_insertion_point(field_list:manip.PCsMsg.CPPosition3D)
  return cpposition3d_;
}
inline ::google::protobuf::RepeatedField< float >*
PCsMsg::mutable_cpposition3d() {
  // @@protoc_insertion_point(field_mutable_list:manip.PCsMsg.CPPosition3D)
  return &cpposition3d_;
}

// repeated float CPPositionObject3D = 2 [packed = true];
inline int PCsMsg::cppositionobject3d_size() const {
  return cppositionobject3d_.size();
}
inline void PCsMsg::clear_cppositionobject3d() {
  cppositionobject3d_.Clear();
}
inline float PCsMsg::cppositionobject3d(int index) const {
  // @@protoc_insertion_point(field_get:manip.PCsMsg.CPPositionObject3D)
  return cppositionobject3d_.Get(index);
}
inline void PCsMsg::set_cppositionobject3d(int index, float value) {
  cppositionobject3d_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.PCsMsg.CPPositionObject3D)
}
inline void PCsMsg::add_cppositionobject3d(float value) {
  cppositionobject3d_.Add(value);
  // @@protoc_insertion_point(field_add:manip.PCsMsg.CPPositionObject3D)
}
inline const ::google::protobuf::RepeatedField< float >&
PCsMsg::cppositionobject3d() const {
  // @@protoc_insertion_point(field_list:manip.PCsMsg.CPPositionObject3D)
  return cppositionobject3d_;
}
inline ::google::protobuf::RepeatedField< float >*
PCsMsg::mutable_cppositionobject3d() {
  // @@protoc_insertion_point(field_mutable_list:manip.PCsMsg.CPPositionObject3D)
  return &cppositionobject3d_;
}

// repeated float CPNormalVector = 3 [packed = true];
inline int PCsMsg::cpnormalvector_size() const {
  return cpnormalvector_.size();
}
inline void PCsMsg::clear_cpnormalvector() {
  cpnormalvector_.Clear();
}
inline float PCsMsg::cpnormalvector(int index) const {
  // @@protoc_insertion_point(field_get:manip.PCsMsg.CPNormalVector)
  return cpnormalvector_.Get(index);
}
inline void PCsMsg::set_cpnormalvector(int index, float value) {
  cpnormalvector_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.PCsMsg.CPNormalVector)
}
inline void PCsMsg::add_cpnormalvector(float value) {
  cpnormalvector_.Add(value);
  // @@protoc_insertion_point(field_add:manip.PCsMsg.CPNormalVector)
}
inline const ::google::protobuf::RepeatedField< float >&
PCsMsg::cpnormalvector() const {
  // @@protoc_insertion_point(field_list:manip.PCsMsg.CPNormalVector)
  return cpnormalvector_;
}
inline ::google::protobuf::RepeatedField< float >*
PCsMsg::mutable_cpnormalvector() {
  // @@protoc_insertion_point(field_mutable_list:manip.PCsMsg.CPNormalVector)
  return &cpnormalvector_;
}

// repeated float CPNormalObjectVector = 4 [packed = true];
inline int PCsMsg::cpnormalobjectvector_size() const {
  return cpnormalobjectvector_.size();
}
inline void PCsMsg::clear_cpnormalobjectvector() {
  cpnormalobjectvector_.Clear();
}
inline float PCsMsg::cpnormalobjectvector(int index) const {
  // @@protoc_insertion_point(field_get:manip.PCsMsg.CPNormalObjectVector)
  return cpnormalobjectvector_.Get(index);
}
inline void PCsMsg::set_cpnormalobjectvector(int index, float value) {
  cpnormalobjectvector_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.PCsMsg.CPNormalObjectVector)
}
inline void PCsMsg::add_cpnormalobjectvector(float value) {
  cpnormalobjectvector_.Add(value);
  // @@protoc_insertion_point(field_add:manip.PCsMsg.CPNormalObjectVector)
}
inline const ::google::protobuf::RepeatedField< float >&
PCsMsg::cpnormalobjectvector() const {
  // @@protoc_insertion_point(field_list:manip.PCsMsg.CPNormalObjectVector)
  return cpnormalobjectvector_;
}
inline ::google::protobuf::RepeatedField< float >*
PCsMsg::mutable_cpnormalobjectvector() {
  // @@protoc_insertion_point(field_mutable_list:manip.PCsMsg.CPNormalObjectVector)
  return &cpnormalobjectvector_;
}

// -------------------------------------------------------------------

// TacMsg

// repeated float CPPosition2D = 1 [packed = true];
inline int TacMsg::cpposition2d_size() const {
  return cpposition2d_.size();
}
inline void TacMsg::clear_cpposition2d() {
  cpposition2d_.Clear();
}
inline float TacMsg::cpposition2d(int index) const {
  // @@protoc_insertion_point(field_get:manip.TacMsg.CPPosition2D)
  return cpposition2d_.Get(index);
}
inline void TacMsg::set_cpposition2d(int index, float value) {
  cpposition2d_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.TacMsg.CPPosition2D)
}
inline void TacMsg::add_cpposition2d(float value) {
  cpposition2d_.Add(value);
  // @@protoc_insertion_point(field_add:manip.TacMsg.CPPosition2D)
}
inline const ::google::protobuf::RepeatedField< float >&
TacMsg::cpposition2d() const {
  // @@protoc_insertion_point(field_list:manip.TacMsg.CPPosition2D)
  return cpposition2d_;
}
inline ::google::protobuf::RepeatedField< float >*
TacMsg::mutable_cpposition2d() {
  // @@protoc_insertion_point(field_mutable_list:manip.TacMsg.CPPosition2D)
  return &cpposition2d_;
}

// required bool contactflag = 2;
inline bool TacMsg::has_contactflag() const {
  return (_has_bits_[0] & 0x00000002u) != 0;
}
inline void TacMsg::set_has_contactflag() {
  _has_bits_[0] |= 0x00000002u;
}
inline void TacMsg::clear_has_contactflag() {
  _has_bits_[0] &= ~0x00000002u;
}
inline void TacMsg::clear_contactflag() {
  contactflag_ = false;
  clear_has_contactflag();
}
inline bool TacMsg::contactflag() const {
  // @@protoc_insertion_point(field_get:manip.TacMsg.contactflag)
  return contactflag_;
}
inline void TacMsg::set_contactflag(bool value) {
  set_has_contactflag();
  contactflag_ = value;
  // @@protoc_insertion_point(field_set:manip.TacMsg.contactflag)
}

// required int32 contactnum = 3;
inline bool TacMsg::has_contactnum() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void TacMsg::set_has_contactnum() {
  _has_bits_[0] |= 0x00000004u;
}
inline void TacMsg::clear_has_contactnum() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void TacMsg::clear_contactnum() {
  contactnum_ = 0;
  clear_has_contactnum();
}
inline ::google::protobuf::int32 TacMsg::contactnum() const {
  // @@protoc_insertion_point(field_get:manip.TacMsg.contactnum)
  return contactnum_;
}
inline void TacMsg::set_contactnum(::google::protobuf::int32 value) {
  set_has_contactnum();
  contactnum_ = value;
  // @@protoc_insertion_point(field_set:manip.TacMsg.contactnum)
}

// required float contactforce = 4;
inline bool TacMsg::has_contactforce() const {
  return (_has_bits_[0] & 0x00000008u) != 0;
}
inline void TacMsg::set_has_contactforce() {
  _has_bits_[0] |= 0x00000008u;
}
inline void TacMsg::clear_has_contactforce() {
  _has_bits_[0] &= ~0x00000008u;
}
inline void TacMsg::clear_contactforce() {
  contactforce_ = 0;
  clear_has_contactforce();
}
inline float TacMsg::contactforce() const {
  // @@protoc_insertion_point(field_get:manip.TacMsg.contactforce)
  return contactforce_;
}
inline void TacMsg::set_contactforce(float value) {
  set_has_contactforce();
  contactforce_ = value;
  // @@protoc_insertion_point(field_set:manip.TacMsg.contactforce)
}

// required float contactorien = 5;
inline bool TacMsg::has_contactorien() const {
  return (_has_bits_[0] & 0x00000010u) != 0;
}
inline void TacMsg::set_has_contactorien() {
  _has_bits_[0] |= 0x00000010u;
}
inline void TacMsg::clear_has_contactorien() {
  _has_bits_[0] &= ~0x00000010u;
}
inline void TacMsg::clear_contactorien() {
  contactorien_ = 0;
  clear_has_contactorien();
}
inline float TacMsg::contactorien() const {
  // @@protoc_insertion_point(field_get:manip.TacMsg.contactorien)
  return contactorien_;
}
inline void TacMsg::set_contactorien(float value) {
  set_has_contactorien();
  contactorien_ = value;
  // @@protoc_insertion_point(field_set:manip.TacMsg.contactorien)
}

// -------------------------------------------------------------------

// VisMsg

// repeated float ObjPosition = 1 [packed = true];
inline int VisMsg::objposition_size() const {
  return objposition_.size();
}
inline void VisMsg::clear_objposition() {
  objposition_.Clear();
}
inline float VisMsg::objposition(int index) const {
  // @@protoc_insertion_point(field_get:manip.VisMsg.ObjPosition)
  return objposition_.Get(index);
}
inline void VisMsg::set_objposition(int index, float value) {
  objposition_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.VisMsg.ObjPosition)
}
inline void VisMsg::add_objposition(float value) {
  objposition_.Add(value);
  // @@protoc_insertion_point(field_add:manip.VisMsg.ObjPosition)
}
inline const ::google::protobuf::RepeatedField< float >&
VisMsg::objposition() const {
  // @@protoc_insertion_point(field_list:manip.VisMsg.ObjPosition)
  return objposition_;
}
inline ::google::protobuf::RepeatedField< float >*
VisMsg::mutable_objposition() {
  // @@protoc_insertion_point(field_mutable_list:manip.VisMsg.ObjPosition)
  return &objposition_;
}

// repeated float ObjOrien = 2 [packed = true];
inline int VisMsg::objorien_size() const {
  return objorien_.size();
}
inline void VisMsg::clear_objorien() {
  objorien_.Clear();
}
inline float VisMsg::objorien(int index) const {
  // @@protoc_insertion_point(field_get:manip.VisMsg.ObjOrien)
  return objorien_.Get(index);
}
inline void VisMsg::set_objorien(int index, float value) {
  objorien_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.VisMsg.ObjOrien)
}
inline void VisMsg::add_objorien(float value) {
  objorien_.Add(value);
  // @@protoc_insertion_point(field_add:manip.VisMsg.ObjOrien)
}
inline const ::google::protobuf::RepeatedField< float >&
VisMsg::objorien() const {
  // @@protoc_insertion_point(field_list:manip.VisMsg.ObjOrien)
  return objorien_;
}
inline ::google::protobuf::RepeatedField< float >*
VisMsg::mutable_objorien() {
  // @@protoc_insertion_point(field_mutable_list:manip.VisMsg.ObjOrien)
  return &objorien_;
}

// required int32 Objnum = 3;
inline bool VisMsg::has_objnum() const {
  return (_has_bits_[0] & 0x00000004u) != 0;
}
inline void VisMsg::set_has_objnum() {
  _has_bits_[0] |= 0x00000004u;
}
inline void VisMsg::clear_has_objnum() {
  _has_bits_[0] &= ~0x00000004u;
}
inline void VisMsg::clear_objnum() {
  objnum_ = 0;
  clear_has_objnum();
}
inline ::google::protobuf::int32 VisMsg::objnum() const {
  // @@protoc_insertion_point(field_get:manip.VisMsg.Objnum)
  return objnum_;
}
inline void VisMsg::set_objnum(::google::protobuf::int32 value) {
  set_has_objnum();
  objnum_ = value;
  // @@protoc_insertion_point(field_set:manip.VisMsg.Objnum)
}

// -------------------------------------------------------------------

// RobotMsg

// repeated float orien = 1 [packed = true];
inline int RobotMsg::orien_size() const {
  return orien_.size();
}
inline void RobotMsg::clear_orien() {
  orien_.Clear();
}
inline float RobotMsg::orien(int index) const {
  // @@protoc_insertion_point(field_get:manip.RobotMsg.orien)
  return orien_.Get(index);
}
inline void RobotMsg::set_orien(int index, float value) {
  orien_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.RobotMsg.orien)
}
inline void RobotMsg::add_orien(float value) {
  orien_.Add(value);
  // @@protoc_insertion_point(field_add:manip.RobotMsg.orien)
}
inline const ::google::protobuf::RepeatedField< float >&
RobotMsg::orien() const {
  // @@protoc_insertion_point(field_list:manip.RobotMsg.orien)
  return orien_;
}
inline ::google::protobuf::RepeatedField< float >*
RobotMsg::mutable_orien() {
  // @@protoc_insertion_point(field_mutable_list:manip.RobotMsg.orien)
  return &orien_;
}

// repeated float position = 2 [packed = true];
inline int RobotMsg::position_size() const {
  return position_.size();
}
inline void RobotMsg::clear_position() {
  position_.Clear();
}
inline float RobotMsg::position(int index) const {
  // @@protoc_insertion_point(field_get:manip.RobotMsg.position)
  return position_.Get(index);
}
inline void RobotMsg::set_position(int index, float value) {
  position_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.RobotMsg.position)
}
inline void RobotMsg::add_position(float value) {
  position_.Add(value);
  // @@protoc_insertion_point(field_add:manip.RobotMsg.position)
}
inline const ::google::protobuf::RepeatedField< float >&
RobotMsg::position() const {
  // @@protoc_insertion_point(field_list:manip.RobotMsg.position)
  return position_;
}
inline ::google::protobuf::RepeatedField< float >*
RobotMsg::mutable_position() {
  // @@protoc_insertion_point(field_mutable_list:manip.RobotMsg.position)
  return &position_;
}

// repeated float ft = 3 [packed = true];
inline int RobotMsg::ft_size() const {
  return ft_.size();
}
inline void RobotMsg::clear_ft() {
  ft_.Clear();
}
inline float RobotMsg::ft(int index) const {
  // @@protoc_insertion_point(field_get:manip.RobotMsg.ft)
  return ft_.Get(index);
}
inline void RobotMsg::set_ft(int index, float value) {
  ft_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.RobotMsg.ft)
}
inline void RobotMsg::add_ft(float value) {
  ft_.Add(value);
  // @@protoc_insertion_point(field_add:manip.RobotMsg.ft)
}
inline const ::google::protobuf::RepeatedField< float >&
RobotMsg::ft() const {
  // @@protoc_insertion_point(field_list:manip.RobotMsg.ft)
  return ft_;
}
inline ::google::protobuf::RepeatedField< float >*
RobotMsg::mutable_ft() {
  // @@protoc_insertion_point(field_mutable_list:manip.RobotMsg.ft)
  return &ft_;
}

// -------------------------------------------------------------------

// MarkerPointsMsg

// required int32 markernum = 1;
inline bool MarkerPointsMsg::has_markernum() const {
  return (_has_bits_[0] & 0x00000001u) != 0;
}
inline void MarkerPointsMsg::set_has_markernum() {
  _has_bits_[0] |= 0x00000001u;
}
inline void MarkerPointsMsg::clear_has_markernum() {
  _has_bits_[0] &= ~0x00000001u;
}
inline void MarkerPointsMsg::clear_markernum() {
  markernum_ = 0;
  clear_has_markernum();
}
inline ::google::protobuf::int32 MarkerPointsMsg::markernum() const {
  // @@protoc_insertion_point(field_get:manip.MarkerPointsMsg.markernum)
  return markernum_;
}
inline void MarkerPointsMsg::set_markernum(::google::protobuf::int32 value) {
  set_has_markernum();
  markernum_ = value;
  // @@protoc_insertion_point(field_set:manip.MarkerPointsMsg.markernum)
}

// repeated float position3D = 2 [packed = true];
inline int MarkerPointsMsg::position3d_size() const {
  return position3d_.size();
}
inline void MarkerPointsMsg::clear_position3d() {
  position3d_.Clear();
}
inline float MarkerPointsMsg::position3d(int index) const {
  // @@protoc_insertion_point(field_get:manip.MarkerPointsMsg.position3D)
  return position3d_.Get(index);
}
inline void MarkerPointsMsg::set_position3d(int index, float value) {
  position3d_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.MarkerPointsMsg.position3D)
}
inline void MarkerPointsMsg::add_position3d(float value) {
  position3d_.Add(value);
  // @@protoc_insertion_point(field_add:manip.MarkerPointsMsg.position3D)
}
inline const ::google::protobuf::RepeatedField< float >&
MarkerPointsMsg::position3d() const {
  // @@protoc_insertion_point(field_list:manip.MarkerPointsMsg.position3D)
  return position3d_;
}
inline ::google::protobuf::RepeatedField< float >*
MarkerPointsMsg::mutable_position3d() {
  // @@protoc_insertion_point(field_mutable_list:manip.MarkerPointsMsg.position3D)
  return &position3d_;
}

// repeated float normalvector = 3 [packed = true];
inline int MarkerPointsMsg::normalvector_size() const {
  return normalvector_.size();
}
inline void MarkerPointsMsg::clear_normalvector() {
  normalvector_.Clear();
}
inline float MarkerPointsMsg::normalvector(int index) const {
  // @@protoc_insertion_point(field_get:manip.MarkerPointsMsg.normalvector)
  return normalvector_.Get(index);
}
inline void MarkerPointsMsg::set_normalvector(int index, float value) {
  normalvector_.Set(index, value);
  // @@protoc_insertion_point(field_set:manip.MarkerPointsMsg.normalvector)
}
inline void MarkerPointsMsg::add_normalvector(float value) {
  normalvector_.Add(value);
  // @@protoc_insertion_point(field_add:manip.MarkerPointsMsg.normalvector)
}
inline const ::google::protobuf::RepeatedField< float >&
MarkerPointsMsg::normalvector() const {
  // @@protoc_insertion_point(field_list:manip.MarkerPointsMsg.normalvector)
  return normalvector_;
}
inline ::google::protobuf::RepeatedField< float >*
MarkerPointsMsg::mutable_normalvector() {
  // @@protoc_insertion_point(field_mutable_list:manip.MarkerPointsMsg.normalvector)
  return &normalvector_;
}


// @@protoc_insertion_point(namespace_scope)

}  // namespace manip

#ifndef SWIG
namespace google {
namespace protobuf {


}  // namespace google
}  // namespace protobuf
#endif  // SWIG

// @@protoc_insertion_point(global_scope)

#endif  // PROTOBUF_mydata_2eproto__INCLUDED
