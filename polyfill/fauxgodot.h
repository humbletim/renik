#pragma once
// ---------------------------------------------------------------------------
// emulates godot's C++ APIs enough to compile RenIK

// ---------------------------------------------------------------------------
// Resource

template <typename T> struct Ref {
  T* value = nullptr;
  void instance() { value = new T(); }
  T* operator->() { return value; }
  bool operator!=(void* other) { return value != other; }
  bool is_null() const { return value; }
};
struct Reference {};
struct Resource : public Reference {};

// ---------------------------------------------------------------------------
// Engine

#include <string>
// #include <core/io/resource.h>
struct ClassDB {
  template <typename M> static void bind_method(std::string a, M b);
  template <typename M> static void register_class();
};

inline std::string D_METHOD(std::string a, std::string b = "", std::string c = "") { return a+"::"+b+"::"+c; }

struct Engine {
  static Engine* get_singleton() { return nullptr; }
  bool is_editor_hint() { return false; }
  double get_physics_interpolation_fraction();
};

double get_physics_process_delta_time();

struct Space {};
struct World {
  Space* get_space() const;
};

// ---------------------------------------------------------------------------
// Node

#include <string>

struct NodePath : public std::string {
  NodePath(std::string s = "") : std::string(s) {}
  bool is_empty() const { return empty(); }
};
struct Object {
  template <typename T> T* cast_to(void *p);
};
struct Node : public Object {
  Node* get_parent();

  void set_process_internal(bool b);
  void set_physics_process_internal(bool b);
  bool is_inside_tree() const;
  Node* get_node_or_null(NodePath const&) const;
  NodePath get_path() const;
};

// ---------------------------------------------------------------------------
// Spatial
#include <cmath>
#include <cstdint>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/vector_angle.hpp>
#include <glm/gtc/epsilon.hpp>
// #include <scene/main/node.h>

#define GDCLASS(a,b)
#define Math_PI M_PI
#define Math_TAU 2.0 * M_PI

using String = std::string;

template <typename T>
struct Vector : public std::vector<T> {
  void invert() { std::reverse(std::vector<T>::begin(), std::vector<T>::end());  }
  void set(int key, T const& v);
};

template <typename T>
struct Set : public std::set<T> {
  // void invert() { std::reverse(std::vector<T>::begin(), std::vector<T>::end());  }
  // void set(int key, T const& v);
};

template <typename K, typename V>
struct Map {
  using S = std::map<K,V>;
  S value;
  bool has(K const& key) const { return value.find(key) != value.end(); }
  void insert(K const& key, V const& v);
  V& operator[](K const& key);
  const V operator[](K const& key) const;
};

struct Vector3 {
public:
  glm::vec3 value;
  operator glm::vec3() const { return value; }
  Vector3() : value(){}
  Vector3(glm::vec3 const& v) : value(v){}
  Vector3(double x, double y, double z) : value(x,y,z){}
  Vector3& operator+=(Vector3 const&);
  Vector3 normalized() const { return glm::normalize(value); }
  void normalize() { value = glm::normalize(value); }
  Vector3 cross(Vector3 const& o) const { return glm::cross(value, o.value); }
  float dot(glm::vec3 const& o) const { return glm::dot(value, o); }
  float length() const;
  float length_squared() const { return glm::length2(value); }
  void rotate(Vector3 axis, float angle);
  Vector3 rotated(Vector3 axis, float angle) const;
  float angle_to(Vector3 const& o) const { return glm::angle(glm::normalize(value), glm::normalize(o.value)); }
  float distance_squared_to(Vector3) const;
  float distance_to(Vector3 const&) const;
  Vector3 linear_interpolate(Vector3 const& other, float alpha) const;
  Vector3 cubic_interpolate(Vector3 const& other, Vector3 alpha, Vector3, float) const;
  Vector3 slerp(Vector3 const& other, float alpha) const;// { return glm::slerp(value, (glm::vec3)other, alpha); }
  Vector3 operator/(double) const;
  Vector3 operator*(double) const;
  friend Vector3 operator*(double, Vector3 const&);
  Vector3 operator*(Vector3) const;
  Vector3 operator+(Vector3) const;
  Vector3 operator-(Vector3) const;
  Vector3 operator-() const;
  float operator[](int i) const { return value[i]; }
  float& operator[](int i) { return value[i]; }
  Vector3 project(Vector3 const&) const;
  struct _X { Vector3* p; inline _X& operator=(float v) { p->x = v; return *this; }inline operator float() const & { return p->value.x; }} x {this};
  struct _Y { Vector3* p; inline _Y& operator=(float v) { p->y = v; return *this; }inline operator float() const & { return p->value.y; }} y {this};
  struct _Z { Vector3* p; inline _Z& operator=(float v) { p->z = v; return *this; }inline operator float() const & { return p->value.z; }} z {this};
};

struct Quat {
  glm::quat value;
  operator const glm::quat& () const { return value; };
  // operator const glm::quat() const;
  Quat() : value() {}
  Quat(glm::quat const& q) : value(q) {}
  Quat(Quat const& q) : value(q) {}
  explicit Quat(Vector3 const& euler) : value(euler.value) {}
  explicit Quat(Vector3 const& axis, float angle) : value(glm::angleAxis(angle, axis.value)) {}
  Quat& operator=(Vector3 const& euler) { value = glm::quat(euler); return *this; }
  Quat slerp(Quat const& other, float alpha) const { return glm::slerp(value, other.value, alpha); }
  Quat inverse() const { return glm::inverse(value); }
  Quat normalized() const { return glm::normalize(value); }
  void normalize() { value = glm::normalize(value); }
  Vector3 xform(Vector3 const& other) const { return value * other.value; }
  Vector3 get_euler() const { return glm::eulerAngles(value); }
  bool operator==(Quat const& o) const { return glm::all(glm::epsilonEqual(value, o.value, 0.0001f)); }
  Quat operator*(Quat const& o) const { return value * o.value; }
  Vector3 operator-() const;
  struct _W { Quat* p; inline _W& operator=(float v) { p->w = v; return *this; } inline operator float() const & { return p->value.w; }} w {this};
  struct _X { Quat* p; inline _X& operator=(float v) { p->x = v; return *this; }inline operator float() const & { return p->value.x; }} x {this};
  struct _Y { Quat* p; inline _Y& operator=(float v) { p->y = v; return *this; }inline operator float() const & { return p->value.y; }} y {this};
  struct _Z { Quat* p; inline _Z& operator=(float v) { p->z = v; return *this; }inline operator float() const & { return p->value.z; }} z {this};
};

struct Basis {
public:
  glm::mat3 value;
  operator glm::mat3() const;
  Basis() : value() {};
  Basis(Basis const& o) : value(o.value) {};
  Basis(glm::mat3 const& o) : value(o) {};
  Basis(glm::mat4 const& o) : value(o) {};
  Basis(Quat const& o) : Basis(glm::toMat3((glm::quat)o)) {};
  Basis(Vector3 const& a, Vector3 const& b, Vector3 const& c) : value(a,b,c) {};
  Basis(Vector3 const& axis, float angle) : value(glm::toMat3(glm::angleAxis(angle, axis.value))) {};
  Basis inverse() const { return glm::inverse(value); }
  Basis rotate_local(Vector3 const&, float);
  Basis rotated_local(Vector3 const& axis, float angle) const;
  void orthonormalize();
  void rotate(glm::vec3 const&, float);
  Vector3 xform(Vector3 const&) const;
  Vector3 xform_inv(Vector3 const&) const;
  Basis orthonormalized() const;
  Basis slerp(Basis const& other, float alpha) const;
  friend Basis operator*(Basis const& lhs, Basis const& rhs);
  Vector3 operator[](int) const;
  Vector3& operator[](int);
  Quat get_rotation_quat() const { return glm::toQuat(value); }
  Quat get_quat() const { return glm::toQuat(value); }
};

struct Transform {
public:
  Basis basis;
  Vector3 origin;
  glm::mat4 as_mat4() const { return glm::translate(glm::mat4(basis.value), origin.value); }
  Transform() {}
  Transform(glm::mat4 const& o) : basis(glm::toQuat(o)), origin(o[3]) {}
  Transform(Quat const& q, Vector3 const& v) : basis(q), origin(v) {}
  Transform(Basis const& q, Vector3 const& v) : basis(q), origin(v) {}
  Transform(Quat const& q) : basis(q) {}
  Transform(Basis const& o) : basis(o) {}
  Transform(Vector3 const& o) : origin(o) {}

  Basis get_basis() const { return basis; }

  Vector3 get_origin() const { return origin; }
  void set_origin(Vector3 const& v) { origin = v; }
  Transform affine_inverse() const { return glm::inverse(as_mat4()); }
  Vector3 xform(Vector3 const&);
  Vector3 xform_inv(Vector3 const&);
  Transform translated(Vector3 const&);
  Transform orthonormalized() const;
  void orthonormalize();
  void rotate_basis(Vector3 const&, float);
  Transform& operator*=(Transform const& lhs);
  Transform operator*(Transform const&) const;
  Transform operator*(Basis const&) const;

  Transform interpolate_with(Transform const&, float) const;
  void translate(Vector3 const&);
};


#include <glm/gtx/string_cast.hpp>
namespace glm {
  template<> inline std::string to_string(Quat const& q) { return glm::to_string<glm::quat>(q.value); }
  template<> inline std::string to_string(Vector3 const& q) { return glm::to_string<glm::vec3>(q.value); }
  template<> inline std::string to_string(Basis const& q) { return glm::to_string<glm::mat3>(q.value); }
  template<> inline std::string to_string(Transform const& q) { return glm::to_string<glm::mat4>(q.as_mat4()); }
}

namespace Math {
  inline float deg2rad (float degrees) { return degrees * M_PI / 180.0; }
  inline float rad2deg(float angle) { return angle * 180.0 / M_PI; }
  inline float fabs(float v) { return ::fabs(v); }
  inline float cos(float v) { return ::cos(v); }
  inline float fmod(float a, float b) { return ::fmod(a,b); }
}
#define abs(x) fabs(x)
//float abs(float v) { return fabs(v); }

struct Spatial : public Node {
  bool is_inside_world();
  Transform get_global_transform();
  Ref<World> get_world();
};

// ---------------------------------------------------------------------------
// Skeleton

using BoneId = int16_t;

struct Skeleton : public Spatial {
  int get_bone_count() { return -1; }
  std::string get_bone_name(int i) { return ":"; }
  Transform get_bone_global_pose(int);
  Transform get_bone_rest(int);
  BoneId get_bone_parent(int);
  Transform set_bone_global_pose_override(int, Transform, int);
  BoneId find_bone(String);
};


// ---------------------------------------------------------------------------
//
struct Variant {};

// ---------------------------------------------------------------------------
static constexpr auto PROPERTY_HINT_ENUM = 1;
static constexpr auto PROPERTY_HINT_NONE= 0;
static constexpr auto NOTIFICATION_READY = 0;
static constexpr auto NOTIFICATION_INTERNAL_PROCESS = 1;
static constexpr auto NOTIFICATION_INTERNAL_PHYSICS_PROCESS = 2;
#define ERR_FAIL_COND(...)
struct PropertyInfo {
  int hint;
  std::string name;
  std::string hint_string;
};

#define ADD_PROPERTY(...)
#define ADD_GROUP(...)

// ---------------------------------------------------------------------------
// Physics
using RID = uint16_t;
struct Vector3;
struct PhysicsDirectSpaceState {
  struct RayResult {
    void* collider;
    Vector3 position;
    Vector3 normal;
  };
  bool intersect_ray(Vector3,Vector3,RayResult,Set<RID>,int,int,int);
};

struct PhysicsServer {
  static PhysicsServer* get_singleton() { return nullptr; }
  PhysicsDirectSpaceState* space_get_direct_state(Space*) const;
};
