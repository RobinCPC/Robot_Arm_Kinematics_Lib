#include "application.h"
// Refernce: Cherno video https://www.youtube.com/watch?v=vWXrFetSH8w

#include "imgui.h"
#include "imgui_internal.h"
#include "ImGuizmo.h"

#include <vector>


bool useWindow = true;
int gizmoCount = 1;
float camDistance = 8.f;
static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
static ImDrawList* g_drawlist;
static float g_mX, g_mY, g_mWidth, g_mHeight;


float objectMatrix[4][16] =
{
  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 3.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    2.f, 0.f, 3.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    2.f, 0.f, 5.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 5.f, 1.f }
};

static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  0.f, 0.f, 0.f, 1.f
};

void FPU_MatrixF_x_MatrixF(const float *a, const float *b, float *r)
{
  r[0] = a[0] * b[0] + a[1] * b[4] + a[2] * b[8] + a[3] * b[12];
  r[1] = a[0] * b[1] + a[1] * b[5] + a[2] * b[9] + a[3] * b[13];
  r[2] = a[0] * b[2] + a[1] * b[6] + a[2] * b[10] + a[3] * b[14];
  r[3] = a[0] * b[3] + a[1] * b[7] + a[2] * b[11] + a[3] * b[15];

  r[4] = a[4] * b[0] + a[5] * b[4] + a[6] * b[8] + a[7] * b[12];
  r[5] = a[4] * b[1] + a[5] * b[5] + a[6] * b[9] + a[7] * b[13];
  r[6] = a[4] * b[2] + a[5] * b[6] + a[6] * b[10] + a[7] * b[14];
  r[7] = a[4] * b[3] + a[5] * b[7] + a[6] * b[11] + a[7] * b[15];

  r[8] = a[8] * b[0] + a[9] * b[4] + a[10] * b[8] + a[11] * b[12];
  r[9] = a[8] * b[1] + a[9] * b[5] + a[10] * b[9] + a[11] * b[13];
  r[10] = a[8] * b[2] + a[9] * b[6] + a[10] * b[10] + a[11] * b[14];
  r[11] = a[8] * b[3] + a[9] * b[7] + a[10] * b[11] + a[11] * b[15];

  r[12] = a[12] * b[0] + a[13] * b[4] + a[14] * b[8] + a[15] * b[12];
  r[13] = a[12] * b[1] + a[13] * b[5] + a[14] * b[9] + a[15] * b[13];
  r[14] = a[12] * b[2] + a[13] * b[6] + a[14] * b[10] + a[15] * b[14];
  r[15] = a[12] * b[3] + a[13] * b[7] + a[14] * b[11] + a[15] * b[15];
}

void Frustum(float left, float right, float bottom, float top, float znear, float zfar, float* m16)
{
   float temp, temp2, temp3, temp4;
   temp = 2.0f * znear;
   temp2 = right - left;
   temp3 = top - bottom;
   temp4 = zfar - znear;
   m16[0] = temp / temp2;
   m16[1] = 0.0;
   m16[2] = 0.0;
   m16[3] = 0.0;
   m16[4] = 0.0;
   m16[5] = temp / temp3;
   m16[6] = 0.0;
   m16[7] = 0.0;
   m16[8] = (right + left) / temp2;
   m16[9] = (top + bottom) / temp3;
   m16[10] = (-zfar - znear) / temp4;
   m16[11] = -1.0f;
   m16[12] = 0.0;
   m16[13] = 0.0;
   m16[14] = (-temp * zfar) / temp4;
   m16[15] = 0.0;
}

struct matrix_t;
struct vec_t
{
public:
   float x, y, z, w;

   void Lerp(const vec_t& v, float t)
   {
      x += (v.x - x) * t;
      y += (v.y - y) * t;
      z += (v.z - z) * t;
      w += (v.w - w) * t;
   }

   void Set(float v) { x = y = z = w = v; }
   void Set(float _x, float _y, float _z = 0.f, float _w = 0.f) { x = _x; y = _y; z = _z; w = _w; }

   vec_t& operator -= (const vec_t& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; return *this; }
   vec_t& operator += (const vec_t& v) { x += v.x; y += v.y; z += v.z; w += v.w; return *this; }
   vec_t& operator *= (const vec_t& v) { x *= v.x; y *= v.y; z *= v.z; w *= v.w; return *this; }
   vec_t& operator *= (float v) { x *= v;    y *= v;    z *= v;    w *= v;    return *this; }

   vec_t operator * (float f) const;
   vec_t operator - () const;
   vec_t operator - (const vec_t& v) const;
   vec_t operator + (const vec_t& v) const;
   vec_t operator * (const vec_t& v) const;

   const vec_t& operator + () const { return (*this); }
   float Length() const { return sqrtf(x * x + y * y + z * z); };
   float LengthSq() const { return (x * x + y * y + z * z); };
   vec_t Normalize() { (*this) *= (1.f / ( Length() > FLT_EPSILON ? Length() : FLT_EPSILON ) ); return (*this); }
   vec_t Normalize(const vec_t& v) { this->Set(v.x, v.y, v.z, v.w); this->Normalize(); return (*this); }
   vec_t Abs() const;

   void Cross(const vec_t& v)
   {
      vec_t res;
      res.x = y * v.z - z * v.y;
      res.y = z * v.x - x * v.z;
      res.z = x * v.y - y * v.x;

      x = res.x;
      y = res.y;
      z = res.z;
      w = 0.f;
   }

   void Cross(const vec_t& v1, const vec_t& v2)
   {
      x = v1.y * v2.z - v1.z * v2.y;
      y = v1.z * v2.x - v1.x * v2.z;
      z = v1.x * v2.y - v1.y * v2.x;
      w = 0.f;
   }

   float Dot(const vec_t& v) const
   {
      return (x * v.x) + (y * v.y) + (z * v.z) + (w * v.w);
   }

   float Dot3(const vec_t& v) const
   {
      return (x * v.x) + (y * v.y) + (z * v.z);
   }

   void Transform(const matrix_t& matrix);
   void Transform(const vec_t& s, const matrix_t& matrix);

   void TransformVector(const matrix_t& matrix);
   void TransformPoint(const matrix_t& matrix);
   void TransformVector(const vec_t& v, const matrix_t& matrix) { (*this) = v; this->TransformVector(matrix); }
   void TransformPoint(const vec_t& v, const matrix_t& matrix) { (*this) = v; this->TransformPoint(matrix); }

   float& operator [] (size_t index) { return ((float*)&x)[index]; }
   const float& operator [] (size_t index) const { return ((float*)&x)[index]; }
   bool operator!=(const vec_t& other) const { return memcmp(this, &other, sizeof(vec_t)) != 0; }
};

vec_t makeVect(float _x, float _y, float _z = 0.f, float _w = 0.f) { vec_t res; res.x = _x; res.y = _y; res.z = _z; res.w = _w; return res; }
vec_t makeVect(ImVec2 v) { vec_t res; res.x = v.x; res.y = v.y; res.z = 0.f; res.w = 0.f; return res; }
vec_t vec_t::operator * (float f) const { return makeVect(x * f, y * f, z * f, w * f); }
vec_t vec_t::operator - () const { return makeVect(-x, -y, -z, -w); }
vec_t vec_t::operator - (const vec_t& v) const { return makeVect(x - v.x, y - v.y, z - v.z, w - v.w); }
vec_t vec_t::operator + (const vec_t& v) const { return makeVect(x + v.x, y + v.y, z + v.z, w + v.w); }
vec_t vec_t::operator * (const vec_t& v) const { return makeVect(x * v.x, y * v.y, z * v.z, w * v.w); }
vec_t vec_t::Abs() const { return makeVect(fabsf(x), fabsf(y), fabsf(z)); }

struct matrix_t
{
public:
  union
  {
    float m[4][4];
    float m16[16];
    struct
    {
      vec_t right, up, dir, position;
    } v;
    vec_t component[4];
  };

  operator float *() { return m16; }
  operator const float *() const { return m16; }
  void Translation(float _x, float _y, float _z) { this->Translation(makeVect(_x, _y, _z)); }

  void Translation(const vec_t &vt)
  {
    v.right.Set(1.f, 0.f, 0.f, 0.f);
    v.up.Set(0.f, 1.f, 0.f, 0.f);
    v.dir.Set(0.f, 0.f, 1.f, 0.f);
    v.position.Set(vt.x, vt.y, vt.z, 1.f);
  }

  void Scale(float _x, float _y, float _z)
  {
    v.right.Set(_x, 0.f, 0.f, 0.f);
    v.up.Set(0.f, _y, 0.f, 0.f);
    v.dir.Set(0.f, 0.f, _z, 0.f);
    v.position.Set(0.f, 0.f, 0.f, 1.f);
  }
  void Scale(const vec_t &s) { Scale(s.x, s.y, s.z); }

  matrix_t &operator*=(const matrix_t &mat)
  {
    matrix_t tmpMat;
    tmpMat = *this;
    tmpMat.Multiply(mat);
    *this = tmpMat;
    return *this;
  }
  matrix_t operator*(const matrix_t &mat) const
  {
    matrix_t matT;
    matT.Multiply(*this, mat);
    return matT;
  }

  void Multiply(const matrix_t &matrix)
  {
    matrix_t tmp;
    tmp = *this;

    FPU_MatrixF_x_MatrixF((float *)&tmp, (float *)&matrix, (float *)this);
  }

  void Multiply(const matrix_t &m1, const matrix_t &m2)
  {
    FPU_MatrixF_x_MatrixF((float *)&m1, (float *)&m2, (float *)this);
  }

  float GetDeterminant() const
  {
    return m[0][0] * m[1][1] * m[2][2] + m[0][1] * m[1][2] * m[2][0] + m[0][2] * m[1][0] * m[2][1] -
           m[0][2] * m[1][1] * m[2][0] - m[0][1] * m[1][0] * m[2][2] - m[0][0] * m[1][2] * m[2][1];
  }

  float Inverse(const matrix_t &srcMatrix, bool affine = false);
  void SetToIdentity()
  {
    v.right.Set(1.f, 0.f, 0.f, 0.f);
    v.up.Set(0.f, 1.f, 0.f, 0.f);
    v.dir.Set(0.f, 0.f, 1.f, 0.f);
    v.position.Set(0.f, 0.f, 0.f, 1.f);
  }
  void Transpose()
  {
    matrix_t tmpm;
    for (int l = 0; l < 4; l++)
    {
      for (int c = 0; c < 4; c++)
      {
        tmpm.m[l][c] = m[c][l];
      }
    }
    (*this) = tmpm;
  }

  void RotationAxis(const vec_t &axis, float angle);

  void OrthoNormalize()
  {
    v.right.Normalize();
    v.up.Normalize();
    v.dir.Normalize();
  }
};

void vec_t::Transform(const matrix_t& matrix)
{
   vec_t out;

   out.x = x * matrix.m[0][0] + y * matrix.m[1][0] + z * matrix.m[2][0] + w * matrix.m[3][0];
   out.y = x * matrix.m[0][1] + y * matrix.m[1][1] + z * matrix.m[2][1] + w * matrix.m[3][1];
   out.z = x * matrix.m[0][2] + y * matrix.m[1][2] + z * matrix.m[2][2] + w * matrix.m[3][2];
   out.w = x * matrix.m[0][3] + y * matrix.m[1][3] + z * matrix.m[2][3] + w * matrix.m[3][3];

   x = out.x;
   y = out.y;
   z = out.z;
   w = out.w;
}

void vec_t::Transform(const vec_t& s, const matrix_t& matrix)
{
   *this = s;
   Transform(matrix);
}

void vec_t::TransformPoint(const matrix_t& matrix)
{
   vec_t out;

   out.x = x * matrix.m[0][0] + y * matrix.m[1][0] + z * matrix.m[2][0] + matrix.m[3][0];
   out.y = x * matrix.m[0][1] + y * matrix.m[1][1] + z * matrix.m[2][1] + matrix.m[3][1];
   out.z = x * matrix.m[0][2] + y * matrix.m[1][2] + z * matrix.m[2][2] + matrix.m[3][2];
   out.w = x * matrix.m[0][3] + y * matrix.m[1][3] + z * matrix.m[2][3] + matrix.m[3][3];

   x = out.x;
   y = out.y;
   z = out.z;
   w = out.w;
}

void vec_t::TransformVector(const matrix_t& matrix)
{
   vec_t out;

   out.x = x * matrix.m[0][0] + y * matrix.m[1][0] + z * matrix.m[2][0];
   out.y = x * matrix.m[0][1] + y * matrix.m[1][1] + z * matrix.m[2][1];
   out.z = x * matrix.m[0][2] + y * matrix.m[1][2] + z * matrix.m[2][2];
   out.w = x * matrix.m[0][3] + y * matrix.m[1][3] + z * matrix.m[2][3];

   x = out.x;
   y = out.y;
   z = out.z;
   w = out.w;
}

ImVec2 worldToPos(const vec_t &worldPos, const matrix_t &mat, ImVec2 position = ImVec2(g_mX, g_mY), ImVec2 size = ImVec2(g_mWidth, g_mHeight))
{
  vec_t trans;
  trans.TransformPoint(worldPos, mat);
  trans *= 0.5f / trans.w;
  trans += makeVect(0.5f, 0.5f);
  trans.y = 1.f - trans.y;
  trans.x *= size.x;
  trans.y *= size.y;
  trans.x += position.x;
  trans.y += position.y;
  return ImVec2(trans.x, trans.y);
}

void ComputeFrustumPlanes(vec_t *frustum, const float *clip)
{
  frustum[0].x = clip[3] - clip[0];
  frustum[0].y = clip[7] - clip[4];
  frustum[0].z = clip[11] - clip[8];
  frustum[0].w = clip[15] - clip[12];

  frustum[1].x = clip[3] + clip[0];
  frustum[1].y = clip[7] + clip[4];
  frustum[1].z = clip[11] + clip[8];
  frustum[1].w = clip[15] + clip[12];

  frustum[2].x = clip[3] + clip[1];
  frustum[2].y = clip[7] + clip[5];
  frustum[2].z = clip[11] + clip[9];
  frustum[2].w = clip[15] + clip[13];

  frustum[3].x = clip[3] - clip[1];
  frustum[3].y = clip[7] - clip[5];
  frustum[3].z = clip[11] - clip[9];
  frustum[3].w = clip[15] - clip[13];

  frustum[4].x = clip[3] - clip[2];
  frustum[4].y = clip[7] - clip[6];
  frustum[4].z = clip[11] - clip[10];
  frustum[4].w = clip[15] - clip[14];

  frustum[5].x = clip[3] + clip[2];
  frustum[5].y = clip[7] + clip[6];
  frustum[5].z = clip[11] + clip[10];
  frustum[5].w = clip[15] + clip[14];

  for (int i = 0; i < 6; i++)
  {
    frustum[i].Normalize();
  }
}

void Perspective(float fovyInDegrees, float aspectRatio, float znear, float zfar, float* m16)
{
   float ymax, xmax;
   ymax = znear * tanf(fovyInDegrees * 3.141592f / 180.0f);
   xmax = ymax * aspectRatio;
   Frustum(-xmax, xmax, -ymax, ymax, znear, zfar, m16);
}

void Cross(const float* a, const float* b, float* r)
{
   r[0] = a[1] * b[2] - a[2] * b[1];
   r[1] = a[2] * b[0] - a[0] * b[2];
   r[2] = a[0] * b[1] - a[1] * b[0];
}

float Dot(const float* a, const float* b)
{
   return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void Normalize(const float* a, float* r)
{
   float il = 1.f / (sqrtf(Dot(a, a)) + FLT_EPSILON);
   r[0] = a[0] * il;
   r[1] = a[1] * il;
   r[2] = a[2] * il;
}

void LookAt(const float* eye, const float* at, const float* up, float* m16)
{
   float X[3], Y[3], Z[3], tmp[3];

   tmp[0] = eye[0] - at[0];
   tmp[1] = eye[1] - at[1];
   tmp[2] = eye[2] - at[2];
   Normalize(tmp, Z);
   Normalize(up, Y);

   Cross(Y, Z, tmp);
   Normalize(tmp, X);

   Cross(Z, X, tmp);
   Normalize(tmp, Y);

   m16[0] = X[0];
   m16[1] = Y[0];
   m16[2] = Z[0];
   m16[3] = 0.0f;
   m16[4] = X[1];
   m16[5] = Y[1];
   m16[6] = Z[1];
   m16[7] = 0.0f;
   m16[8] = X[2];
   m16[9] = Y[2];
   m16[10] = Z[2];
   m16[11] = 0.0f;
   m16[12] = -Dot(X, eye);
   m16[13] = -Dot(Y, eye);
   m16[14] = -Dot(Z, eye);
   m16[15] = 1.0f;
}

void OrthoGraphic(const float l, float r, float b, const float t, float zn, const float zf, float* m16)
{
   m16[0] = 2 / (r - l);
   m16[1] = 0.0f;
   m16[2] = 0.0f;
   m16[3] = 0.0f;
   m16[4] = 0.0f;
   m16[5] = 2 / (t - b);
   m16[6] = 0.0f;
   m16[7] = 0.0f;
   m16[8] = 0.0f;
   m16[9] = 0.0f;
   m16[10] = 1.0f / (zf - zn);
   m16[11] = 0.0f;
   m16[12] = (l + r) / (l - r);
   m16[13] = (t + b) / (b - t);
   m16[14] = zn / (zn - zf);
   m16[15] = 1.0f;
}

void EditTransform(float* cameraView, float* cameraProjection, float* matrix, bool editTransformDecomposition)
{
   static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::LOCAL);
   static bool useSnap = false;
   static float snap[3] = { 1.f, 1.f, 1.f };
   static float bounds[] = { -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f };
   static float boundsSnap[] = { 0.1f, 0.1f, 0.1f };
   static bool boundSizing = false;
   static bool boundSizingSnap = false;

   if (editTransformDecomposition)
   {
      if (ImGui::IsKeyPressed(ImGuiKey_T))
         mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
      if (ImGui::IsKeyPressed(ImGuiKey_E))
         mCurrentGizmoOperation = ImGuizmo::ROTATE;
      if (ImGui::IsKeyPressed(ImGuiKey_R)) // r Key
         mCurrentGizmoOperation = ImGuizmo::SCALE;
      if (ImGui::RadioButton("Translate", mCurrentGizmoOperation == ImGuizmo::TRANSLATE))
         mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
      ImGui::SameLine();
      if (ImGui::RadioButton("Rotate", mCurrentGizmoOperation == ImGuizmo::ROTATE))
         mCurrentGizmoOperation = ImGuizmo::ROTATE;
      ImGui::SameLine();
      if (ImGui::RadioButton("Scale", mCurrentGizmoOperation == ImGuizmo::SCALE))
         mCurrentGizmoOperation = ImGuizmo::SCALE;
      if (ImGui::RadioButton("Universal", mCurrentGizmoOperation == ImGuizmo::UNIVERSAL))
         mCurrentGizmoOperation = ImGuizmo::UNIVERSAL;
      float matrixTranslation[3], matrixRotation[3], matrixScale[3];
      ImGuizmo::DecomposeMatrixToComponents(matrix, matrixTranslation, matrixRotation, matrixScale);
      ImGui::InputFloat3("Tr", matrixTranslation);
      ImGui::InputFloat3("Rt", matrixRotation);
      ImGui::InputFloat3("Sc", matrixScale);
      ImGuizmo::RecomposeMatrixFromComponents(matrixTranslation, matrixRotation, matrixScale, matrix);

      if (mCurrentGizmoOperation != ImGuizmo::SCALE)
      {
         if (ImGui::RadioButton("Local", mCurrentGizmoMode == ImGuizmo::LOCAL))
            mCurrentGizmoMode = ImGuizmo::LOCAL;
         ImGui::SameLine();
         if (ImGui::RadioButton("World", mCurrentGizmoMode == ImGuizmo::WORLD))
            mCurrentGizmoMode = ImGuizmo::WORLD;
      }
      if (ImGui::IsKeyPressed(ImGuiKey_S))
         useSnap = !useSnap;
      ImGui::Checkbox("##UseSnap", &useSnap);
      ImGui::SameLine();

      switch (mCurrentGizmoOperation)
      {
      case ImGuizmo::TRANSLATE:
         ImGui::InputFloat3("Snap", &snap[0]);
         break;
      case ImGuizmo::ROTATE:
         ImGui::InputFloat("Angle Snap", &snap[0]);
         break;
      case ImGuizmo::SCALE:
         ImGui::InputFloat("Scale Snap", &snap[0]);
         break;
      }
      ImGui::Checkbox("Bound Sizing", &boundSizing);
      if (boundSizing)
      {
         ImGui::PushID(3);
         ImGui::Checkbox("##BoundSizing", &boundSizingSnap);
         ImGui::SameLine();
         ImGui::InputFloat3("Snap", boundsSnap);
         ImGui::PopID();
      }
   }

   ImGuiIO& io = ImGui::GetIO();
   float viewManipulateRight = io.DisplaySize.x;
   float viewManipulateTop = 0;
   static ImGuiWindowFlags gizmoWindowFlags = 0;
   if (useWindow)
   {
      ImGui::SetNextWindowSize(ImVec2(800, 400), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowPos(ImVec2(100,100), ImGuiCond_FirstUseEver);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, (ImVec4)ImColor(0.35f, 0.3f, 0.3f));
      ImGui::Begin("Gizmo", 0, gizmoWindowFlags);
      ImGuizmo::SetDrawlist();
      float windowWidth = (float)ImGui::GetWindowWidth();
      float windowHeight = (float)ImGui::GetWindowHeight();
      g_drawlist = ImGui::GetWindowDrawList();
      g_mWidth = windowWidth;
      g_mHeight = windowHeight;
      g_mX = ImGui::GetWindowPos().x;
      g_mY = ImGui::GetWindowPos().y;
      ImGuizmo::SetRect(ImGui::GetWindowPos().x, ImGui::GetWindowPos().y, windowWidth, windowHeight);
      viewManipulateRight = ImGui::GetWindowPos().x + windowWidth;
      viewManipulateTop = ImGui::GetWindowPos().y;
      ImGuiWindow* window = ImGui::GetCurrentWindow();
      gizmoWindowFlags = ImGui::IsWindowHovered() && ImGui::IsMouseHoveringRect(window->InnerRect.Min, window->InnerRect.Max) ? ImGuiWindowFlags_NoMove : 0;
   }
   else
   {
      ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
   }

   ImGuizmo::DrawGrid(cameraView, cameraProjection, identityMatrix, 100.f);
   ImGuizmo::DrawCubes(cameraView, cameraProjection, &objectMatrix[0][0], gizmoCount);
   ImGuizmo::Manipulate(cameraView, cameraProjection, mCurrentGizmoOperation, mCurrentGizmoMode, matrix, NULL, useSnap ? &snap[0] : NULL, boundSizing ? bounds : NULL, boundSizingSnap ? boundsSnap : NULL);

   ImGuizmo::ViewManipulate(cameraView, camDistance, ImVec2(viewManipulateRight - 128, viewManipulateTop), ImVec2(128, 128), 0x10101010);

   if (useWindow)
   {
      ImGui::End();
      ImGui::PopStyleColor(1);
   }
}

namespace MyApp
{
  std::unique_ptr<rb::kin::Artic> robot {new rb::kin::Artic};   // with C++11 support
  //std::unique_ptr<rb::kin::Artic> robot = std::make_unique<rb::kin::Artic>();   // with C++14 support
  rb::kin::ArmPose pose_tcp;
  rb::kin::ArmAxisValue joint_value;
  bool show_gizmo_window = true;

  // Initial setting for ImGuizmo
  int lastUsing = 0;

  float cameraView[16] =
  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f };

  float cameraProjection[16];

  // Camera projection
  bool isPerspective = true;
  float fov = 27.f;
  float viewWidth = 10.f; // for orthographic
  float camYAngle = 165.f / 180.f * 3.14159f;
  float camXAngle = 32.f / 180.f * 3.14159f;

  bool firstFrame = true;

  void RenderUI()
  {

    /*
    static bool opt_fullscreen = true;
    static bool opt_padding = false;
    static ImGuiDockNodeFlags dockspace_flags = ImGuiDockNodeFlags_None;

    // We are using the ImGuiWindowFlags_NoDocking flag to make the parent window not dockable into,
    // because it would be confusing to have two docking targets within each others.
    ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
    if (opt_fullscreen)
    {
        const ImGuiViewport* viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->WorkPos);
        ImGui::SetNextWindowSize(viewport->WorkSize);
        ImGui::SetNextWindowViewport(viewport->ID);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
        window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
        window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
    }
    else
    {
        dockspace_flags &= ~ImGuiDockNodeFlags_PassthruCentralNode;
    }

    // When using ImGuiDockNodeFlags_PassthruCentralNode, DockSpace() will render our background
    // and handle the pass-thru hole, so we ask Begin() to not render a background.
    if (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode)
        window_flags |= ImGuiWindowFlags_NoBackground;

    // Important: note that we proceed even if Begin() returns false (aka window is collapsed).
    // This is because we want to keep our DockSpace() active. If a DockSpace() is inactive,
    // all active windows docked into it will lose their parent and become undocked.
    // We cannot preserve the docking relationship between an active window and an inactive docking, otherwise
    // any change of dockspace/settings would lead to windows being stuck in limbo and never being visible.
    if (!opt_padding)
        ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpace Demo", nullptr, window_flags);
    if (!opt_padding)
        ImGui::PopStyleVar();

    if (opt_fullscreen)
        ImGui::PopStyleVar(2);

    // Submit the DockSpace
    ImGuiIO& io = ImGui::GetIO();
    if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
    {
        ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
        ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), dockspace_flags);
    }

    // PassthruCenterNode alway on
    dockspace_flags |= ImGuiDockNodeFlags_PassthruCentralNode;

    if (ImGui::BeginMenuBar())
    {
        if (ImGui::BeginMenu("Options"))
        {
            // Disabling fullscreen would allow the window to be moved to the front of other windows,
            // which we can't undo at the moment without finer window depth/z control.
            ImGui::MenuItem("Fullscreen", NULL, &opt_fullscreen);
            ImGui::MenuItem("Padding", NULL, &opt_padding);
            ImGui::Separator();

            if (ImGui::MenuItem("Flag: NoSplit",                "", (dockspace_flags & ImGuiDockNodeFlags_NoSplit) != 0))                 { dockspace_flags ^= ImGuiDockNodeFlags_NoSplit; }
            if (ImGui::MenuItem("Flag: NoResize",               "", (dockspace_flags & ImGuiDockNodeFlags_NoResize) != 0))                { dockspace_flags ^= ImGuiDockNodeFlags_NoResize; }
            if (ImGui::MenuItem("Flag: NoDockingInCentralNode", "", (dockspace_flags & ImGuiDockNodeFlags_NoDockingInCentralNode) != 0))  { dockspace_flags ^= ImGuiDockNodeFlags_NoDockingInCentralNode; }
            if (ImGui::MenuItem("Flag: AutoHideTabBar",         "", (dockspace_flags & ImGuiDockNodeFlags_AutoHideTabBar) != 0))          { dockspace_flags ^= ImGuiDockNodeFlags_AutoHideTabBar; }
            if (ImGui::MenuItem("Flag: PassthruCentralNode",    "", (dockspace_flags & ImGuiDockNodeFlags_PassthruCentralNode) != 0, opt_fullscreen)) { dockspace_flags ^= ImGuiDockNodeFlags_PassthruCentralNode; }
            ImGui::Separator();
            ImGui::EndMenu();
        }

        ImGui::EndMenuBar();
    }
    */

    ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    // Add ImGuizmo
    if(show_gizmo_window)
    {
    ImGuiIO& io = ImGui::GetIO();
    if (isPerspective)
    {
      Perspective(fov, io.DisplaySize.x / io.DisplaySize.y, 0.1f, 100.f, cameraProjection);
    }
    else
    {
      float viewHeight = viewWidth * io.DisplaySize.y / io.DisplaySize.x;
      OrthoGraphic(-viewWidth, viewWidth, -viewHeight, viewHeight, 1000.f, -1000.f, cameraProjection);
    }
    ImGuizmo::SetOrthographic(!isPerspective);
    ImGuizmo::BeginFrame();

    ImGui::SetNextWindowPos(ImVec2(1024, 100), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(256, 256), ImGuiCond_Appearing);

    // create a window and insert the inspector
    ImGui::SetNextWindowPos(ImVec2(200, 400), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(320, 340), ImGuiCond_FirstUseEver);
    ImGui::Begin("Editor");
    /*  only use gimzo with window, need tune Full view  with MainViewport docking
    if (ImGui::RadioButton("Full view", !useWindow)) useWindow = false;
    ImGui::SameLine();
    if (ImGui::RadioButton("Window", useWindow)) useWindow = true;
    */

    ImGui::Text("Camera");
    bool viewDirty = false;
    if (ImGui::RadioButton("Perspective", isPerspective)) isPerspective = true;
    ImGui::SameLine();
    if (ImGui::RadioButton("Orthographic", !isPerspective)) isPerspective = false;
    if (isPerspective)
    {
      ImGui::SliderFloat("Fov", &fov, 20.f, 110.f);
    }
    else
    {
      ImGui::SliderFloat("Otho width", &viewWidth, 1, 20);
    }
    viewDirty |= ImGui::SliderFloat("Distance", &camDistance, 1.f, 10.f);
    ImGui::SliderInt("Gizmo count", &gizmoCount, 1, 4);

    if (viewDirty || firstFrame)
    {
      float eye[] = { cosf(camYAngle) * cosf(camXAngle) * camDistance, sinf(camXAngle) * camDistance, sinf(camYAngle) * cosf(camXAngle) * camDistance };
      float at[] = { 0.f, 0.f, 0.f };
      float up[] = { 0.f, 1.f, 0.f };
      LookAt(eye, at, up, cameraView);
      firstFrame = false;
    }

    ImGui::Text("X: %f Y: %f", io.MousePos.x, io.MousePos.y);
    if (ImGuizmo::IsUsing())
    {
      ImGui::Text("Using gizmo");
    }
    else
    {
      ImGui::Text(ImGuizmo::IsOver()?"Over gizmo":"");
      ImGui::SameLine();
      ImGui::Text(ImGuizmo::IsOver(ImGuizmo::TRANSLATE) ? "Over translate gizmo" : "");
      ImGui::SameLine();
      ImGui::Text(ImGuizmo::IsOver(ImGuizmo::ROTATE) ? "Over rotate gizmo" : "");
      ImGui::SameLine();
      ImGui::Text(ImGuizmo::IsOver(ImGuizmo::SCALE) ? "Over scale gizmo" : "");
    }
    ImGui::Separator();
    for (int matId = 0; matId < gizmoCount; matId++)
    {
      ImGuizmo::SetID(matId);

      EditTransform(cameraView, cameraProjection, objectMatrix[matId], lastUsing == matId);
      if (ImGuizmo::IsUsing())
      {
        lastUsing = matId;
      }
    }

    ImGui::End(); // Begin Editor
    }


    // Build main kinDemo window
    {
      ImGui::SetNextWindowSize(ImVec2(740, 350), ImGuiCond_Once);
      //ImGui::SetWindowSize(ImVec2(740, 350), ImGuiCond_FirstUseEver);
      ImGui::Begin("KinDemo", nullptr);

      // make dummy DH table by vector, vector float
      static std::vector<std::vector<float>> dh_table = {
        // a   alpha  d  theta up  down
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0},
        {  0.0,  0.0,  0.0,  0.0,  0.0,  0.0}
      };
      static std::vector<std::vector<float>> sol_table = {
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 1
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 2
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 3
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 4
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 5
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 6
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}, // sol 7
        {  0.0,  0.0,   0.0,   0.0,    0.0,    0.0}  // sol 8
      };
      static float cart_inp[3] = { 0.0, 0.0, 0.0 };
      static float coor_inp[3] = { 0.0, 0.0, 0.0 };

      static char ik_result_str[128] = "Initial";
      const char* sol_combText[] = {"solution_1", "solution_2", "solution_3", "solution_4",
                                  "solution_5", "solution_6", "solution_7", "solution_8"};
      static int sol_current_idx = 0;
      const char *combo_preview_value = sol_combText[sol_current_idx]; // Pass in the preview value visible before opening the combo (it could be anything)

      if (ImGui::BeginTable("DH-Table", 7, ImGuiTableFlags_Borders, ImVec2(480, 155)))
      {
        ImGui::TableSetupColumn("Jnt#");
        ImGui::TableSetupColumn("A");
        ImGui::TableSetupColumn("Alpha");
        ImGui::TableSetupColumn("D");
        ImGui::TableSetupColumn("Theta");
        ImGui::TableSetupColumn("Up");
        ImGui::TableSetupColumn("Down");
        ImGui::TableHeadersRow();

        for (int row = 0; row < 6; row++)
        {
          ImGui::TableNextRow();
          if (row == 0)
          {
            // Setup ItemWidth once (instead of setting up every time, which is also possible but less efficient)
            for (int col = 0; col < 7; col++)
            {
              ImGui::TableSetColumnIndex(col);
              ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
            }
          }

          // Draw our contents
          ImGui::PushID(row);
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("Joint %d", row);
          ImGui::TableSetColumnIndex(1);
          ImGui::InputFloat("##a", &dh_table[row][0], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(2);
          ImGui::InputFloat("##alpha", &dh_table[row][1], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(3);
          ImGui::InputFloat("##d", &dh_table[row][2], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(4);
          ImGui::InputFloat("##theta", &dh_table[row][3], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(5);
          ImGui::InputFloat("##up", &dh_table[row][4], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(6);
          ImGui::InputFloat("##down", &dh_table[row][5], 0.0f, 1.0f, "%.2f");
          ImGui::PopID();
        }
        ImGui::EndTable();
      }
      ImGui::SameLine();

      ImGui::BeginGroup(); // Lock X position
      if (ImGui::Button("Initial_KR5"))
      { // Btn Init Callback
        if( robot == nullptr) // initial robot first
        {
          //robot = std::make_unique<rb::kin::Artic>();
          robot = std::unique_ptr<rb::kin::Artic> {new rb::kin::Artic};
        }
        rb::math::VectorX a0 = robot->getA();
        rb::math::VectorX al0 = robot->getAlpha();
        rb::math::VectorX d0 = robot->getD();
        rb::math::VectorX th0 = robot->getTheta();
        rb::math::VectorX up0 = robot->getUpLimit();
        rb::math::VectorX low0 = robot->getLowLimit();

        for(int jn=0; jn < 6; jn++)
        {
          dh_table[jn][0] = a0[jn];
          dh_table[jn][1] = al0[jn];
          dh_table[jn][2] = d0[jn];
          dh_table[jn][3] = th0[jn];
          dh_table[jn][4] = up0[jn];
          dh_table[jn][5] = low0[jn];
        }
      } ImGui::SameLine();
      if (ImGui::Button("Reset_Config"))
      {
        // get data from dhTable
        int num_row = dh_table.size(); //(ui->dhTableWidget->rowCount());

        rb::math::VectorX a0(num_row);
        rb::math::VectorX alpha0(num_row);
        rb::math::VectorX d0(num_row);
        rb::math::VectorX th0(num_row);
        rb::math::VectorX up0(num_row);
        rb::math::VectorX low0(num_row);

        for (int i = 0; i < num_row; ++i)
        {
          // get link length A from ui
          a0[i] = dh_table[i][0];

          // get link twist(alpha) from ui
          alpha0[i] = dh_table[i][1];

          // get link offset (d) from ui
          d0[i] = dh_table[i][2];

          // get joint angle (theta) form ui
          th0[i] = dh_table[i][3];

          // get limits from ui
          up0[i] = dh_table[i][4];
          low0[i] = dh_table[i][5];
        }

        robot = std::unique_ptr<rb::kin::Artic>{new rb::kin::Artic(a0, alpha0, d0,
                                                                   th0, up0, low0)};
      }
      if (ImGui::Button("Forward_Kin"))
      {
        rb::math::VectorX th(6);
        th << dh_table[0][3], dh_table[1][3], dh_table[2][3],
              dh_table[3][3], dh_table[4][3], dh_table[5][3];

        pose_tcp = robot->forwardKin(th);

        // output to ui
        cart_inp[0] = pose_tcp.x;
        cart_inp[1] = pose_tcp.y;
        cart_inp[2] = pose_tcp.z;
        coor_inp[0] = pose_tcp.c;
        coor_inp[1] = pose_tcp.b;
        coor_inp[2] = pose_tcp.a;

#ifndef NDEBUG
        // get pose of jnt
        for (int i = 0; i < robot->getDOF(); ++i)
        {
          rb::kin::ArmPose jnt_pos = robot->getJointPos(i);
          printf("Cartesian of Joint #%d: %.3f, %.3f, %.3f\n", i, jnt_pos.x, jnt_pos.y, jnt_pos.z);
        }
#endif
      } ImGui::SameLine();
      if (ImGui::Button("Inversed_Kin"))
      {
        // get tcp position & orientation from ui
        double px = cart_inp[0];
        double py = cart_inp[1];
        double pz = cart_inp[2];

        double row = coor_inp[2];
        double pitch = coor_inp[1];
        double yaw = coor_inp[0];

        rb::math::VectorX q(robot->getDOF());
        rb::kin::IK_RESULT check = robot->inverseKin(px, py, pz, row, pitch,
                                                     yaw, q, joint_value);

        switch (check)
        {
        case rb::kin::IK_RESULT::IK_COMPLETE:
          // ik_result_str = "Find Solutions.";
          std::strncpy(ik_result_str, "Find Solutions.", sizeof(ik_result_str) - 1);
          break;
        case rb::kin::IK_RESULT::IK_NO_SOLUTION:
          std::strncpy(ik_result_str, "No Solutions.", sizeof(ik_result_str) - 1);
          break;
        case rb::kin::IK_RESULT::IK_ANGLE_LIMIT:
          std::strncpy(ik_result_str, "Joint Limit.", sizeof(ik_result_str) - 1);
          break;
        case rb::kin::IK_RESULT::IK_SINGULAR:
          std::strncpy(ik_result_str, "Singular Point Reach!.", sizeof(ik_result_str) - 1);
          break;
        case rb::kin::IK_RESULT::IK_INPUT_INVALID:
          std::strncpy(ik_result_str, "Input Invalid!.", sizeof(ik_result_str) - 1);
          break;
        }
        ik_result_str[sizeof(ik_result_str) - 1] = '\0'; // Ensure null-termination

        // show most fit solution.
        sol_current_idx = joint_value.fit;

        // output all solutions to ui.
        for (int row = 0; row < 8; ++row)
        {
          for (int col = 0; col < 6; ++col)
          {
            sol_table[row][col] = joint_value.axis_value(row, col);
          }
        }
      }
      ImGui::SeparatorText("TCP");

      ImGui::Columns(3);  // Divide the layout into columns

      ImGui::PushItemWidth(65.0);
      ImGui::Text("X");
      ImGui::InputFloat("##X", &cart_inp[0]);
      ImGui::Text("Yaw/Rx");
      ImGui::InputFloat("##Yaw", &coor_inp[0]);
      ImGui::PopItemWidth();
      ImGui::NextColumn();

      ImGui::PushItemWidth(65.0);
      ImGui::Text("Y");
      ImGui::InputFloat("##Y", &cart_inp[1]);
      ImGui::Text("Pitch/Ry");
      ImGui::InputFloat("##Pitch", &coor_inp[1]);
      ImGui::PopItemWidth();
      ImGui::NextColumn();

      ImGui::PushItemWidth(65.0);
      ImGui::Text("Z");
      ImGui::InputFloat("##Z", &cart_inp[2]);
      ImGui::Text("Roll/Rz");
      ImGui::InputFloat("##Roll", &coor_inp[2]);
      ImGui::PopItemWidth();

      ImGui::Columns(1); // Reset column layout
      ImGui::Separator();

      ImGui::Indent(20.0f);
      ImGui::TextColored( ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Result: %s", ik_result_str);
      ImGui::Unindent();

      static ImGuiComboFlags comb_flags = 0;
      //comb_flags &= ImGuiComboFlags_PopupAlignLeft;
      ImGui::SetNextItemWidth(150.0f);
      if (ImGui::BeginCombo("Best Fit", combo_preview_value, comb_flags))
      {
        for (int n = 0; n < IM_ARRAYSIZE(sol_combText); n++)
        {
          const bool is_selected = (sol_current_idx == n);
          if (ImGui::Selectable(sol_combText[n], is_selected))
            sol_current_idx = n;

          // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
          if (is_selected)
            ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }

      ImGui::Checkbox("Gizmo Window (Testing only)", &show_gizmo_window);      // Edit bools storing gizmo window open/close state
      ImGui::EndGroup();
      ImGui::Separator();

      if (ImGui::BeginTable("Solution-Table", 8, ImGuiTableFlags_Borders, ImVec2(680, 120)))
      {
        ImGui::TableSetupColumn("Sol_1");
        ImGui::TableSetupColumn("Sol_2");
        ImGui::TableSetupColumn("Sol_3");
        ImGui::TableSetupColumn("Sol_4");
        ImGui::TableSetupColumn("Sol_5");
        ImGui::TableSetupColumn("Sol_6");
        ImGui::TableSetupColumn("Sol_7");
        ImGui::TableSetupColumn("Sol_8");
        ImGui::TableHeadersRow();

        for (int row = 0; row < 6; row++)
        {
          ImGui::TableNextRow();

          ImGui::PushID(row);
          for (int col = 0; col < 8; col++)
          {
            // Setup ItemWidth once (instead of setting up every time, which is also possible but less efficient)
            if (row == 0)
            {
              ImGui::TableSetColumnIndex(col);
              //ImGui::PushItemWidth(-FLT_MIN); // Right-aligned
            }
            ImGui::TableSetColumnIndex(col);
            ImGui::Text("%f", sol_table[col][row]);
          }
          ImGui::PopID();
        }
        ImGui::EndTable();
      }

      if (show_gizmo_window && g_drawlist)
      {
        // try to draw Links with ImGuizmo
        ImU32 link_color = ImGui::GetColorU32(ImVec4(1.0f, 0.5f, 0.0f, 1.0f));
        ImU32 jnt_color = ImGui::GetColorU32(ImVec4(0.3f, 0.16, 0.8f, 1.0f));
        matrix_t viewProjection = *(matrix_t *)cameraView * *(matrix_t *)cameraProjection;
        vec_t frustum[6];
        ComputeFrustumPlanes(frustum, viewProjection.m16);
        matrix_t res = *(matrix_t *)identityMatrix * viewProjection;

        // Get postion of each joint and store in vec_t
        rb::kin::ArmPose jnt_pos[6];
        vec_t jnt_xyz[robot->getDOF() + 1];
        jnt_xyz[0] = makeVect(0.f, 0.f, 0.f);   // Add origin (base location).
        for (int i = 0; i < robot->getDOF(); ++i)
        {
          jnt_pos[i] = robot->getJointPos(i);
          jnt_xyz[i+1] = makeVect(jnt_pos[i].y / 250.f, jnt_pos[i].z / 250.f, jnt_pos[i].x / 250.f);
          // draw links
          g_drawlist->AddLine(worldToPos(jnt_xyz[i], res), worldToPos(jnt_xyz[i+1], res), link_color, 8.0f);
        }
        for (auto& i : jnt_xyz)
        { // draw joints
          g_drawlist->AddCircleFilled(worldToPos(i, res), 6, jnt_color);
        }
        //g_drawlist->AddLine(ImVec2(150.5f, 200.5f), ImVec2(500.0f, 500.0f), line_color, 5.0f);
      }

      ImGui::End();   // Begin KinDemo
    }

    //ImGui::End();   // DockSpace Demo

  }
}
