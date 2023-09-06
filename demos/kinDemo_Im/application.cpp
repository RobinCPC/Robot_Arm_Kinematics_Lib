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

float objectMatrix[4][16] =
{
  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    2.f, 0.f, 0.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    2.f, 0.f, 2.f, 1.f },

  { 1.f, 0.f, 0.f, 0.f,
    0.f, 1.f, 0.f, 0.f,
    0.f, 0.f, 1.f, 0.f,
    0.f, 0.f, 2.f, 1.f }
};

static const float identityMatrix[16] =
{ 1.f, 0.f, 0.f, 0.f,
  0.f, 1.f, 0.f, 0.f,
  0.f, 0.f, 1.f, 0.f,
  0.f, 0.f, 0.f, 1.f
};

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
  bool show_gizmo_window = false;

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

      ImGui::End();
    }

    //ImGui::End();   // DockSpace Demo

  }
}
