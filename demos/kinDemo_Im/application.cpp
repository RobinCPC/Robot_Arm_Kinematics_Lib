#include "application.h"
// Refernce: Cherno video https://www.youtube.com/watch?v=vWXrFetSH8w

#include "imgui.h"
#include <vector>


namespace MyApp
{
  void RenderUI()
  {


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

    //ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());

    ImGui::Begin("Settings");
    ImGui::Button("Hello ImGui");
    static float value = 0.0f;
    ImGui::DragFloat("Value", &value);
    ImGui::End();   // Settings

    ImGui::Begin("Viewport");
    ImGui::End();   // Viewport

    // 4. Test to build my window and make it full screen
    {
      ImGui::SetNextWindowSize(ImVec2(740, 350), ImGuiCond_Once);
      //ImGui::SetWindowSize(ImVec2(740, 350), ImGuiCond_FirstUseEver);
      ImGui::Begin("KinDemo", nullptr);

      // make dummy DH table by vector, vector float
      static std::vector<std::vector<float>> kr5_table = {
        {  0.0,  0., 339.,  0.,170,-170},
        {  0.0, 90.,   0., 90., 45,-190},
        {250.0,  0.,   0.,  0., 79,-209},
        { 70.0, 90., 250.,  0.,190,-190},
        {  0.0,-90.,   0.,  0.,120,-120},
        {  0.0, 90.,  95.,  0.,350,-350}
      };
      static std::vector<std::vector<float>> sol_table = {
        {  0.0,  0.,  39.,  0.,  170,  170},
        {  0.0, 90.,   0., 90.,   45,  190},
        {  0.0,  0.,   0.,  0.,   79,  209},
        {  0.0, 90.,   0.,  0.,   90,  190},
        {  0.0,-90.,   0.,  0.,   20,   20},
        {  0.0,  0.,  50.,  0.,   90,   90},
        {  0.0, 90.,   0.,  0.,   90,   90},
        {  0.0,  0.,   5.,  0.,  350,   50}
      };
      static float cart_inp[3] = { 0.0, 0.0, 0.0 };
      static float coor_inp[3] = { 0.0, 0.0, 0.0 };

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
          static float dummy_f = 0.0f;
          ImGui::PushID(row);
          ImGui::TableSetColumnIndex(0);
          ImGui::Text("Joint %d", row);
          ImGui::TableSetColumnIndex(1);
          ImGui::InputFloat("##a", &kr5_table[row][0], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(2);
          ImGui::InputFloat("##alpha", &kr5_table[row][1], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(3);
          ImGui::InputFloat("##d", &kr5_table[row][2], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(4);
          ImGui::InputFloat("##theta", &kr5_table[row][3], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(5);
          ImGui::InputFloat("##up", &kr5_table[row][4], 0.0f, 1.0f, "%.2f");
          ImGui::TableSetColumnIndex(6);
          ImGui::InputFloat("##down", &kr5_table[row][5], 0.0f, 1.0f, "%.2f");
          ImGui::PopID();
        }
        ImGui::EndTable();
      }
      ImGui::SameLine();

      ImGui::BeginGroup(); // Lock X position
      if (ImGui::Button("Initial_KR5")) { kr5_table[0][3] += 1.0; } ImGui::SameLine();
      if (ImGui::Button("Reset_Config")) { kr5_table[1][3] += 1.0; }
      if (ImGui::Button("Forward_Kin")) { kr5_table[2][3] += 1.0; } ImGui::SameLine();
      if (ImGui::Button("Inversed_Kin")) { kr5_table[3][3] += 1.0; }
      ImGui::SeparatorText("TCP");
      //ImGui::Separator();

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
      ImGui::TextColored( ImVec4(1.0f, 1.0f, 0.0f, 1.0f), "Find Solution %s", sol_combText[sol_current_idx]);
      ImGui::Unindent();

      static ImGuiComboFlags comb_flags = 0;
      //comb_flags &= ImGuiComboFlags_PopupAlignLeft;
      ImGui::SetNextItemWidth(150.0f);
      if (ImGui::BeginCombo("Best Sol", combo_preview_value, comb_flags))
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


    ImGui::End();   // DockSpace Demo

  }
}
