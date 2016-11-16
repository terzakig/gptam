// George Terzakis 2016
//
// University of Portsmouth
//
// Code based on PTAM by Klein and Murray (Copyright 2008 Isis Innovation Limited)

#ifndef __GL_WINDOW_MENU_H
#define __GL_WINDOW_MENU_H

// A simple gvars-driven menu system for GLWindow2
// N.b. each GLWindowMenu class internally contains sub-menus

#include <vector>
#include <map>
#include "Persistence/PVars.h"
#include "GLWindow2.h"

class GLWindowMenu
{
 public:
  
  GLWindowMenu(std::string sName, std::string sTitle);
  ~GLWindowMenu();
  void Render(int nTop, int nHeight, int nWidth, GLWindow2 &glw);
  void FillBox(int l, int r, int t, int b);
  void LineBox(int l, int r, int t, int b);
  
  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  
  bool HandleClick(int button, int state, int x, int y);

  

 private:
  enum MenuItemType { Button, Toggle, Monitor, Slider };
  
  struct MenuItem
  {
    MenuItemType type;
    std::string sName;
    std::string sParam;
    std::string sNextMenu;
    Persistence::pvar_int gvnIntValue;  // Not used by all, but used by some (that's a shortcut for pvar2<int>...
    int min;
    int max;
  };
  
  struct SubMenu
  {
    std::vector<MenuItem> mvItems;
  };
  
  std::map<std::string, SubMenu> mmSubMenus;
  std::string msCurrentSubMenu;
  std::string msName;
  std::string msTitle;

  
  int mnWidth;
  int mnMenuTop;
  int mnMenuHeight;
  int mnTextOffset;
  
  Persistence::pvar_int mgvnEnabled;
  Persistence::pvar_int mgvnMenuItemWidth;
  Persistence::pvar_int mgvnMenuTextOffset;
  
  int mnLeftMostCoord;
  
};

#endif










