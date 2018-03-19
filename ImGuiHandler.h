#ifndef SOLEIL__IMGUIHANDLER_H_
#define SOLEIL__IMGUIHANDLER_H_

#include <functional>
#include <iostream>
#include <osg/Camera>
#include <osgGA/GUIEventHandler>
#include <osgViewer/ViewerEventHandlers>

#include "imgui.h"

class ImGUIEventHandler : public osgGA::GUIEventHandler
{
private:
  double       time;
  bool         mousePressed[3];
  float        mouseWheel;
  bool         initialized;
  unsigned int contextID;

  std::function<void(void)> drawui;

private:
  GLuint FontTexture;
  GLuint ShaderHandle;
  GLuint VertHandle;
  GLuint FragHandle;
  GLuint AttribLocationTex;
  GLuint AttribLocationProjMtx;
  GLuint AttribLocationPosition;
  GLuint AttribLocationUV;
  GLuint AttribLocationColor;
  GLuint VboHandle;
  GLuint VaoHandle;
  GLuint ElementsHandle;

protected:
  void initialize(osg::RenderInfo& renderInfo);

public:
  ImGUIEventHandler(
    std::function<void(void)> drawui = std::function<void(void)>());
  ~ImGUIEventHandler();

  void newFrame(osg::RenderInfo& renderInfo);
  void render(osg::RenderInfo& renderInfo);

  bool handle(const osgGA::GUIEventAdapter& eventAdapter,
              osgGA::GUIActionAdapter& /*actionAdapter*/,
              osg::Object* /*object*/, osg::NodeVisitor* /*nodeVisitor*/);
};

class ImGUINewFrame : public osg::Camera::DrawCallback
{
  ImGUIEventHandler& imguiHandler;

public:
  ImGUINewFrame(ImGUIEventHandler& imguiHandler)
    : imguiHandler(imguiHandler)
  {
  }

  void operator()(osg::RenderInfo& renderInfo) const override
  {
    imguiHandler.newFrame(renderInfo);
  }
};

class ImGUIRender : public osg::Camera::DrawCallback
{
  ImGUIEventHandler& imguiHandler;

public:
  ImGUIRender(ImGUIEventHandler& imguiHandler)
    : imguiHandler(imguiHandler)
  {
  }

public:
  void operator()(osg::RenderInfo& renderInfo) const override;
};

#endif /* SOLEIL__IMGUIHANDLER_H_ */
