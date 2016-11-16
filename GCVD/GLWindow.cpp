#include "GLWindow.h"
#include <exception>

//#include <X11/Xlib.h>
//#include <X11/keysym.h>
//#include <GL/glx.h>


GLXInterface::Exceptions::GLWindow::CreationError::CreationError(std::string w)
{
    
    
    //what() = "GLWindow creation error: " + w;
}

GLXInterface::Exceptions::GLWindow::RuntimeError::RuntimeError(std::string w)
{
    //std::string str = 
    //what() = "GLWindow error: " + w;
}



void GLXInterface::GLWindow::init(const cv::Size2i& size, int bpp, const std::string& title, const std::string& disp)
{
    Display* display = XOpenDisplay(disp == "" ? NULL : const_cast<char*>(disp.c_str()));
    if (display == 0)
	throw Exceptions::GLWindow::CreationError("Cannot open X display");

    int visualAttributes[] = {
	GLX_RGBA,
	GLX_DOUBLEBUFFER,
	GLX_RED_SIZE,      bpp/3,
	GLX_GREEN_SIZE,    bpp/3,
	GLX_BLUE_SIZE,     bpp/3,
	GLX_DEPTH_SIZE,    8,
	GLX_STENCIL_SIZE, 8,
	None
    };
    XVisualInfo* visualInfo = glXChooseVisual(display, DefaultScreen(display),visualAttributes);
    if(visualAttributes == 0) {
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXChooseVisual failed");
    }

    Window rootWindow = RootWindow(display, visualInfo->screen);
    XWindowAttributes windowAttributes;

    XGetWindowAttributes(display, rootWindow, &windowAttributes);

    XSetWindowAttributes attributes;
    attributes.border_pixel = 0;
    attributes.colormap = XCreateColormap(display, rootWindow, visualInfo->visual, AllocNone);
    attributes.event_mask = KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | PointerMotionMask | StructureNotifyMask | ExposureMask;

    Window window = XCreateWindow(display,
				  rootWindow,
				  0, 0, size.width, size.height,
				  0, visualInfo->depth,
				  InputOutput,
				  visualInfo->visual,
				  CWBorderPixel | CWColormap | CWEventMask,
				  &attributes);
    XStoreName(display, window, title.c_str());
    XClassHint classHint;
	char res_name[] = "cvd";
    classHint.res_class = res_name;
    classHint.res_name = (char *)title.c_str();
    XSetClassHint(display, window, &classHint);
    XMapWindow(display, window);
    XEvent ev;
    do {
        XNextEvent(display,&ev);
    } while (ev.type != MapNotify);

    Atom delete_atom = XInternAtom(display, "WM_DELETE_WINDOW", True);
    XSetWMProtocols(display, window, &delete_atom, 1);

    GLXContext context = glXCreateContext(display, visualInfo, 0, True);
    if (context == 0) {
	XDestroyWindow(display, window);
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXCreateContext failed");
    }

    if (glXMakeCurrent(display, window, context) == False) {
	glXDestroyContext(display, context);
	XDestroyWindow(display, window);
	XCloseDisplay(display);
	throw Exceptions::GLWindow::CreationError("glXMakeCurrent failed");
    }
    glLoadIdentity();
    glViewport(0, 0, size.width, size.height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glColor3f(1.0f,1.0f,1.0f);
    glRasterPos2f(-1, 1);
    glOrtho(-0.375, size.width-0.375, size.height-0.375, -0.375, -1 , 1); //offsets to make (0,0) the top left pixel (rather than off the display)
    glPixelZoom(1,-1);

    XColor black = {0, 0, 0, 0, 0, 0};
    XFontStruct* fixed = XLoadQueryFont(display, "-misc-fixed-medium-r-*-*-12-*-*-*-*-*-*-1" );
    Cursor null_cursor = XCreateGlyphCursor(display, fixed->fid, fixed->fid, ' ', ' ', &black, &black);
    XFreeFont(display, fixed);

    state = new State();
    state->size = size;
    state->title = title;
    state->display = display;
    state->window = window;
    state->delete_atom = delete_atom;
    state->null_cursor = null_cursor;
    state->context = context;
}

GLXInterface::GLWindow::~GLWindow()
{
    glXMakeCurrent(state->display, None, 0);
    glXDestroyContext(state->display, state->context);

    XUnmapWindow(state->display, state->window);
    XDestroyWindow(state->display, state->window);
    XCloseDisplay(state->display);
    delete state;
}

cv::Size2i GLXInterface::GLWindow::size() const { return state->size; }

void GLXInterface::GLWindow::set_size(const cv::Size2i &s_){
    // we don't set state->size here, so that it changes through the event system
    // and we react to it there
    XResizeWindow(state->display, state->window, s_.width, s_.height);
}

cv::Point2i GLXInterface::GLWindow::position() const { return state->position; }

void GLXInterface::GLWindow::set_position(const cv::Point2i &p_){
    
    state->position = p_;
    XMoveWindow(state->display, state->window, p_.x, p_.y);
	XFlush(state->display);
}

void GLXInterface::GLWindow::set_cursor_position(const cv::Point2i &where)
{
    XWarpPointer(state->display, None, state->window, 0, 0, 0, 0, where.x, where.y);
}

cv::Point2i GLXInterface::GLWindow::cursor_position() const
{
    Window wtmp;
    int itmp;
    unsigned int utmp;
    cv::Point2i where;
    XQueryPointer(state->display, state->window, &wtmp, &wtmp, &itmp, &itmp, &where.x, &where.y, &utmp);
    return where;
}

void GLXInterface::GLWindow::show_cursor(bool show)
{
    if (show)
	XUndefineCursor(state->display, state->window);
    else
	XDefineCursor(state->display, state->window, state->null_cursor);
}

std::string GLXInterface::GLWindow::title() const
{
    return state->title;
}

void GLXInterface::GLWindow::set_title(const std::string& title)
{
    state->title = title;
    XStoreName(state->display, state->window, title.c_str());
}

void GLXInterface::GLWindow::swap_buffers()
{
    glXSwapBuffers(state->display, state->window);
}

inline int convertButton(unsigned int button)
{
  switch (button) {
    case Button1: return GLXInterface::GLWindow::BUTTON_LEFT;
    case Button2: return GLXInterface::GLWindow::BUTTON_MIDDLE;
    case Button3: return GLXInterface::GLWindow::BUTTON_RIGHT;
    case Button4: return GLXInterface::GLWindow::BUTTON_WHEEL_UP;
    case Button5: return GLXInterface::GLWindow::BUTTON_WHEEL_DOWN;
  }
  return 0;
}

inline int convertButtonState(unsigned int state)
{
  int ret = 0;
  if (state & Button1Mask) ret |= GLXInterface::GLWindow::BUTTON_LEFT;
  if (state & Button2Mask) ret |= GLXInterface::GLWindow::BUTTON_MIDDLE;
  if (state & Button3Mask) ret |= GLXInterface::GLWindow::BUTTON_RIGHT;
  if (state & ControlMask) ret |= GLXInterface::GLWindow::BUTTON_MOD_CTRL;
  if (state & ShiftMask) ret |= GLXInterface::GLWindow::BUTTON_MOD_SHIFT;
  return ret;
}

void GLXInterface::GLWindow::handle_events(EventHandler& handler)
{
    XEvent event;
	KeySym k;
    while (XPending(state->display)) {
	XNextEvent(state->display, &event);
	switch (event.type) {
	case ButtonPress:
	    handler.on_mouse_down(*this, cv::Point2i(event.xbutton.x, event.xbutton.y),
				  convertButtonState(event.xbutton.state), convertButton(event.xbutton.button));
	    break;
	case ButtonRelease:
	    handler.on_mouse_up(*this, cv::Point2i(event.xbutton.x, event.xbutton.y),
				convertButtonState(event.xbutton.state), convertButton(event.xbutton.button));
	    break;
	case MotionNotify:
	    handler.on_mouse_move(*this, cv::Point2i(event.xmotion.x, event.xmotion.y), convertButtonState(event.xbutton.state));
	    break;
	case KeyPress:
		{
		XLookupString(&event.xkey, 0, 0, &k, 0);
	    handler.on_key_down(*this, k);
	    break;
		}
	case KeyRelease:
		XLookupString(&event.xkey, 0, 0, &k, 0);
	    handler.on_key_up(*this, k);
	    break;
	    //case UnmapNotify: active = 0; break;
	    //case MapNotify: active = 1; break;
	case ConfigureNotify:
	    if (event.xconfigure.width != state->size.width || event.xconfigure.height != state->size.height) {
		activate();
		state->size = cv::Size2i(event.xconfigure.width, event.xconfigure.height);
		glViewport(0, 0, state->size.width, state->size.height);
		//glRasterPos2f(0,0);
		//glPixelZoom(float(event.xconfigure.width)/myWidth,-float(event.xconfigure.height)/myHeight);
		handler.on_resize(*this, state->size);
	    }
	    break;
	case Expose:
		handler.on_event(*this, EVENT_EXPOSE);
		break;
	case ClientMessage:
	    if (event.xclient.data.l[0] == (int)state->delete_atom)
		handler.on_event(*this, EVENT_CLOSE);
	    else
		handler.on_event(*this, event.xclient.message_type);
	    break;
	default:
	    handler.on_event(*this, event.type);
	    break;
	}
    }
}

class SaveEvents : public GLXInterface::GLWindow::EventHandler {
private:
    std::vector<GLXInterface::GLWindow::Event>& events;
public:
    SaveEvents(std::vector<GLXInterface::GLWindow::Event>& events_) : events(events_) {}
    
    void on_key_down(GLXInterface::GLWindow&, int key) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::KEY_DOWN;
	e.which = key;
	events.push_back(e);
    }
    
    void on_key_up(GLXInterface::GLWindow&, int key) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::KEY_UP;
	e.which = key;
	events.push_back(e);
    }

    void on_mouse_move(GLXInterface::GLWindow&, cv::Point2i where, int state) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_MOVE;
	e.state = state;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_down(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_DOWN;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_mouse_up(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::MOUSE_UP;
	e.state = state;
	e.which = button;
	e.where = where;
	events.push_back(e);
    }

    void on_resize(GLXInterface::GLWindow&, cv::Size2i size) {
	
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::RESIZE;
	e.size = size;
	events.push_back(e);
    }

    void on_event(GLXInterface::GLWindow&, int event) {
	GLXInterface::GLWindow::Event e;
	e.type = GLXInterface::GLWindow::Event::EVENT;
	e.which = event;
	events.push_back(e);
    }
};

void GLXInterface::GLWindow::get_events(std::vector<Event>& events)
{
    SaveEvents saver(events);
    handle_events(saver);
}

bool GLXInterface::GLWindow::EventSummary::should_quit() const
{
    return key_down.count(XK_Escape) || events.count(GLXInterface::GLWindow::EVENT_CLOSE);
}

class MakeSummary : public GLXInterface::GLWindow::EventHandler {
private:
    GLXInterface::GLWindow::EventSummary& summary;
public:
    MakeSummary(GLXInterface::GLWindow::EventSummary& summary_) : summary(summary_) {}

    void on_key_down(GLXInterface::GLWindow&, int key) {	++summary.key_down[key]; }
    void on_key_up(GLXInterface::GLWindow&, int key) { ++summary.key_up[key]; }
    void on_mouse_move(GLXInterface::GLWindow&, cv::Point2i where, int) { summary.cursor = where; summary.cursor_moved = true; }
    void on_mouse_down(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) { summary.mouse_down[button] = std::make_pair(where,state); }
    void on_mouse_up(GLXInterface::GLWindow&, cv::Point2i where, int state, int button) { summary.mouse_up[button] = std::make_pair(where,state); }
    void on_event(GLXInterface::GLWindow&, int event) { ++summary.events[event]; }
};

void GLXInterface::GLWindow::get_events(EventSummary& summary)
{
    summary.cursor = cursor_position();
    MakeSummary ms(summary);
    handle_events(ms);
}

bool GLXInterface::GLWindow::has_events() const
{
    return XPending(state->display);
}

void GLXInterface::GLWindow::activate()
{
    if (glXMakeCurrent(state->display, state->window, state->context) == False)
	throw Exceptions::GLWindow::RuntimeError("glXMakeCurrent failed");
}
