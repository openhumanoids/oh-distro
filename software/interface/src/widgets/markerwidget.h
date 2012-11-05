#ifndef GTKMM_CUSTOM_WIDGET_MYWIDGET_H
#define GTKMM_CUSTOM_WIDGET_MYWIDGET_H

#include <gtkmm/widget.h>

/*struct GLConfigUtil
{
	static void print_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig,
			const char* attrib_str,
			int attrib,
			bool is_boolean);

	static void examine_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig);
};

//
// Print a configuration attribute.
//
void GLConfigUtil::print_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig,
		const char* attrib_str,
		int attrib,
		bool is_boolean)
{
	int value;

	if (glconfig->get_attrib(attrib, value))
	{
		std::cout << attrib_str << " = ";
		if (is_boolean)
			std::cout << (value == true ? "true" : "false") << std::endl;
		else
			std::cout << value << std::endl;
	}
	else
	{
		std::cout << "*** Cannot get "
				<< attrib_str
				<< " attribute value\n";
	}
}

//
// Print configuration attributes.
//
void GLConfigUtil::examine_gl_attrib(const Glib::RefPtr<const Gdk::GL::Config>& glconfig)
{
	std::cout << "\nOpenGL visual configurations :\n\n";

	std::cout << "glconfig->is_rgba() = "
			<< (glconfig->is_rgba() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->is_double_buffered() = "
			<< (glconfig->is_double_buffered() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->is_stereo() = "
			<< (glconfig->is_stereo() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_alpha() = "
			<< (glconfig->has_alpha() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_depth_buffer() = "
			<< (glconfig->has_depth_buffer() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_stencil_buffer() = "
			<< (glconfig->has_stencil_buffer() ? "true" : "false")
			<< std::endl;
	std::cout << "glconfig->has_accum_buffer() = "
			<< (glconfig->has_accum_buffer() ? "true" : "false")
			<< std::endl;

	std::cout << std::endl;

	print_gl_attrib(glconfig, "Gdk::GL::USE_GL",           Gdk::GL::USE_GL,           true);
	print_gl_attrib(glconfig, "Gdk::GL::BUFFER_SIZE",      Gdk::GL::BUFFER_SIZE,      false);
	print_gl_attrib(glconfig, "Gdk::GL::LEVEL",            Gdk::GL::LEVEL,            false);
	print_gl_attrib(glconfig, "Gdk::GL::RGBA",             Gdk::GL::RGBA,             true);
	print_gl_attrib(glconfig, "Gdk::GL::DOUBLEBUFFER",     Gdk::GL::DOUBLEBUFFER,     true);
	print_gl_attrib(glconfig, "Gdk::GL::STEREO",           Gdk::GL::STEREO,           true);
	print_gl_attrib(glconfig, "Gdk::GL::AUX_BUFFERS",      Gdk::GL::AUX_BUFFERS,      false);
	print_gl_attrib(glconfig, "Gdk::GL::RED_SIZE",         Gdk::GL::RED_SIZE,         false);
	print_gl_attrib(glconfig, "Gdk::GL::GREEN_SIZE",       Gdk::GL::GREEN_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::BLUE_SIZE",        Gdk::GL::BLUE_SIZE,        false);
	print_gl_attrib(glconfig, "Gdk::GL::ALPHA_SIZE",       Gdk::GL::ALPHA_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::DEPTH_SIZE",       Gdk::GL::DEPTH_SIZE,       false);
	print_gl_attrib(glconfig, "Gdk::GL::STENCIL_SIZE",     Gdk::GL::STENCIL_SIZE,     false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_RED_SIZE",   Gdk::GL::ACCUM_RED_SIZE,   false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_GREEN_SIZE", Gdk::GL::ACCUM_GREEN_SIZE, false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_BLUE_SIZE",  Gdk::GL::ACCUM_BLUE_SIZE,  false);
	print_gl_attrib(glconfig, "Gdk::GL::ACCUM_ALPHA_SIZE", Gdk::GL::ACCUM_ALPHA_SIZE, false);

	std::cout << std::endl;
}*/

class MarkerWidget : public Gtk::Widget
{
public:
	MarkerWidget(std::string name);
	virtual ~MarkerWidget();

	void setActual(double percentage);
	void setCommanded(double percentage);
	double getCommanded();
	int getMinHeight();

protected:

	//Overrides:
	virtual void on_size_request(Gtk::Requisition* requisition);
	virtual void on_size_allocate(Gtk::Allocation& allocation);
	virtual void on_map();
	virtual void on_unmap();
	virtual void on_realize();
	virtual void on_unrealize();
	virtual bool on_expose_event(GdkEventExpose* event);

	Glib::RefPtr<Gdk::Window> m_refGdkWindow;


	double markAtPercentage;
	double actualAtPercentage;
	int m_scale;
	std::string name;
};

#endif //GTKMM_CUSTOM_WIDGET_MYWIDGET_H
