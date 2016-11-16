#ifndef __GUI_READLINE_H
#define __GUI_READLINE_H

#include <string>

// This file is small, but due to static emmebrs, requires a .cpp, hence the GUI_readline.cpp
namespace Persistence
{

	class spawn_readline_thread
	{
		public:
			spawn_readline_thread(const std::string&);
			~spawn_readline_thread();

		private:
			// unfortunately the following stuff is static and MUST be initialized in a source file.............
			static bool running;
			static bool quit;
			static std::string quit_callback;
			pthread_t cmd;
			bool 	  none;
			static  void* proc(void*);
	};


	class readline_in_current_thread
	{
		public:
			void poll();
			readline_in_current_thread(const std::string&);
			~readline_in_current_thread();
		private:
			static void lineread(char*);	
			static std::string quit_callback;


	};
};
#endif