
#ifndef PER_GUI_H
#define PER_GUI_H

#include "PVars.h"

#include <vector>
#include <iostream>
#include <set>

namespace Persistence
{
	
	// GUICallBackProc is a pointer to void functions that receive two strings as arguments: "sCommand" and "Params" 
	// In other words, this our delegate for GUI event-handling functions
	typedef void (*GUICallbackProc)(void* ptr, std::string sCommand, std::string sParams);

	//This is the structure that registers a "GUI Command". In effect, it contains a callback pointer
	// and a self-pointer (which becomes useful along the way)
	typedef struct 
	{
	  GUICallbackProc cbp;
	  void* thisptr;
	} CallbackInfoStruct;

	typedef std::vector<CallbackInfoStruct> CallbackVector;


	// GUI_impl is essentially a class with all the GUI operation,
	// While the GUI class can be actually spawned in many different threads.
	// So, each many GUI instantiations can potentially share a single GUI_impl object. 
	class GUI_impl;

	
	class GUI
	{
		public:
			// The function that returns the shared GUI implementation class. 
			static GUI_impl& I();

			GUI();
			
			/// Register a command name and callback to the GUI_impl object
			void RegisterCommand(std::string sCommandName, GUICallbackProc callback, void* thisptr=NULL);
			/// Release all commands associated with "thisptr"
			void UnRegisterAllCommands(void* thisptr);
			/// Unregister specific command from THIS GUI by name
			void UnRegisterCommand(std::string sCommandName, void* thisptr);
			/// Unregister command by name regardless of GUI object
			void UnRegisterCommand(std::string sCommandName);
			// Thisd is the function that parses lines in configuration files - Awsome work by Rosten!
			void ParseLine(std::string s, bool bSilentFailure = false);
			// Parse a configuration file stream, line-by-line
			void ParseStream(std::istream& is);
			// load a configuration file!
			void LoadFile(std::string sFileName);
			// execute callback (if the command has indeed registered callbakcs in the GUI_impl object)
			bool CallCallbacks(std::string sCommand, std::string sParams);
			
			void SetupReadlineCompletion();

			/// Start a thread which parses user input from the console.
			/// Uses libreadline if configured, or just plain old iostream readline
			void StartParserThread();
			/// Stop the console parser thread, if running
			/// Top tip: This is static so that it can be used with atexit(void*)
			static void StopParserThread();

			/// parse command line arguments for GVar values. It expects the form --name value and will stop
			/// parsing when this form is not true anymore. possible cases are a single --, an argument, etc..
			/// if it finds an argument --exec it interprets the next argument as a file name to load via LoadFile
			/// @arg argc total number of arguments from main argc
			/// @arg argv pointer to array of strings from main argv
			/// @arg start first argument to look at to be compatible with other parsers
			/// @arg prefix the prefix to use. note that everything after the prefix will be interpreted as part of the variable name
			/// @arg execKeyword keyword to use to trigger execution of the file given as parameter with LoadFile
			/// @return
			int parseArguments( const int argc, char * argv[], int start = 1, const std::string prefix = "--", const std::string execKeyword = "exec" );
	};

}

#endif