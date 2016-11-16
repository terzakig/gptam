#include "GStringUtil.h"

using namespace std;

namespace Persistence 
{

string UncommentString(string s)
{
  //int n = s.find("//");
  //return s.substr(0,n);

  int q=0;

  for(string::size_type n=0; n < s.size(); n++)
  {
  	if(s[n] == '"')
		q = !q;

	if(s[n] == '/' && !q)
	{
		if(n < s.size() -1 && s[n+1] == '/')
			return s.substr(0, n);
	}
  }

  return s;
};

vector<string> ChopAndUnquoteString(string s)
{
  vector<string> v;
  string::size_type nPos=0;
  string::size_type nLength = s.length();
  while(1)
    {
    string sTarget;
    char cDelim;
    // Get rid of leading whitespace:  
    while((nPos<nLength)&&(s[nPos]==' '))
      nPos++;
    if(nPos==nLength)
      return v;
    
    // First non-whitespace char...
    if(s[nPos]!='\"')
      cDelim=' ';
    else
      {
	cDelim = '\"';
	nPos++;
      }
    for (; nPos < nLength; ++nPos) {
	char c = s[nPos];
	if (c == cDelim)
	    break;
	if (cDelim == '"' && nPos+1<nLength && c == '\\') {
	    char escaped = s[++nPos];
	    switch (escaped) {
	    case 'n': c = '\n'; break;
	    case 'r': c = '\r'; break;
	    case 't': c = '\t'; break;
	    default: c = escaped; break;
	    }
	}
	sTarget+=c;
    }
    v.push_back(sTarget);

    if(cDelim=='\"')
      nPos++;
    }
};

}