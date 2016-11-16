//#include "GLHelpers.h"
#include "GLFont.h"

#include <cassert>
#include <cmath> 
#include <map>


// the fonts defined in these headers are derived from Bitstream Vera fonts. See http://www.gnome.org/fonts/ for license and details



#include "sans.h"
#include "mono.h"
#include "serif.h"


using namespace std;

namespace GLXInterface {


void glSetFont( const std::string &fontname ) {
    
  if(GLXInterface::data.fonts.count(fontname) > 0)
        data.currentFontName = fontname;
}

const std::string& glGetFont() {
    
  return data.currentFontName;
}

std::pair<double,double> glDrawText(const std::string& text, enum TEXT_STYLE style, double spacing, double kerning){
    glPushMatrix();
    if(style == NICE) {
        glPushAttrib( GL_COLOR_BUFFER_BIT | GL_LINE_BIT );
        glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
        glEnable(GL_BLEND);
        glEnable(GL_LINE_SMOOTH);
        glLineWidth(1);
    }
    glEnableClientState(GL_VERTEX_ARRAY);

    // figure out which operation to do on the Char (yes, this is a pointer to member function :)
    void (Font::* operation)(const char c) const;
    switch(style){
        case FILL: operation = &Font::fill;
            break;
        case OUTLINE: operation = &Font::outline;
            break;
        case NICE: operation = &Font::draw;
            break;
        default: assert(false);
    }

    int lines = 0;
    double max_total = 0;
    double total=0;
    const Font * font = data.currentFont();
    const Font::Char * space = font->findChar(' ');
    const double tab_width = 8 * ((space)?(space->advance):1);
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            glTranslated(-total,-spacing, 0);
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        if(c == '\t'){
            const float advance = tab_width - std::fmod(total, tab_width);
            total += advance;
            glTranslated(advance, 0, 0);
            continue;
        }
        const Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        (font->*operation)(c);

        double w = ch->advance + kerning;
        glTranslated(w, 0, 0);
        total += w;
    }

    glDisableClientState(GL_VERTEX_ARRAY);
    if(style == NICE){
        glPopAttrib();
    }
    glPopMatrix();

    max_total = std::max(total, max_total);
    return std::make_pair(max_total, (lines+1)*spacing);
}

std::pair<double, double> glGetExtends(const std::string & text, double spacing, double kerning)
{
    int lines = 0;
    double max_total = 0;
    double total=0;
    const Font* font = data.currentFont();
    for (size_t i=0; i<text.length(); ++i) {
        char c = text[i];
        if (c == '\n') {
            max_total = std::max(max_total, total);
            total = 0;
            ++lines;
            continue;
        }
        const Font::Char * ch = font->findChar(c);
        if(!ch){
            c = toupper(c);
            ch = font->findChar(c);
            if(!ch) {
                c = '?';
                ch = font->findChar(c);
            }
        }
        if(!ch)
            continue;
        total += ch->advance + kerning;
    }
    max_total = std::max(total, max_total);
    return std::make_pair(max_total, (lines+1)*spacing);
}

}