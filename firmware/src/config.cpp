#include <cassert>

#include "ff/ff.h"

#include "config.h"

namespace config {


static char const * filename = "gpslog.txt";

int get_int(char const * key) {
    FIL file;
    assert(f_open(&file, filename, FA_OPEN_EXISTING | FA_READ) == FR_OK);
    
    bool already_passed_newline = false;
    auto getc = [&]() {
        char res;
        UINT count;
        f_read(&file, &res, 1, &count);
        assert(count == 0 || count == 1);
        if(count == 0) {
            if(already_passed_newline) assert(false);
            already_passed_newline = true;
            return '\n';
        }
        return res;
    };
    
start_of_line:
    {
        int chars_matched = 0;
        while(key[chars_matched]) {
            char c = getc();
            if(c == '\n' || c == '\r') goto start_of_line;
            else if(c == key[chars_matched]) {
                chars_matched++;
            } else goto skip_line;
        }
    }
    
    {
        char c = getc();
        if(c == '\n' || c == '\r') goto start_of_line;
        else if(c == '=') {
        } else goto skip_line;
    }
    
    {
        int res = 0;
        while(true) {
            char c = getc();
            if(c == '\n' || c == '\r') {
                assert(f_close(&file) == FR_OK);
                return res;
            } else if(c >= '0' && c <= '9') {
                res = 10 * res + (c - '0');
            } else assert(false);
        }
    }

skip_line:
    while(true) {
        char c = getc();
        if(c == '\n' || c == '\r') goto start_of_line;
    }
}


}
