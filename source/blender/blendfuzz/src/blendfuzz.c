#include <stdio.h>
#include <string.h>

#include "blendfuzz.h"

#include "BLI_rect.h"
#include "../../imbuf/IMB_imbuf.h"
#include "../../imbuf/IMB_imbuf_types.h"

#include "BLI_fileops.h"
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>

int main(int argc, char** argv) {
    if(argc != 3) {
        printf("Usage: %s option target\n", argv[0]);
        return 0;
    }

    char *option = argv[1];
    char *filename = argv[2];

    if(strcmp(option, "imbuf") == 0) {
        const int file = BLI_open(filename, O_BINARY | O_RDONLY, 0);
        if(file == -1) {
            printf("ERR\n");
            return 1;
        }
        ImBuf *imbuf = IMB_loadifffile(file, filename, IB_rect, NULL, filename);
        if(imbuf == NULL) {
            printf("ERR\n");
            close(file);
            return 2;
        }
        close(file);
    }

    //else if(strcmp()) {
    //}

    return 0;
}
