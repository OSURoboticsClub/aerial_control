/**
 * These stubs are necessary to make newlib work.
 *
 * See
 * http://stackoverflow.com/questions/9632595/compiling-basic-c-file-for-the-arm-processor
 */
void __cxa_pure_virtual() {while(1);}
void _sbrk() {while(1);}
void _exit(int e) {while(1);}
void _kill() {while(1);}
void _getpid() {while(1);}
void _write() {while(1);}
void _close() {while(1);}
void _fstat() {while(1);}
void _isatty() {while(1);}
void _lseek() {while(1);}
void _read() {while(1);}

// Per GCC documentation, this should be a unique value for every shared object.
// We don't have to worry about dynamic linking since this is a main program, so
// we can just zero it.
void *__dso_handle = 0;
